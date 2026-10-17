/**
 * @file    app_msg_send.h
 * @brief   消息发送模块 - 管理发送缓冲、重传和 ACK 处理
 *
 * 职责描述：
 *   ✓ 管理待发送帧的缓冲（带重传保护）
 *   ✓ 记录帧的序列号、发送时间、重试次数
 *   ✓ 处理 ACK/NACK 通知和超时重传
 *   ✓ 提供发送状态查询接口
 *   ✓ 周期检测超时和自动重传
 *
 * 分层关系：
 *   app_frame_handler（帧业务处理）
 *       ↓ 调用
 *   app_msg_send（本模块）← 消息发送管理（包含协议打包 + 调制调用）
 *       ↓ 调用 proto_frame_pack
 *   bsw_proto（协议层打包）
 *       ↓ 调用 bsw_bfsk_mod_send
 *   bsw_bfsk_mod（调制发送）
 *       ↓ 调用硬件
 *   AD9833（调制芯片）
 *
 * 使用流程：
 *   1. app_msg_send_init()           - 初始化模块
 *   2. app_msg_send_frame()          - 发送帧（自动保存）
 *   3. app_msg_send_notify_ack()     - 收到 ACK 时调用
 *   4. app_msg_send_notify_nack()    - 收到 NACK 时调用
 *   5. app_msg_send_tick()           - 周期调用（检测超时）
 *
 * @dependency  bsw_proto.h          (帧结构定义 + 打包函数)
 *              bsw_bfsk_mod.h       (调制发送)
 *
 * @example
 *   // 初始化
 *   app_msg_send_init();
 *
 *   // 发送一个帧（内部使用 5 秒超时）
 *   proto_frame_t frame = {...};
 *   app_msg_send_frame(&frame);
 *
 *   // 在定时器中周期调用（例如每 10ms）
 *   void timer_callback(void) {
 *       app_msg_send_tick(get_tick_count());
 *   }
 */

#ifndef APP_MSG_SEND_H
#define APP_MSG_SEND_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "bsw_proto.h"

/* ========== 返回码定义 ========== */

/**
 * @brief 消息发送模块返回码
 */
typedef enum
{
    MSG_SEND_OK             = 0,    /* 操作成功 */
    MSG_SEND_ERR_BUSY       = -1,   /* 上一个帧还在等待 ACK，缓冲区忙 */
    MSG_SEND_ERR_TIMEOUT    = -2,   /* 重传超时（达到最大重试次数） */
    MSG_SEND_ERR_MOD_FAIL   = -3,   /* 调制器发送失败 */
    MSG_SEND_ERR_NOT_INIT   = -4,   /* 模块未初始化 */
    MSG_SEND_ERR_PARAM      = -5,   /* 参数错误 */
} msg_send_ret_t;

/* ========== 配置常量 ========== */

/**
 * @brief 最大重传次数
 * @rationale: 控制重传行为，防止无限重试
 */
#define MSG_SEND_MAX_RETRY          (3)

/**
 * @brief 默认超时时间（毫秒）
 * @rationale: 等待 ACK/NACK 的时间，从帧发送完毕后开始计时，超时则自动重传
 * @value 5000ms（5 秒）- 考虑物理链路延迟 + 对方处理时间 + 回复传输时间
 */
#define MSG_SEND_DEFAULT_TIMEOUT_MS (5000U)

/* ========== 公共 API ========== */

/**
 * @brief 初始化消息发送模块
 * @return void
 * @note  系统启动时调用一次
 *
 * @details
 * 初始化内容：
 * - 清空缓冲区
 * - 设置模块为就绪状态
 */
void app_msg_send_init(void);

/**
 * @brief 发送一个协议帧（带重传保护）
 * @param p_frame 待发送的帧指针
 * @return msg_send_ret_t - 成功返回 MSG_SEND_OK
 *
 * @pre   app_msg_send_init() 已调用
 * @pre   p_frame 指向有效的 proto_frame_t 结构体
 *
 * @details
 * 功能：
 * - 打包帧到内部缓冲区
 * - 立即调用调制器发送帧
 * - 记录序列号、发送时间戳
 * - 等待 ACK/NACK 回复（超时时间固定为 MSG_SEND_DEFAULT_TIMEOUT_MS）
 * - 设置 is_waiting_ack 标志
 *
 * 超时计时：
 * - 起点：bsw_bfsk_mod_send() 返回 OK（帧发送完毕）
 * - 等待对象：ACK 或 NACK 回复
 * - 超时处理：自动重传（最多 MSG_SEND_MAX_RETRY 次）
 *
 * 失败情况：
 * - MSG_SEND_ERR_BUSY：上一个帧还在等待 ACK
 * - MSG_SEND_ERR_MOD_FAIL：调制器拒绝发送
 * - MSG_SEND_ERR_NOT_INIT：模块未初始化
 * - MSG_SEND_ERR_PARAM：参数无效（p_frame==NULL）
 *
 * 线程安全：仅从主任务调用，不支持中断或多任务并发
 */
msg_send_ret_t app_msg_send_frame(const proto_frame_t *p_frame);

/**
 * @brief 发送 ACK/NACK 帧（无需等待对方的 ACK，不重传）
 * @param p_frame 待发送的 ACK/NACK 帧指针
 * @return msg_send_ret_t - 成功返回 MSG_SEND_OK
 *
 * @pre   app_msg_send_init() 已调用
 * @pre   p_frame 指向有效的 proto_frame_t 结构体
 *
 * @details
 * 功能：
 * - 打包帧并立即发送
 * - 不保存到缓冲区，不等待 ACK
 * - 适用于 ACK/NACK 帧（确认帧本身不需要确认）
 *
 * 与 app_msg_send_frame() 的区别：
 * - 本函数：发完即走，不重传
 * - app_msg_send_frame()：带 ACK 等待和重传保障
 *
 * 失败情况：
 * - MSG_SEND_ERR_MOD_FAIL：调制器拒绝发送
 * - MSG_SEND_ERR_NOT_INIT：模块未初始化
 * - MSG_SEND_ERR_PARAM：参数无效（p_frame==NULL）
 *
 * 线程安全：仅从主任务调用，不支持中断或多任务并发
 */
msg_send_ret_t app_msg_send_ack(const proto_frame_t *p_frame);

/**
 * @brief 通知模块收到了 ACK
 * @param seq 帧序列号（来自接收到的 ACK 帧）
 * @return void
 *
 * @details
 * 功能：
 * - 验证 seq 与缓冲区帧的 seq 匹配
 * - 匹配则清空缓冲区，重置状态
 * - 不匹配则忽略（可能是迟到的 ACK）
 *
 * 何时调用：
 * - app_frame_handler 收到 ACK 帧后调用
 * - ACK 帧通常是对方对我们发送的帧的确认
 */
void app_msg_send_notify_ack(uint8_t seq);

/**
 * @brief 通知发送模块收到 NACK
 * @param seq 序列号
 * @note NACK 帧不携带错误码（payload_len = 0），收到即重传
 */
void app_msg_send_notify_nack(uint8_t seq);

/**
 * @brief 周期调用此函数检测超时并自动重传
 * @param now_tick 当前时刻（毫秒计数）
 * @return void
 *
 * @details
 * 功能（仅在 is_waiting_ack==true 时）：
 * - 计算已等待时间：now_tick - last_send_time_ms
 * - 若超过 timeout_ms 则：
 *   - 若 retry_count < MAX_RETRY：重发并增加计数
 *   - 若 retry_count >= MAX_RETRY：放弃，清空缓冲区，记录错误
 *
 * 调用频率：
 * - 建议每 10-50ms 调用一次
 * - 可从定时器中断或周期任务调用
 * - 不从 ISR 调用（过于耗时）
 *
 * 线程安全：仅从主任务调用
 *
 * @example
 *   // 在 app_task 中
 *   void app_task_main(void) {
 *       while (1) {
 *           uint32_t tick = xTaskGetTickCount();
 *           app_msg_send_tick(tick);
 *           vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 周期
 *       }
 *   }
 */
void app_msg_send_tick(uint32_t now_tick);

/**
 * @brief 查询模块是否在等待 ACK
 * @return bool - true 表示缓冲区有帧在等待 ACK，false 表示空闲
 *
 * @details
 * 用途：
 * - 判断是否可以发送下一个帧
 * - 在发送前检查：if (!app_msg_send_is_busy()) { ... }
 *
 * 线程安全：原子读
 */
bool app_msg_send_is_busy(void);

/**
 * @brief 获取当前缓冲区中的帧副本（已废弃）
 * @param p_frame 输出指针（未使用）
 * @return msg_send_ret_t - 总是返回 MSG_SEND_ERR_PARAM
 *
 * @deprecated 优化内存后不再保存原始帧，仅保存打包后的字节流
 * @note 此接口已不可用，保留以兼容旧代码
 */
msg_send_ret_t app_msg_send_get_buffered_frame(proto_frame_t *p_frame);

#ifdef __cplusplus
}
#endif

#endif /* APP_MSG_SEND_H */
