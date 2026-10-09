/**
 * @file    bsw_bfsk_mod.h
 * @brief   BFSK 调制器 - BSW 层
 *
 * @note    与 bsw_bfsk_demod（解调接收）配对，构成井下节点的双向通信：
 *              bsw_bfsk_mod    ── 上行发送帧（回复查询 / 主动告警）
 *              bsw_bfsk_demod  ── 下行接收帧（查询 / 控制）
 *
 * 调制链路：
 *   1. 接收字节流（已打包的帧：bsw_proto.c::proto_frame_pack 输出）
 *   2. 字节 → bit 流（MSB first，协议层定义）
 *   3. bit 流 → 频率切换序列（bit=0 → f0，bit=1 → f1）
 *   4. 通过 bsw_ad9833 切换 FREQ0/FREQ1 输出
 *   5. 每个 bit 持续 bit_period_ms（协议层统一配置）
 *
 * 频率对来源：
 *   - 从 app_node_fsm 的 s_ctx.cur_freq 注入 f0_hz / f1_hz
 *   - 与 bsw_bfsk_demod 使用相同的频率对
 *
 * @dependency  bsw_ad9833    (频率输出硬件抽象)
 *              mcal_timer    (定时器，控制 bit 持续时间)
 *
 * @usage
 *   bsw_bfsk_mod_init();
 *   // 设置当前频率对（FSM 锁频后）
 *   bsw_bfsk_mod_set_freq(f0_hz, f1_hz, bit_period_ms);
 *   // 发送一帧
 *   bsw_bfsk_mod_send(tx_buf, tx_len);
 *   // 主循环中查询状态
 *   if (bsw_bfsk_mod_get_state() == BSW_BFSK_MOD_IDLE) {
 *       // 发送完成，可以发下一帧
 *   }
 */

#ifndef BSW_BFSK_MOD_H
#define BSW_BFSK_MOD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========== 调制器状态机 ========== */
typedef enum {
    BSW_BFSK_MOD_IDLE        = 0,   /**< 空闲，可以发送 */
    BSW_BFSK_MOD_SENDING     = 1,   /**< 正在发送 bit 流 */
    BSW_BFSK_MOD_DONE_OK     = 2,   /**< 发送完成 */
    BSW_BFSK_MOD_DONE_ERR    = 3,   /**< 发送失败（硬件错误）*/
} bsw_bfsk_mod_state_t;

/* ========== 返回码 ========== */
typedef enum {
    BSW_BFSK_MOD_OK        =  0,
    BSW_BFSK_MOD_ERR_PARAM = -1,   /**< 参数非法（NULL / 长度溢出）*/
    BSW_BFSK_MOD_ERR_STATE = -2,   /**< 状态不允许（正在发送时重复调用）*/
    BSW_BFSK_MOD_ERR_HW    = -3,   /**< 硬件错误（AD9833 故障）*/
} bsw_bfsk_mod_ret_t;

/* ========== 发送缓冲区大小 ========== */
#define BSW_BFSK_MOD_MAX_BYTES  64u   /**< 最大支持 64 字节帧（协议层上限）*/

/* ========== 函数声明 ========== */

/**
 * @brief   初始化 BFSK 调制器
 * @note    1) 初始化内部状态机为 IDLE
 *          2) 注册定时器回调（用于 bit 定时）
 *          3) 不启动硬件
 * @retval  BSW_BFSK_MOD_OK
 */
bsw_bfsk_mod_ret_t bsw_bfsk_mod_init(void);

/**
 * @brief   设置调制频率对和 bit 周期
 * @param   f0_hz         低频点（Hz），对应 bit=0
 * @param   f1_hz         高频点（Hz），对应 bit=1
 * @param   bit_period_ms 每个 bit 持续时间（ms）
 * @retval  BSW_BFSK_MOD_OK / ERR_PARAM
 * @note    必须在 send 之前调用
 *          频率对应与 bsw_ad9833 的 FREQ0/FREQ1 寄存器
 */
bsw_bfsk_mod_ret_t bsw_bfsk_mod_set_freq(uint16_t f0_hz,
                                         uint16_t f1_hz,
                                         uint32_t bit_period_ms);

/**
 * @brief   发送一帧（异步，非阻塞）
 * @param   data     帧字节流（已打包，含转义）
 * @param   len      字节数（1 ~ BSW_BFSK_MOD_MAX_BYTES）
 * @retval  BSW_BFSK_MOD_OK           启动发送
 *          BSW_BFSK_MOD_ERR_PARAM    data=NULL / len=0 / len>MAX
 *          BSW_BFSK_MOD_ERR_STATE    上一帧还在发送中
 * 
 * @note    1) 拷贝数据到内部缓冲区
 *          2) 转换为 bit 流（MSB first）
 *          3) 启动定时器，逐 bit 切换 AD9833 频率
 *          4) 状态置 SENDING
 *          5) 调用方通过 bsw_bfsk_mod_get_state() 查询发送完成
 */
bsw_bfsk_mod_ret_t bsw_bfsk_mod_send(const uint8_t *data, uint16_t len);

/**
 * @brief   主循环钩子：定时器任务 + 状态机推进
 * @param   now_tick    HAL_GetTick() 或 OS 时钟（ms）
 * @note    需在 app_task 中周期调用（建议 1~10ms）
 */
void bsw_bfsk_mod_run(uint32_t now_tick);

/**
 * @brief   查询调制器状态
 */
bsw_bfsk_mod_state_t bsw_bfsk_mod_get_state(void);

/**
 * @brief   主动停止发送（异常恢复 / FSM 状态切换时用）
 * @note    1) 关闭定时器
 *          2) 关闭 AD9833 输出（可选）
 *          3) 清空缓冲
 *          4) 状态置 IDLE
 */
bsw_bfsk_mod_ret_t bsw_bfsk_mod_stop(void);

/* ========== 内部回调（定时器调用，不要在应用层直接调）========== */

/**
 * @brief   定时器回调：每个 bit_period_ms 触发一次
 * @note    由 mcal_timer 注册
 *          本函数负责：取下一个 bit → 切换 AD9833 频率 → 推进状态机
 */
void bsw_bfsk_mod_on_bit_timer(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_BFSK_MOD_H */
