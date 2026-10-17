/**
 * @file    app_scan.h
 * @brief   主动扫频模块 - 向下级发送扫频信号
 * 
 * @details
 * 功能：
 * - SCAN 状态的完整逻辑封装
 * - 19 个频点的发送控制（每个 50ms）
 * - AD9833 DAC 控制
 * - 850ms 发送超时管理
 * - SCAN_WAIT_REPLY 的 1010 检测逻辑
 * - ACK 发送和重试机制
 * - PHASE1 和 PHASE2 的状态管理
 * 
 * 架构说明：
 * - 从 app_node_fsm 分离出 SCAN 和 SCAN_WAIT_REPLY 状态的业务逻辑
 * - app_node_fsm 负责状态转换协调
 * - app_scan 负责主动扫频和下级回应监听的具体实现
 * 
 * 模块职责：
 * 1. 管理 19 个频点的逐个发送（每个 50ms）
 * 2. 监听下级节点的 1010 交替波形回应
 * 3. 发送 ACK 并处理重试逻辑
 * 4. 管理 PHASE1（等待 1010）和 PHASE2（等待确认）
 * 
 * 初始化：
 * - app_scan_init() 在系统启动时调用
 * 
 * 使用流程：
 * 1. app_scan_init() 初始化模块
 * 2. 进入 SCAN 状态时调用 app_scan_start()
 * 3. 周期调用 app_scan_run() 执行扫频发送和状态转换
 * 4. ADC 检测到频率时调用 app_scan_on_freq_detected()（用于 1010 检测）
 * 5. 完成或失败时调用 app_scan_get_result() 获取结果
 * 6. 退出状态时调用 app_scan_stop()
 */

#ifndef APP_SCAN_H
#define APP_SCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================ */
/* 公开 API 声明                                                                */
/* ============================================================================ */

/**
 * @brief 初始化主动扫频模块
 * @details 必须在使用前调用一次
 */
void app_scan_init(void);

/**
 * @brief 启动主动扫频会话
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  进入 SCAN 状态时调用，完成以下初始化：
 *  1. 禁用 ADC 环形缓冲（发送模式不需要接收）
 *  2. 停止定时器输入捕获（不需要解调）
 *  3. 启动扫频会话计时器
 *  4. 初始化扫频索引
 *  5. 唤醒 AD9833 并发送第一个频点
 *  6. 设置 LED 显示为"扫频状态"
 */
void app_scan_start(uint32_t now_tick);

/**
 * @brief 停止主动扫频会话
 * 
 * @details
 *  退出 SCAN 状态时调用，完成以下清理：
 *  1. 停止 AD9833（进入睡眠）
 *  2. 重置扫频状态标志
 */
void app_scan_stop(void);

/**
 * @brief 进入等待下级回应阶段
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  扫频发送完成后调用，进入监听下级 1010 交替波形的阶段：
 *  1. 停止 AD9833 发送（进入休眠）
 *  2. 启用 ADC 环形缓冲（接收模式）
 *  3. 清空幅度表
 *  4. 初始化回应检测参数
 * 
 * @note LED 状态保持为 SCAN 状态（不重复设置）
 */
void app_scan_enter_wait_reply(uint32_t now_tick);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCAN_H */
