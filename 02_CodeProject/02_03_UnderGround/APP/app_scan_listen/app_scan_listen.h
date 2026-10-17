/**
 * @file    app_scan_listen.h
 * @brief   扫频监听模块 - 被动接收上级扫频信号
 * 
 * @details
 * 功能：
 * - SCAN_LISTEN 状态的完整逻辑封装
 * - ADC 频率检测处理
 * - 幅度表管理（19 个频点）
 * - 频率验证（是否在扫频表内，±15Hz 容差）
 * - 频对决选算法（70% 门限 + 最接近原则）
 * - 850ms 超时管理
 * 
 * 架构说明：
 * - 从 app_node_fsm 分离出 SCAN_LISTEN 状态的业务逻辑
 * - app_node_fsm 负责状态转换协调
 * - app_scan_listen 负责扫频监听的具体实现
 * 
 * 模块职责：
 * 1. 管理扫频监听过程（启动/停止/超时）
 * 2. 处理 ADC 频率检测事件，更新幅度表
 * 3. 执行频对决选算法
 * 
 * 初始化：
 * - app_scan_listen_init() 在系统启动时调用
 * 
 * 使用流程：
 * 1. app_scan_listen_init() 初始化模块
 * 2. 进入 SCAN_LISTEN 状态时调用 app_scan_listen_start()
 * 3. ADC 检测到频率时调用 app_scan_listen_on_freq_detected()
 * 4. 周期调用 app_scan_listen_run() 检查超时
 * 5. 超时后调用 app_scan_listen_select_freq_pair() 决选频对
 * 6. 退出状态时调用 app_scan_listen_stop()
 */

#ifndef APP_SCAN_LISTEN_H
#define APP_SCAN_LISTEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================ */
/* 公开常量                                                                      */
/* ============================================================================ */

/**
 * @brief 频对最小间隔（Hz）
 * @details 两个频点之间至少相距 150Hz 才能有效区分
 */
#define APP_SCAN_LISTEN_MIN_FREQ_GAP_HZ  (150U)

/**
 * @brief 有效信号幅度门限
 * @details 扫频监听时，只有幅值超过此门限的频点才被认为是有效信号
 */
#define APP_SCAN_LISTEN_AMP_VALID_THRESHOLD  (100U)

/* ============================================================================ */
/* 公开 API 声明                                                                */
/* ============================================================================ */

/**
 * @brief 初始化扫频监听模块
 * @details 必须在使用前调用一次
 */
void app_scan_listen_init(void);

/**
 * @brief 启动扫频监听会话
 * @param now_tick 当前时刻（ms）
 * 
 * @details
 *  进入 SCAN_LISTEN 状态时调用，完成以下初始化：
 *  1. 清空幅度表和计时器
 *  2. 启用 ADC 环形缓冲（接收信号）
 *  3. 停止定时器输入捕获（监听模式不需要解调）
 *  4. 设置 LED 显示为"监听状态"
 */
void app_scan_listen_start(uint32_t now_tick);

/**
 * @brief 停止扫频监听会话
 * 
 * @details
 *  退出 SCAN_LISTEN 状态时调用，完成以下清理：
 *  1. 禁用 ADC 环形缓冲
 *  2. 重置监听状态标志
 */
void app_scan_listen_stop(void);

/**
 * @brief 获取有效频点数（幅值超过门限的频点数）
 * @return 有效频点数
 * 
 * @details
 *  统计所有幅值超过 APP_NODE_FSM_SWEEP_AMP_VALID_THRESHOLD 的频点
 *  用于判断扫频监听是否检测到足够的信号
 */
uint8_t app_scan_listen_get_valid_freq_count(void);

/**
 * @brief 更新幅度表（保留各频点的最大幅值）
 * @param idx 频点索引（0~18）
 * @param amplitude 当前检测到的幅度
 * 
 * @details
 *  用于在扫频过程中累积各频点的信号强度
 *  调用者应先通过 app_msg_freq_to_index() 将检测频率转换为索引
 */
void app_scan_listen_update_amp_table(uint8_t idx, uint16_t amplitude);

/**
 * @brief 频对决选算法
 * @param[out] p_out_f0 选出的低频点（输出）
 * @param[out] p_out_f1 选出的高频点（输出）
 * @return 0 = 成功选出频对，-1 = 有效频点不足，-2 = 找不到满足间距的频对
 * 
 * @details
 *  根据扫频监听过程中累积的幅度表，选择最佳频对
 *  算法步骤：
 *  1. 找出幅值最大的频点
 *  2. 计算强度门限 = max_amp * 70%
 *  3. 筛选超过门限的频点
 *  4. 在强频点中找幅值最接近的两个，且满足最小间隔要求（≥150Hz）
 */
int32_t app_scan_listen_select_freq_pair(uint16_t *p_out_f0, uint16_t *p_out_f1);

#ifdef __cplusplus
}
#endif

#endif /* APP_SCAN_LISTEN_H */
