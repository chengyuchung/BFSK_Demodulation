/**
 * @file    app_freq_detector.h
 * @brief   频率检测任务 - APP 层集成
 *
 * @details
 * 在 FreeRTOS 任务中定期调用频率分析，并将结果注入到节点 FSM。
 * 
 * 职责：
 *   - 每 10ms 从 ADC 环形缓冲中取快照
 *   - 调用 BSW 频率分析模块计算当前频率
 *   - 检测频率变化，仅在变化时触发 FSM 事件
 *   - 这样避免频率相同时的重复事件
 *
 * @dependency   bsw_adc_ringbuf, bsw_freq_analyzer, app_node_fsm
 */

#ifndef APP_FREQ_DETECTOR_H
#define APP_FREQ_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * 返回码定义
 * ============================================================================ */

typedef enum {
    APP_FREQ_DETECTOR_OK    = 0,    // 操作成功
    APP_FREQ_DETECTOR_ERROR = -1,   // 操作失败
} app_freq_detector_ret_t;

/* ============================================================================
 * 公共接口
 * ============================================================================ */

/**
 * @brief 初始化频率检测模块
 * @return app_freq_detector_ret_t - 成功返回 APP_FREQ_DETECTOR_OK
 * @note 必须在 bsw_adc_ringbuf_init() 之后调用
 * @details 初始化内部状态、快照缓冲和 BSW 分析器模块
 */
app_freq_detector_ret_t app_freq_detector_init(void);

/**
 * @brief 频率检测周期任务
 * @param now_tick 当前系统时刻（如 HAL_GetTick()）
 * @note 每 10ms 从 FreeRTOS 任务调度器调用
 * @details 获取 ADC 快照，执行频率分析，仅当频率改变时触发 FSM 事件
 */
void app_freq_detector_task(uint32_t now_tick);

/**
 * @brief 获取最后检测到的频率值
 * @return uint16_t 频率（Hz），0 表示无有效检测
 * @note 非阻塞缓存读取
 */
uint16_t app_freq_detector_get_last_freq(void);

/**
 * @brief 获取最后检测到的信号能量
 * @return uint32_t 最后一次分析的能量值
 * @note 非阻塞缓存读取
 */
uint32_t app_freq_detector_get_last_energy(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FREQ_DETECTOR_H */
