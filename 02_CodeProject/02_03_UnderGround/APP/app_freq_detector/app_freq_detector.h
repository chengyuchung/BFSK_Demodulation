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

/* ========== 返回码 ========== */

typedef enum {
    APP_FREQ_DETECTOR_OK    = 0,
    APP_FREQ_DETECTOR_ERROR = -1,
} app_freq_detector_ret_t;

/* ========== 公共接口 ========== */

/**
 * @brief   初始化频率检测器
 * 
 * @note    必须在 bsw_adc_ringbuf_init() 之后调用
 * @retval  APP_FREQ_DETECTOR_OK
 */
app_freq_detector_ret_t app_freq_detector_init(void);

/**
 * @brief   频率检测主循环（由 FreeRTOS 任务每 10ms 调用一次）
 * 
 * @param   now_tick  当前时刻 (HAL_GetTick)
 * 
 * @note    - 此函数应该在 10ms 周期的 FreeRTOS 任务中调用
 *          - 内部会自动检测频率变化，只在变化时注入 FSM 事件
 *          - 如果当前检测不到有效信号，会保持上一次的结果
 */
void app_freq_detector_task(uint32_t now_tick);

/**
 * @brief   获取最后一次检测的频率
 * 
 * @retval  上次检测的频率 (Hz)，0 表示无效或未检测
 */
uint16_t app_freq_detector_get_last_freq(void);

/**
 * @brief   获取最后一次检测的能量
 * 
 * @retval  上次检测的能量值
 */
uint32_t app_freq_detector_get_last_energy(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_FREQ_DETECTOR_H */
