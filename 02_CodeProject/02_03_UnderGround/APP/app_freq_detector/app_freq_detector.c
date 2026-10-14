/**
 * @file    app_freq_detector.c
 * @brief   频率检测任务 - APP 层实现
 */

#include "app_freq_detector.h"
#include "bsw_freq_analyzer.h"
#include "bsw_adc_ringbuf.h"
#include "../app_node_fsm/app_node_fsm.h"
#include "bsw_log.h"

#include <string.h>

/* ============================================================================
 * 频率检测配置宏定义
 * ============================================================================ */

#define APP_FREQ_DETECTOR_SNAPSHOT_SIZE     (400)   // 400 个 ADC 采样点 = 40ms
#define APP_FREQ_DETECTOR_SNAPSHOT_TIME_MS  (40)    // 快照持续时间

/* ============================================================================
 * 私有状态变量
 * ============================================================================ */

/**
 * @brief 模块初始化标志
 * @rationale: 确保单次初始化，防止重入
 */
static uint8_t s_inited = 0;

/**
 * @brief 最后检测到的频率
 * @rationale: 缓存的结果供 app_freq_detector_get_last_freq() 查询
 */
static uint16_t s_last_detected_freq = 0;

/**
 * @brief 最后检测到的信号能量
 * @rationale: 缓存的结果供 app_freq_detector_get_last_energy() 查询
 */
static uint32_t s_last_detected_energy = 0;

/**
 * @brief ADC 快照缓冲（400 个采样点）
 * @rationale: 预分配缓冲供频率分析器输入；必须是静态以确保内存位置固定
 */
static uint16_t s_snapshot[APP_FREQ_DETECTOR_SNAPSHOT_SIZE];

/* ============================================================================
 * 公共接口实现
 * ============================================================================ */

/**
 * @brief 初始化频率检测模块
 * @return app_freq_detector_ret_t - 成功返回 APP_FREQ_DETECTOR_OK
 * @note 可以安全地多次调用；后续调用立即返回
 */
app_freq_detector_ret_t app_freq_detector_init(void)
{
    if (s_inited)
    {
        return APP_FREQ_DETECTOR_OK;
    }

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_last_detected_freq = 0;
    s_last_detected_energy = 0;

    // 初始化 BSW 频率分析模块
    bsw_freq_analyzer_init();

    s_inited = 1;
    bsw_log("[FreqDetector] initialized\r\n");

    return APP_FREQ_DETECTOR_OK;
}

/**
 * @brief 频率检测周期任务
 * @param now_tick 当前系统时刻（用于 FSM 事件时间戳）
 * @note 需要定期调用（如每 40ms 或 50ms）
 * @details 对 ADC 快照进行频率分析，当频率改变时触发 FSM 事件
 */
void app_freq_detector_task(uint32_t now_tick)
{
    if (!s_inited)
    {
        return;
    }

    // 步骤 1：从环形缓冲获取 ADC 快照
    // 为什么：400 个采样点的新窗口（40ms）最大化频率分辨率
    if (bsw_adc_ringbuf_snapshot(s_snapshot) != BSW_ADC_RINGBUF_OK)
    {
        return;
    }

    // 步骤 2：执行频率分析
    bsw_freq_analyzer_result_t result = {0};
    bsw_freq_analyzer_ret_t ret = bsw_freq_analyzer_analyze(s_snapshot, &result);

    if (ret != BSW_FREQ_ANALYZER_OK)
    {
        // 未检测到有效信号，保持前一次结果不变
        return;
    }

    // 步骤 3：缓存分析结果
    s_last_detected_freq = result.freq_hz;
    s_last_detected_energy = result.energy;

    // 步骤 4：仅在频率改变时注入 FSM 事件
    // 为什么：防止相同频率的重复状态转换；
    //        减少不必要的 FSM 重新触发和日志输出
    static uint16_t prev_freq = 0;

    if (result.freq_hz != prev_freq)
    {
        prev_freq = result.freq_hz;

        // 用检测到的频率信息调用 FSM 回调
        app_node_fsm_on_sweep_detected(result.freq_hz, result.amplitude, now_tick);

        bsw_log("[FreqDetector] freq changed: %u Hz (energy=%lu)\r\n",
                result.freq_hz, result.energy);
    }
}

/**
 * @brief 获取最后检测到的频率
 * @return uint16_t 最后检测的频率（Hz）
 * @note 返回缓存值，非阻塞
 */
uint16_t app_freq_detector_get_last_freq(void)
{
    return s_last_detected_freq;
}

/**
 * @brief 获取最后检测到的信号能量
 * @return uint32_t 最后检测的能量值
 * @note 返回缓存值，非阻塞
 */
uint32_t app_freq_detector_get_last_energy(void)
{
    return s_last_detected_energy;
}
