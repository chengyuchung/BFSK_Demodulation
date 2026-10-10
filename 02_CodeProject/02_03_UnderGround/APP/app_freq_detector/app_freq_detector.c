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

/* ========== 私有状态 ========== */

static uint8_t  s_inited = 0;
static uint16_t s_last_detected_freq = 0;     /* 上次检测的频率 */
static uint32_t s_last_detected_energy = 0;   /* 上次检测的能量 */
static uint16_t s_snapshot[400];              /* ADC 快照缓冲 */

/* ========== 公共接口实现 ========== */

app_freq_detector_ret_t app_freq_detector_init(void)
{
    if (s_inited) {
        return APP_FREQ_DETECTOR_OK;
    }

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_last_detected_freq = 0;
    s_last_detected_energy = 0;

    /* 初始化 BSW 频率分析模块 */
    bsw_freq_analyzer_init();

    s_inited = 1;
    bsw_log("[FreqDetector] initialized\r\n");

    return APP_FREQ_DETECTOR_OK;
}

void app_freq_detector_task(uint32_t now_tick)
{
    if (!s_inited) {
        return;
    }

    /* 【步骤 1】从 ADC 环形缓冲取快照（最新 400 点 = 40ms） */
    if (bsw_adc_ringbuf_snapshot(s_snapshot) != BSW_ADC_RINGBUF_OK) {
        return;
    }

    /* 【步骤 2】进行频率分析 */
    bsw_freq_analyzer_result_t result = {0};
    bsw_freq_analyzer_ret_t ret = bsw_freq_analyzer_analyze(s_snapshot, &result);

    if (ret != BSW_FREQ_ANALYZER_OK) {
        /* 未检测到有效信号，保持上一次的结果不变 */
        return;
    }

    /* 【步骤 3】保存分析结果 */
    s_last_detected_freq = result.freq_hz;
    s_last_detected_energy = result.energy;

    /* 【步骤 4】【关键】只在频率变化时才注入 FSM 事件 */
    static uint16_t prev_freq = 0;

    if (result.freq_hz != prev_freq) {
        prev_freq = result.freq_hz;

        /* 调用 FSM 回调 */
        app_node_fsm_on_sweep_detected(result.freq_hz, result.amplitude, now_tick);

        bsw_log("[FreqDetector] freq changed: %u Hz (energy=%lu)\r\n",
                result.freq_hz, result.energy);
    }
}

uint16_t app_freq_detector_get_last_freq(void)
{
    return s_last_detected_freq;
}

uint32_t app_freq_detector_get_last_energy(void)
{
    return s_last_detected_energy;
}
