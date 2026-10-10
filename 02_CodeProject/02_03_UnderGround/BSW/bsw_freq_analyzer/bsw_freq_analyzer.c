/**
 * @file    bsw_freq_analyzer.c
 * @brief   频率分析模块 - BSW 层实现
 *
 * 实现原理：
 *   使用 Goertzel 算法在时域内高效计算特定频率的能量。
 *   不需要 FFT，计算量小，适合实时嵌入式应用。
 *
 * Goertzel 算法简述：
 *   对于目标频率 f_target，计算过滤后的响应。
 *   通过二阶 IIR 滤波器和最后的功率计算，得到该频率分量的能量。
 *
 * 性能：
 *   - 时间复杂度：O(19×N) = O(19×400) = O(7600)
 *   - 空间复杂度：O(1)（无需存储整个频域）
 */

#include "bsw_freq_analyzer.h"
#include <math.h>
#include <string.h>

/* ========== 静态数据与初始化 ========== */

/** 19 个标准扫频频点 (Hz) */
static const uint16_t s_standard_freqs[BSW_FREQ_ANALYZER_FREQ_COUNT] = {
    125, 175, 225, 275, 325, 375, 425, 475, 525,
    575, 625, 675, 725, 775, 825, 875, 925, 975, 1025
};

static uint8_t s_inited = 0;

/* ========== 私有函数 - Goertzel 核心算法 ========== */

/**
 * @brief   Goertzel 算法：计算特定频率在信号中的能量（功率）
 * 
 * @param   samples       输入采样数组
 * @param   num_samples   采样点数
 * @param   freq_hz       目标频率 (Hz)
 * @param   sample_rate_hz 采样频率 (Hz)
 * 
 * @retval  该频率对应的能量值（无量纲）
 * 
 * 算法流程：
 *   1. 根据目标频率和采样率，计算归一化频率 k
 *   2. 初始化二阶 IIR 滤波系数
 *   3. 逐点输入采样，更新状态变量 s0, s1, s2
 *   4. 根据最终状态计算实部和虚部，得到功率 = real² + imag²
 */
static uint32_t _goertzel_power(const uint16_t *samples,
                                uint16_t num_samples,
                                uint16_t freq_hz,
                                uint16_t sample_rate_hz)
{
    if (samples == NULL || num_samples == 0) {
        return 0;
    }

    /* 【步骤 1】计算归一化频率 bin */
    float k = ((float)num_samples * (float)freq_hz) / (float)sample_rate_hz;
    float w = 2.0f * 3.14159265359f * k / (float)num_samples;

    /* 【步骤 2】计算 IIR 滤波系数 */
    float coeff = 2.0f * cos(w);

    /* 【步骤 3】初始化状态变量 */
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;

    /* 【步骤 4】逐点通过二阶滤波器 */
    for (uint16_t i = 0; i < num_samples; i++) {
        float sample = (float)samples[i];
        s0 = sample + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    /* 【步骤 5】计算功率 (实部² + 虚部²) */
    float cos_w = cos(w);
    float sin_w = sin(w);

    float real = s1 - s2 * cos_w;
    float imag = s2 * sin_w;

    float power = real * real + imag * imag;

    /* 【步骤 6】归一化（可选，这里直接返回原始能量） */
    return (uint32_t)power;
}

/* ========== 公共接口实现 ========== */

bsw_freq_analyzer_ret_t bsw_freq_analyzer_init(void)
{
    if (s_inited) {
        return BSW_FREQ_ANALYZER_OK;
    }

    /* 本模块无状态，init 仅作为标记
     * 未来若需预计算三角函数表可在此进行 */

    s_inited = 1;
    return BSW_FREQ_ANALYZER_OK;
}

bsw_freq_analyzer_ret_t bsw_freq_analyzer_analyze(const uint16_t *snapshot,
                                                  bsw_freq_analyzer_result_t *out_result)
{
    if (snapshot == NULL || out_result == NULL) {
        return BSW_FREQ_ANALYZER_ERR_PARAM;
    }

    uint32_t max_energy = 0;
    uint16_t best_freq = s_standard_freqs[0];

    /* 【步骤 1】遍历所有 19 个标准频点，计算各自的能量 */
    for (uint8_t i = 0; i < BSW_FREQ_ANALYZER_FREQ_COUNT; i++) {
        uint32_t energy = _goertzel_power(snapshot,
                                          BSW_FREQ_ANALYZER_WINDOW_LEN,
                                          s_standard_freqs[i],
                                          BSW_FREQ_ANALYZER_SAMPLE_RATE_HZ);

        /* 【步骤 2】记录能量最大的频点 */
        if (energy > max_energy) {
            max_energy = energy;
            best_freq = s_standard_freqs[i];
        }
    }

    /* 【步骤 3】判断是否检测到有效信号 */
    if (max_energy < BSW_FREQ_ANALYZER_ENERGY_THRESHOLD) {
        return BSW_FREQ_ANALYZER_ERR_NO_SIGNAL;
    }

    /* 【步骤 4】填充输出结果 */
    out_result->freq_hz = best_freq;
    out_result->energy = max_energy;
    out_result->amplitude = (uint16_t)(sqrt((double)max_energy) * 0.5f);  // 简化幅度估计

    return BSW_FREQ_ANALYZER_OK;
}

uint16_t bsw_freq_analyzer_get_freq(uint8_t idx)
{
    if (idx >= BSW_FREQ_ANALYZER_FREQ_COUNT) {
        return 0;
    }
    return s_standard_freqs[idx];
}

uint8_t bsw_freq_analyzer_freq_to_index(uint16_t freq_hz, uint16_t tolerance_hz)
{
    for (uint8_t i = 0; i < BSW_FREQ_ANALYZER_FREQ_COUNT; i++) {
        uint16_t standard_freq = s_standard_freqs[i];

        /* 检查是否在容差范围内 */
        if (freq_hz >= standard_freq - tolerance_hz &&
            freq_hz <= standard_freq + tolerance_hz) {
            return i;
        }
    }

    return 0xFF;  /* 未找到匹配的频点 */
}
