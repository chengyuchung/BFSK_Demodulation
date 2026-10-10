/**
 * @file    bsw_freq_analyzer.h
 * @brief   频率分析模块 - BSW 层接口
 *
 * @details
 * 基于 Goertzel 算法对 ADC 采样数据进行频率分析。
 * 
 * 功能：
 *   - 从 40ms 的 ADC 采样窗口（400 点 @ 10kHz）中检测主导频率
 *   - 支持 19 个标准扫频频点（125Hz~1025Hz，步进 50Hz）
 *   - 返回检测到的频率和相应的能量幅度
 *
 * 使用流程：
 *   1. 上层（app_task）每 10ms 调用一次 bsw_freq_analyzer_analyze()
 *   2. 传入 ADC snapshot（400 点）
 *   3. 返回 (频率, 幅度) 对
 *   4. 上层比较频率是否变化，变化时调用 app_node_fsm_on_sweep_detected()
 *
 * @dependency   无（纯信号处理，不依赖硬件）
 */

#ifndef BSW_FREQ_ANALYZER_H
#define BSW_FREQ_ANALYZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 常量定义 ========== */

/** ADC 采样频率 (Hz) */
#define BSW_FREQ_ANALYZER_SAMPLE_RATE_HZ   (10000U)

/** ADC 窗口长度（点） */
#define BSW_FREQ_ANALYZER_WINDOW_LEN       (400U)

/** 标准扫频频点数 */
#define BSW_FREQ_ANALYZER_FREQ_COUNT       (19U)

/** 扫频起始频率 (Hz) */
#define BSW_FREQ_ANALYZER_FREQ_START       (125U)

/** 扫频频率步进 (Hz) */
#define BSW_FREQ_ANALYZER_FREQ_STEP        (50U)

/** 能量阈值：低于此值认为未检测到有效信号 */
#define BSW_FREQ_ANALYZER_ENERGY_THRESHOLD (5000U)

/* ========== 返回码 ========== */

typedef enum {
    BSW_FREQ_ANALYZER_OK           =  0,
    BSW_FREQ_ANALYZER_ERR_PARAM    = -1,    /**< 参数非法 */
    BSW_FREQ_ANALYZER_ERR_NO_SIGNAL = -2,   /**< 未检测到有效信号 */
} bsw_freq_analyzer_ret_t;

/* ========== 结果结构体 ========== */

/**
 * @struct  bsw_freq_analyzer_result_t
 * @brief   频率分析结果
 */
typedef struct {
    uint16_t freq_hz;      /**< 检测到的频率 (Hz)，取值范围 [125, 1025] */
    uint32_t energy;       /**< 该频率的能量（无量纲相对值） */
    uint16_t amplitude;    /**< 幅度估计（仅供参考） */
} bsw_freq_analyzer_result_t;

/* ========== 公共接口 ========== */

/**
 * @brief   初始化频率分析模块
 * 
 * @note    可选调用。本模块无状态，不调用也可以直接使用 analyze()。
 *          此接口用于未来扩展（如内部表预计算）。
 * 
 * @retval  BSW_FREQ_ANALYZER_OK
 */
bsw_freq_analyzer_ret_t bsw_freq_analyzer_init(void);

/**
 * @brief   分析 ADC 采样窗口，检测主导频率
 * 
 * @param   snapshot  ADC 采样数据，长度必须为 BSW_FREQ_ANALYZER_WINDOW_LEN (400)
 * @param   out_result  接收分析结果（频率 + 能量 + 幅度）
 * 
 * @retval  BSW_FREQ_ANALYZER_OK            成功，out_result 有效
 *          BSW_FREQ_ANALYZER_ERR_PARAM     snapshot == NULL 或其他参数错误
 *          BSW_FREQ_ANALYZER_ERR_NO_SIGNAL 未检测到有效信号（所有频点能量都太低）
 * 
 * @note    - 此函数线程安全（无全局状态修改）
 *          - 计算时间：~200~500us @ STM32L4 80MHz（19 个 Goertzel + 遍历）
 *          - 栈使用：~200 字节（临时浮点变量）
 * 
 * @example
 *   uint16_t snapshot[400];
 *   bsw_adc_ringbuf_snapshot(snapshot);
 *   
 *   bsw_freq_analyzer_result_t result;
 *   if (bsw_freq_analyzer_analyze(snapshot, &result) == BSW_FREQ_ANALYZER_OK) {
 *       printf("Detected: %u Hz, Energy: %lu\r\n", result.freq_hz, result.energy);
 *   }
 */
bsw_freq_analyzer_ret_t bsw_freq_analyzer_analyze(const uint16_t *snapshot,
                                                  bsw_freq_analyzer_result_t *out_result);

/**
 * @brief   获取第 i 个标准扫频频点值
 * 
 * @param   idx  频点索引，范围 [0, 18]
 * 
 * @retval  频率值 (Hz)，若 idx 越界返回 0
 * 
 * @example
 *   for (int i = 0; i < 19; i++) {
 *       uint16_t f = bsw_freq_analyzer_get_freq(i);
 *       printf("Freq[%d] = %u Hz\r\n", i, f);
 *   }
 */
uint16_t bsw_freq_analyzer_get_freq(uint8_t idx);

/**
 * @brief   频率值转换为频点索引
 * 
 * @param   freq_hz  频率值 (Hz)，必须是标准频点（125, 175, ..., 1025）
 * @param   tolerance_hz  容差范围（Hz），默认 ±15Hz
 * 
 * @retval  [0, 18]      成功，返回对应的频点索引
 *          0xFF         失败，频率不匹配任何标准频点
 * 
 * @example
 *   uint8_t idx = bsw_freq_analyzer_freq_to_index(125, 15);  // 返回 0
 *   idx = bsw_freq_analyzer_freq_to_index(128, 15);  // 返回 0 (容差内)
 *   idx = bsw_freq_analyzer_freq_to_index(150, 15);  // 返回 0xFF (不匹配)
 */
uint8_t bsw_freq_analyzer_freq_to_index(uint16_t freq_hz, uint16_t tolerance_hz);

#ifdef __cplusplus
}
#endif

#endif /* BSW_FREQ_ANALYZER_H */
