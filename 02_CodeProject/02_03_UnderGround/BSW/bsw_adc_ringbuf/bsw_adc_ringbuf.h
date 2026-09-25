/**
 * @file    bsw_adc_ringbuf.h
 * @brief   ADC1 环形缓冲区 - BSW 层
 *
 * @details 该模块接管 ADC1 的 DMA 通道，把一片 2 的幂次方大小的环形 buffer
 *          持续填满。对外暴露两个原语：
 *            - snapshot()：按时间顺序把"最新 400 个采样点"（40 ms × 10 kHz）
 *                         拷贝到调用方数组，用于扫频协议分析窗口。
 *            - read()    ：按"倒数 N 个点"的索引读单个样本，用于噪声估计、
 *                         调试等需要任意位置取样的场景。
 *
 * @note    容量选择：512 = 2^9。协议要求的窗口是 400 点（40 ms），
 *          512 点对应 51.2 ms，留有 28% 余量可吸收晶振容差与切频时序抖动。
 *          同时 2 的 9 次方使 wrap-around 用 `& 0x1FF` 一条指令即可，避开取模运算。
 *
 * @dependency  mcal_adc (HAL_ADC + DMA)
 *
 * @usage
 *   // 1. 上电流程：mcal_clock/gpio/dma/timer/adc 初始化之后
 *   bsw_adc_ringbuf_init();
 *
 *   // 2. 在任务上下文中每 10 ms 调一次
 *   uint16_t window[BSW_ADC_RINGBUF_WINDOW_LEN];
 *   if (bsw_adc_ringbuf_snapshot(window) == BSW_ADC_RINGBUF_OK) {
 *       // window[0] = 最旧一点, window[399] = 最新一点
 *       feed_to_freq_analyzer(window);
 *   }
 */

#ifndef BSW_ADC_RINGBUF_H
#define BSW_ADC_RINGBUF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 容量与窗口常量 ==========
 *
 * CAPACITY 必须是 2 的幂次方。改这两个宏时务必同步改 snapshot() 内的
 * 双段 memcpy 假设（窗口 ≤ 容量 ≤ 2 倍窗口才安全）。
 */
#define BSW_ADC_RINGBUF_CAPACITY       (512u)   /**< 环形缓冲区总长（点），51.2 ms 余量 */
#define BSW_ADC_RINGBUF_CAPACITY_MASK  ((uint16_t)(BSW_ADC_RINGBUF_CAPACITY - 1u))   /**< 位掩码 wrap-around */
#define BSW_ADC_RINGBUF_WINDOW_LEN     (400u)   /**< 分析窗口长度（点），40 ms × 10 kHz */

/* ========== 返回码 ========== */
typedef enum {
    BSW_ADC_RINGBUF_OK         =  0,
    BSW_ADC_RINGBUF_ERR_PARAM  = -1,     /**< 参数非法（NULL 指针 / 越界） */
    BSW_ADC_RINGBUF_ERR_STATE  = -2,     /**< 状态非法（未初始化 / MCAL 拒接） */
} bsw_adc_ringbuf_ret_t;

/* ========== 公共 API ========== */

/**
 * @brief   初始化环形缓冲区，接管 ADC1 DMA 循环写入自带 buffer。
 *
 * @note    前置依赖（按顺序）：mcal_clock_init → mcal_gpio_init →
 *          mcal_dma_init → mcal_timer_init → mcal_adc_init。
 *          内部会注册 ADC1 DMA 半传输 / 全传输回调，并启动 DMA 循环到模块
 *          内置的 512 点 buffer。Idempotent：重复调用幂等。
 *
 * @retval  BSW_ADC_RINGBUF_OK
 *          BSW_ADC_RINGBUF_ERR_STATE
 */
bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_init(void);

/**
 * @brief   反初始化：停 ADC1 DMA、注销回调、清状态。
 * @note    上层需要切到 AD9833 发射等场景时可调，调完后 buffer 不可再读。
 */
bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_deinit(void);

/**
 * @brief   抓取当前最新 400 个采样点快照，按时间正序填到 out。
 *
 * @param   out  调用方提供的 BSW_ADC_RINGBUF_WINDOW_LEN 长 uint16_t 数组。
 * @retval  BSW_ADC_RINGBUF_OK
 *          BSW_ADC_RINGBUF_ERR_PARAM (out == NULL)
 *          BSW_ADC_RINGBUF_ERR_STATE (未初始化)
 *
 * @note    out[0] 是最旧一点，out[BSW_ADC_RINGBUF_WINDOW_LEN-1] 是最新一点。
 *          内部关中断（约 5–10 μs @ 80 MHz）保护拷贝，避免被 DMA 写入污染。
 *          对 10 kHz 采样（100 μs 一点）影响忽略不计。
 */
bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_snapshot(uint16_t *out);

/**
 * @brief   读环形缓冲内某个历史位置的样本（用于噪声估计等）。
 *
 * @param   index_back  从最新一点往回数多少点。0 = 最新点，1 = 倒数第二新点。
 *                      合法范围：[0, BSW_ADC_RINGBUF_CAPACITY-1]。
 * @param   out         接收样本值。
 * @retval  同上
 */
bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_read(uint32_t index_back, uint16_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BSW_ADC_RINGBUF_H */
