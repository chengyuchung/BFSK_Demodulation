/**
 * @file    mcal_adc.h
 * @brief   ADC 采样驱动 - MCAL 层
 * @note    直接调用 STM32 HAL_ADC / LL_ADC，不包含业务逻辑
 *
 * @dependency   mcal_dma (DMA 模式依赖)
 */

#ifndef MCAL_ADC_H
#define MCAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 头文件 ========== */
#include "main.h"

/* ========== 类型定义 ========== */

/* ========== 宏定义 ========== */
#define ADC_CHANNEL_0     0   /* 示例：ADC 通道 0 */

/* ========== 函数声明 ========== */

/**
 * @brief   ADC 初始化
 * @note    配置 ADC 为 DMA 模式或轮询模式
 */
void mcal_adc_init(void);

/**
 * @brief   启动一次 ADC 转换
 * @param   ch  ADC 通道号
 * @retval  ADC 原始值
 */
uint32_t mcal_adc_read(uint32_t ch);

/**
 * @brief   启动 DMA 连续采集
 * @param   buf     数据缓冲区指针
 * @param   len     缓冲区长度
 * @note    DMA 完成后通过回调通知上层
 */
void mcal_adc_start_dma(uint32_t *buf, uint32_t len);

/**
 * @brief   停止 DMA 采集
 */
void mcal_adc_stop_dma(void);

/* ========== 回调函数（供 DMA 中断调用） ========== */

/**
 * @brief   DMA 传输完成回调（弱定义，可由上层覆盖）
 */
__weak void mcal_adc_dma_cplt_callback(void);

/**
 * @brief   DMA 半传输完成回调（弱定义）
 */
__weak void mcal_adc_dma_half_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_ADC_H */
