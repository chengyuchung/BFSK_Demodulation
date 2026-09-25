/**
 * @file    mcal_adc.h
 * @brief   ADC 采样驱动 - MCAL 层
 * @note    双 ADC 架构：
 *            - ADC1: TIM6 TRGO + DMA 循环（高速采样，给 BFSK 用）
 *            - ADC2: 软件触发单次读（慢速监控）
 *
 * @dependency   HAL_ADC, HAL_DMA
 */

#ifndef MCAL_ADC_H
#define MCAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 类型定义 ========== */

typedef enum {
    MCAL_ADC_DEV1 = 0,   /**< ADC1: TIM6 TRGO + DMA 循环（高速） */
    MCAL_ADC_DEV2,       /**< ADC2: 软件触发单次读（慢速监控） */
    MCAL_ADC_DEV_MAX
} mcal_adc_dev_t;

typedef enum {
    MCAL_ADC_OK = 0,
    MCAL_ADC_ERR_INVALID = -1,
    MCAL_ADC_ERR_NOT_INIT = -2,
    MCAL_ADC_ERR_TIMEOUT = -3,
    MCAL_ADC_ERR_DMA = -4,
    MCAL_ADC_ERR_BUSY = -5
} mcal_adc_ret_t;

typedef void (*mcal_adc_dma_cplt_cb_t)(uint16_t *buf, uint32_t len);
typedef void (*mcal_adc_dma_half_cb_t)(uint16_t *buf, uint32_t len);

/* ========== 函数声明 ========== */

/**
 * @brief   初始化 ADC1 + ADC2（内部完成 HAL_ADC_Init）
 * @note    必须先调用 mcal_dma_init() 和 mcal_timer_init()
 *          失败会调用 Error_Handler()
 */
mcal_adc_ret_t mcal_adc_init(void);

/**
 * @brief   单次轮询读取（阻塞，适用于 ADC2）
 */
mcal_adc_ret_t mcal_adc_read_poll(mcal_adc_dev_t dev, uint16_t *value);

/**
 * @brief   启动 DMA 连续采样（适用于 ADC1）
 * @note    采样由 TIM6 触发，DMA 循环搬运到 buf
 */
mcal_adc_ret_t mcal_adc_start_dma(mcal_adc_dev_t dev, uint16_t *buf, uint32_t len);

mcal_adc_ret_t mcal_adc_stop_dma(mcal_adc_dev_t dev);

mcal_adc_ret_t mcal_adc_register_dma_cplt(mcal_adc_dev_t dev, mcal_adc_dma_cplt_cb_t cb);
mcal_adc_ret_t mcal_adc_register_dma_half(mcal_adc_dev_t dev, mcal_adc_dma_half_cb_t cb);

/* ========== Getter（给 stm32l4xx_it.c 用） ========== */
ADC_HandleTypeDef *mcal_adc_get_handle1(void);
ADC_HandleTypeDef *mcal_adc_get_handle2(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_ADC_H */
