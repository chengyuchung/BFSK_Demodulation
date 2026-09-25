/**
 * @file    mcal_dma.h
 * @brief   DMA 驱动 - MCAL 层
 * @note    STM32L4 DMA 通道管理（基于 HAL DMA）
 */

#ifndef MCAL_DMA_H
#define MCAL_DMA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 函数声明 ========== */

/**
 * @brief   DMA 初始化
 * @note    启用 DMA1 时钟并配置 NVIC 中断（来自原 MX_DMA_Init）
 *          通道句柄由各外设模块自己管理
 */
void mcal_dma_init(void);

/**
 * @brief   启动 DMA 传输（保留接口，方便后续扩展）
 */
void mcal_dma_start(uint32_t stream, uint32_t src, uint32_t dst, uint32_t len);

/**
 * @brief   停止 DMA 传输（保留接口）
 */
void mcal_dma_stop(uint32_t stream);

/* ========== Getter（给 stm32l4xx_it.c 用） ========== */

/**
 * @brief   获取 ADC1 使用的 DMA 句柄（DMA1_Channel1）
 * @note    供中断服务程序 HAL_DMA_IRQHandler() 调用
 */
DMA_HandleTypeDef *mcal_dma_get_handle_adc1(void);

/* ========== 回调 ========== */
__weak void mcal_dma_cplt_callback(uint32_t stream);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_DMA_H */
