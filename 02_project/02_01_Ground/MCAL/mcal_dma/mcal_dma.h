/**
 * @file    mcal_dma.h
 * @brief   DMA 驱动 - MCAL 层
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
 */
void mcal_dma_init(void);

/**
 * @brief   启动 DMA 传输
 * @param   stream    DMA Stream 编号
 * @param   src       源地址
 * @param   dst       目的地址
 * @param   len       数据长度（字节）
 */
void mcal_dma_start(uint32_t stream, uint32_t src, uint32_t dst, uint32_t len);

/**
 * @brief   停止 DMA 传输
 * @param   stream    DMA Stream 编号
 */
void mcal_dma_stop(uint32_t stream);

/* ========== 回调 ========== */
__weak void mcal_dma_cplt_callback(uint32_t stream);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_DMA_H */
