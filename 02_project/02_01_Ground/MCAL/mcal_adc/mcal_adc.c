/**
 * @file    mcal_adc.c
 * @brief   ADC 采样驱动实现
 */

#include "mcal_adc.h"

/* ========== 变量 ========== */
static uint32_t g_adc_buf[1024];  /* 示例 DMA 缓冲区 */

/* ========== 函数实现 ========== */

void mcal_adc_init(void)
{
    /* TODO: 调用 HAL_ADC_Init() */
}

uint32_t mcal_adc_read(uint32_t ch)
{
    /* TODO: 单次轮询读取 */
    return 0;
}

void mcal_adc_start_dma(uint32_t *buf, uint32_t len)
{
    /* TODO: 启动 DMA 连续采集 */
    (void)buf;
    (void)len;
}

void mcal_adc_stop_dma(void)
{
    /* TODO: 停止 DMA */
}

/* ========== 回调 ========== */

__weak void mcal_adc_dma_cplt_callback(void)
{
    /* TODO: 由 BSW 层实现具体逻辑 */
}

__weak void mcal_adc_dma_half_callback(void)
{
    /* TODO: 半传输中断处理 */
}
