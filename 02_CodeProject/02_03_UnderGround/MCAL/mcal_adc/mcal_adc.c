/**
 * @file    mcal_adc.c
 * @brief   ADC 采样驱动 - MCAL 层实现
 *
 * 双 ADC 架构：
 *   - ADC1: TIM6 TRGO 触发 → DMA 循环 → 半传输/全传输回调
 *   - ADC2: 软件触发 → 阻塞读取
 *
 * HAL 回调重写：
 *   本文件重写 HAL_ADC_ConvCpltCallback / HAL_ADC_ConvHalfCpltCallback
 *   把中断事件分发给注册好的用户回调
 */

#include "mcal_adc.h"

/* ========== 内部类型 ========== */
typedef struct {
    ADC_HandleTypeDef    *hadc;          /* HAL 句柄 */
    uint8_t               inited;        /* 是否已初始化 */
    uint8_t               dma_running;   /* DMA 是否在跑 */
    uint16_t             *cur_buf;       /* 当前 DMA 缓冲区 */
    uint32_t              cur_len;       /* 当前 DMA 长度 */
    mcal_adc_dma_cplt_cb_t cplt_cb;      /* 传输完成回调 */
    mcal_adc_dma_half_cb_t half_cb;      /* 半传输回调 */
} mcal_adc_dev_ctx_t;

/* ========== 内部变量 ========== */
static mcal_adc_dev_ctx_t s_dev[MCAL_ADC_DEV_MAX] = {0};

/* ========== 内部辅助：根据 hadc 找 dev ========== */
static mcal_adc_dev_t find_dev_by_hadc(ADC_HandleTypeDef *hadc)
{
    for (uint8_t i = 0; i < MCAL_ADC_DEV_MAX; i++) {
        if (s_dev[i].inited && s_dev[i].hadc == hadc) {
            return (mcal_adc_dev_t)i;
        }
    }
    return MCAL_ADC_DEV_MAX;
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */

mcal_adc_ret_t mcal_adc_init(mcal_adc_dev_t dev, ADC_HandleTypeDef *hadc)
{
    if (dev >= MCAL_ADC_DEV_MAX || hadc == NULL) {
        return MCAL_ADC_ERR_INVALID;
    }

    s_dev[dev].hadc        = hadc;
    s_dev[dev].inited      = 1;
    s_dev[dev].dma_running = 0;
    s_dev[dev].cplt_cb     = NULL;
    s_dev[dev].half_cb     = NULL;

    /* ADC1 已经由 CubeMX 配置为 TIM6 TRGO 触发，不需要软件启动
     * ADC2 每次单次读时再启动 */

    return MCAL_ADC_OK;
}

mcal_adc_ret_t mcal_adc_read_poll(mcal_adc_dev_t dev, uint16_t *value)
{
    if (dev >= MCAL_ADC_DEV_MAX || value == NULL || !s_dev[dev].inited) {
        return MCAL_ADC_ERR_INVALID;
    }

    if (HAL_ADC_Start(s_dev[dev].hadc) != HAL_OK) {
        return MCAL_ADC_ERR_BUSY;
    }

    if (HAL_ADC_PollForConversion(s_dev[dev].hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(s_dev[dev].hadc);
        return MCAL_ADC_ERR_TIMEOUT;
    }

    *value = (uint16_t)HAL_ADC_GetValue(s_dev[dev].hadc);
    HAL_ADC_Stop(s_dev[dev].hadc);
    return MCAL_ADC_OK;
}

mcal_adc_ret_t mcal_adc_start_dma(mcal_adc_dev_t dev, uint16_t *buf, uint32_t len)
{
    if (dev >= MCAL_ADC_DEV_MAX || buf == NULL || len == 0 || !s_dev[dev].inited) {
        return MCAL_ADC_ERR_INVALID;
    }

    if (s_dev[dev].dma_running) {
        return MCAL_ADC_ERR_BUSY;
    }

    s_dev[dev].cur_buf = buf;
    s_dev[dev].cur_len = len;

    /* HAL_ADC_Start_DMA 内部启动 ADC + 配置 DMA 循环模式 */
    if (HAL_ADC_Start_DMA(s_dev[dev].hadc, (uint32_t *)buf, len) != HAL_OK) {
        return MCAL_ADC_ERR_DMA;
    }

    s_dev[dev].dma_running = 1;
    return MCAL_ADC_OK;
}

mcal_adc_ret_t mcal_adc_stop_dma(mcal_adc_dev_t dev)
{
    if (dev >= MCAL_ADC_DEV_MAX || !s_dev[dev].inited) {
        return MCAL_ADC_ERR_INVALID;
    }

    if (!s_dev[dev].dma_running) {
        return MCAL_ADC_OK;   /* 没在跑，静默成功 */
    }

    HAL_ADC_Stop_DMA(s_dev[dev].hadc);
    s_dev[dev].dma_running = 0;
    return MCAL_ADC_OK;
}

mcal_adc_ret_t mcal_adc_register_dma_cplt(mcal_adc_dev_t dev, mcal_adc_dma_cplt_cb_t cb)
{
    if (dev >= MCAL_ADC_DEV_MAX || !s_dev[dev].inited) {
        return MCAL_ADC_ERR_INVALID;
    }
    s_dev[dev].cplt_cb = cb;
    return MCAL_ADC_OK;
}

mcal_adc_ret_t mcal_adc_register_dma_half(mcal_adc_dev_t dev, mcal_adc_dma_half_cb_t cb)
{
    if (dev >= MCAL_ADC_DEV_MAX || !s_dev[dev].inited) {
        return MCAL_ADC_ERR_INVALID;
    }
    s_dev[dev].half_cb = cb;
    return MCAL_ADC_OK;
}

/* ================================================================ */
/*                  HAL ADC 回调重写（中断上下文）                     */
/* ================================================================ */

/**
 * @brief HAL ADC 转换完成回调（DMA 模式下 = DMA 传输完成）
 * @note  重写 __weak 默认实现，根据 hadc 找到 dev 并回调用户
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mcal_adc_dev_t dev = find_dev_by_hadc(hadc);
    if (dev >= MCAL_ADC_DEV_MAX) return;

    if (s_dev[dev].cplt_cb) {
        s_dev[dev].cplt_cb(s_dev[dev].cur_buf, s_dev[dev].cur_len);
    }
}

/**
 * @brief HAL ADC 半传输完成回调
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    mcal_adc_dev_t dev = find_dev_by_hadc(hadc);
    if (dev >= MCAL_ADC_DEV_MAX) return;

    if (s_dev[dev].half_cb) {
        s_dev[dev].half_cb(s_dev[dev].cur_buf, s_dev[dev].cur_len);
    }
}
