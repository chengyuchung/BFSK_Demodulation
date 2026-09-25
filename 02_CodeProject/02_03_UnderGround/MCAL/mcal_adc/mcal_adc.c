/**
 * @file    mcal_adc.c
 * @brief   ADC 采样驱动 - MCAL 层实现
 *
 * 双 ADC 架构：
 *   - ADC1: TIM6 TRGO 触发 → DMA 循环 → 半传输/全传输回调
 *   - ADC2: 软件触发 → 阻塞读取
 *
 * @note    原 main.c 的 MX_ADC1_Init() 和 MX_ADC2_Init() 已移植到本文件的
 *          内部 static 函数。
 */

#include "mcal_adc.h"
#include "mcal_dma.h"   /* mcal_dma_get_handle_adc1() */

/* ========== 内部类型 ========== */
typedef struct {
    ADC_HandleTypeDef    *hadc;
    uint8_t               inited;
    uint8_t               dma_running;
    uint16_t             *cur_buf;
    uint32_t              cur_len;
    mcal_adc_dma_cplt_cb_t cplt_cb;
    mcal_adc_dma_half_cb_t half_cb;
} mcal_adc_dev_ctx_t;

/* ========== 内部变量 ========== */
static ADC_HandleTypeDef s_hadc1;
static ADC_HandleTypeDef s_hadc2;
static mcal_adc_dev_ctx_t s_dev[MCAL_ADC_DEV_MAX] = {0};

/* ========== HAL ADC MSP 初始化重写 ========== */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hadc == &s_hadc1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /* PA6 = ADC1_IN6 */
        GPIO_InitStruct.Pin = GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* 把 ADC1 链接到 DMA1_Channel1（取自 mcal_dma 模块） */
        DMA_HandleTypeDef *hdma = mcal_dma_get_handle_adc1();
        __HAL_LINKDMA(hadc, DMA_Handle, *hdma);
    } else if (hadc == &s_hadc2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /* PB1 = ADC2_IN9 */
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/* ================================================================ */
/*                    原 MX_ADC1_Init 内容（移植）                    */
/* ================================================================ */
static void _adc1_init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    s_hadc1.Instance = ADC1;
    s_hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
    s_hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    s_hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    s_hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    s_hadc1.Init.LowPowerAutoWait      = DISABLE;
    s_hadc1.Init.ContinuousConvMode    = DISABLE;
    s_hadc1.Init.NbrOfConversion       = 1;
    s_hadc1.Init.DiscontinuousConvMode = DISABLE;
    s_hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T6_TRGO;
    s_hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
    s_hadc1.Init.DMAContinuousRequests = ENABLE;
    s_hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    s_hadc1.Init.OversamplingMode      = DISABLE;

    if (HAL_ADC_Init(&s_hadc1) != HAL_OK) {
        extern void Error_Handler(void);
        Error_Handler();
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&s_hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel         = ADC_CHANNEL_6;
    sConfig.Rank            = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime    = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff      = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber    = ADC_OFFSET_NONE;
    sConfig.Offset          = 0;
    if (HAL_ADC_ConfigChannel(&s_hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                    原 MX_ADC2_Init 内容（移植）                    */
/* ================================================================ */
static void _adc2_init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    s_hadc2.Instance = ADC2;
    s_hadc2.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV2;
    s_hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
    s_hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    s_hadc2.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    s_hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    s_hadc2.Init.LowPowerAutoWait      = DISABLE;
    s_hadc2.Init.ContinuousConvMode    = DISABLE;
    s_hadc2.Init.NbrOfConversion       = 1;
    s_hadc2.Init.DiscontinuousConvMode = DISABLE;
    s_hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    s_hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_hadc2.Init.DMAContinuousRequests = DISABLE;
    s_hadc2.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
    s_hadc2.Init.OversamplingMode      = DISABLE;

    if (HAL_ADC_Init(&s_hadc2) != HAL_OK) {
        extern void Error_Handler(void);
        Error_Handler();
    }

    sConfig.Channel         = ADC_CHANNEL_9;
    sConfig.Rank            = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime    = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff      = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber    = ADC_OFFSET_NONE;
    sConfig.Offset          = 0;
    if (HAL_ADC_ConfigChannel(&s_hadc2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
mcal_adc_ret_t mcal_adc_init(void)
{
    /* ADC1 初始化（含 DMA 绑定） */
    _adc1_init();
    s_dev[MCAL_ADC_DEV1].hadc        = &s_hadc1;
    s_dev[MCAL_ADC_DEV1].inited      = 1;
    s_dev[MCAL_ADC_DEV1].dma_running = 0;
    s_dev[MCAL_ADC_DEV1].cplt_cb     = NULL;
    s_dev[MCAL_ADC_DEV1].half_cb     = NULL;

    /* ADC2 初始化 */
    _adc2_init();
    s_dev[MCAL_ADC_DEV2].hadc        = &s_hadc2;
    s_dev[MCAL_ADC_DEV2].inited      = 1;
    s_dev[MCAL_ADC_DEV2].dma_running = 0;
    s_dev[MCAL_ADC_DEV2].cplt_cb     = NULL;
    s_dev[MCAL_ADC_DEV2].half_cb     = NULL;

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
        return MCAL_ADC_OK;
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

/* ========== Getter 实现 ========== */
ADC_HandleTypeDef *mcal_adc_get_handle1(void) { return &s_hadc1; }
ADC_HandleTypeDef *mcal_adc_get_handle2(void) { return &s_hadc2; }

/* ================================================================ */
/*                  HAL ADC 回调重写（中断上下文）                     */
/* ================================================================ */
static mcal_adc_dev_t _find_dev_by_hadc(ADC_HandleTypeDef *hadc)
{
    for (uint8_t i = 0; i < MCAL_ADC_DEV_MAX; i++) {
        if (s_dev[i].inited && s_dev[i].hadc == hadc) {
            return (mcal_adc_dev_t)i;
        }
    }
    return MCAL_ADC_DEV_MAX;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    mcal_adc_dev_t dev = _find_dev_by_hadc(hadc);
    if (dev >= MCAL_ADC_DEV_MAX) return;
    if (s_dev[dev].cplt_cb) {
        s_dev[dev].cplt_cb(s_dev[dev].cur_buf, s_dev[dev].cur_len);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    mcal_adc_dev_t dev = _find_dev_by_hadc(hadc);
    if (dev >= MCAL_ADC_DEV_MAX) return;
    if (s_dev[dev].half_cb) {
        s_dev[dev].half_cb(s_dev[dev].cur_buf, s_dev[dev].cur_len);
    }
}
