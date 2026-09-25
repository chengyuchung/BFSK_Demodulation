/**
 * @file    mcal_dma.c
 * @brief   DMA 驱动 - MCAL 层实现
 * @note    从原 main.c 的 MX_DMA_Init() 移植
 *          当前项目只用 DMA1_Channel1（ADC1 触发）
 */

#include "mcal_dma.h"

/* ========== 内部句柄（main.c 的 hdma_adc1 搬过来） ========== */
static DMA_HandleTypeDef s_hdma_adc1;

/* ========== HAL DMA MSP 初始化回调（取代原 HAL_MspInit 部分） ========== */
/* STM32 HAL 在 HAL_DMA_Init() 内部会调用 HAL_DMA_MspInit()，
 * 我们把它重写，让所有 DMA 外设的底层初始化都集中在 MCAL 层。
 * 注意：此函数是 HAL 提供的 __weak 实现的重写。 */
void HAL_DMA_MspInit(DMA_HandleTypeDef *hdma)
{
    if (hdma == &s_hdma_adc1) {
        /* DMA1 时钟已在 mcal_dma_init() 中使能 */
        /* NVIC 配置也已在 mcal_dma_init() 中完成 */
    }
}

/* ========== Getter 实现 ========== */
DMA_HandleTypeDef *mcal_dma_get_handle_adc1(void)
{
    return &s_hdma_adc1;
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void mcal_dma_init(void)
{
    /* 启用 DMA1 时钟 */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* NVIC 中断配置：DMA1_Channel1（ADC1 DMA 使用） */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* 注册 s_hdma_adc1 给 mcal_adc 模块使用（如果需要可补充初始化） */
    s_hdma_adc1.Instance                 = DMA1_Channel1;
    s_hdma_adc1.Init.Request             = DMA_REQUEST_0;     /* ADC1 */
    s_hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    s_hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    s_hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    s_hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&s_hdma_adc1) != HAL_OK) {
        /* 错误处理（HAL 库默认走 Error_Handler） */
        extern void Error_Handler(void);
        Error_Handler();
    }

    /* 绑定 DMA 句柄到 ADC1（在 mcal_adc_init 中调用 __HAL_LINKDMA 完成） */
}

void mcal_dma_start(uint32_t stream, uint32_t src, uint32_t dst, uint32_t len)
{
    /* 当前项目没有用到通用 DMA 启动接口，保留为空实现 */
    (void)stream; (void)src; (void)dst; (void)len;
}

void mcal_dma_stop(uint32_t stream)
{
    (void)stream;
}

__weak void mcal_dma_cplt_callback(uint32_t stream)
{
    (void)stream;
}
