/**
 * @file    mcal_timer.c
 * @brief   定时器驱动 - MCAL 层实现
 * @note    原 main.c 的 MX_TIM1_Init / MX_TIM2_Init / MX_TIM6_Init 已移植到本文件
 *          - TIM1: 输入捕获（IC1上升沿 + IC2下降沿）
 *          - TIM2: 基础定时器，1ms 周期（80MHz / (79+1) / (999+1) = 1kHz）
 *          - TIM6: 基础定时器，10μs 周期，TRGO 触发 ADC1
 *          - DWT 寄存器仍用于微秒级精确延时
 */

#include "mcal_timer.h"
#include "core_cm4.h"

extern void Error_Handler(void);

/* ========== 内部句柄 ========== */
static TIM_HandleTypeDef s_htim1;
static TIM_HandleTypeDef s_htim2;
static TIM_HandleTypeDef s_htim6;

/* ========== Getter ========== */
TIM_HandleTypeDef *mcal_timer_get_handle1(void) { return &s_htim1; }
TIM_HandleTypeDef *mcal_timer_get_handle2(void) { return &s_htim2; }
/* TIM6 通过 TRGO 硬件触发 ADC，不需要 IRQ 句柄访问器 */

/* ========== DWT 微秒延时（保留） ========== */
static void _dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static int s_dwt_inited = 0;
static void _dwt_init(void)
{
    if (s_dwt_inited) return;
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {
        _dwt_enable();
    }
    DWT->CYCCNT = 0;
    s_dwt_inited = 1;
}

void mcal_timer_delay_us(uint32_t us)
{
    _dwt_init();
    uint32_t start  = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < cycles) {
        __NOP();
    }
}

/* ================================================================ */
/*                 原 MX_TIM1_Init 内容（输入捕获）                   */
/* ================================================================ */
static void _tim1_init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef     sConfigIC     = {0};

    s_htim1.Instance               = TIM1;
    s_htim1.Init.Prescaler         = 0;
    s_htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim1.Init.Period            = 65535;
    s_htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim1.Init.RepetitionCounter = 0;
    s_htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_IC_Init(&s_htim1) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger   = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2  = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode       = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter    = 0;
    if (HAL_TIM_IC_ConfigChannel(&s_htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    if (HAL_TIM_IC_ConfigChannel(&s_htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*              原 MX_TIM2_Init 内容（1ms 基础定时器）                */
/* ================================================================ */
static void _tim2_init(void)
{
    TIM_ClockConfigTypeDef  sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig      = {0};

    s_htim2.Instance               = TIM2;
    s_htim2.Init.Prescaler         = 79;
    s_htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim2.Init.Period            = 999;
    s_htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&s_htim2) != HAL_OK) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&s_htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*           原 MX_TIM6_Init 内容（10μs TRGO 触发 ADC）              */
/* ================================================================ */
static void _tim6_init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    s_htim6.Instance               = TIM6;
    s_htim6.Init.Prescaler         = 79;
    s_htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim6.Init.Period            = 99;
    s_htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&s_htim6) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&s_htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void mcal_timer_init(void)
{
    _tim1_init();
    _tim2_init();
    _tim6_init();
    /* 注意：TIM6 此处不启动，由 bsw_adc_ringbuf_enable() 按状态控制 */
}

void mcal_timer_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ========== TIM6 base start/stop（由 bsw_adc_ringbuf 按状态机启停） ========== */

void mcal_timer_base_start(mcal_timer_id_t id)
{
    TIM_HandleTypeDef *htim = NULL;
    switch (id) {
        case MCAL_TIMER_TIM1: htim = &s_htim1; break;
        case MCAL_TIMER_TIM2: htim = &s_htim2; break;
        case MCAL_TIMER_TIM6: htim = &s_htim6; break;
        default: return;
    }
    (void)HAL_TIM_Base_Start(htim);
}

void mcal_timer_base_stop(mcal_timer_id_t id)
{
    TIM_HandleTypeDef *htim = NULL;
    switch (id) {
        case MCAL_TIMER_TIM1: htim = &s_htim1; break;
        case MCAL_TIMER_TIM2: htim = &s_htim2; break;
        case MCAL_TIMER_TIM6: htim = &s_htim6; break;
        default: return;
    }
    (void)HAL_TIM_Base_Stop(htim);
}

/* ========== TIM1 输入捕获启停（仅供 BFSK 解调用，FSM 控制） ==========
 * 注意：仅 LINKED 态调用 ic_start，其余 5 个状态必须 ic_stop。
 *   - LINKED:    TIM1 IC 启（BFSK 帧解调：CH1 上升沿 + CH2 下降沿 → 测频）
 *   - SCAN:      TIM1 IC 停（频点由 ADC ringbuf 的 FFT 解出）
 *   - SCAN_LISTEN: TIM1 IC 停（杂散边沿会污染 BSW 解调状态机）
 *   - BOOT/SLEEP/FAULT: TIM1 IC 停 */

void mcal_timer_ic_start(mcal_timer_id_t id)
{
    if (id != MCAL_TIMER_TIM1) {
        return;   /* 仅 TIM1 支持 IC */
    }
    /* CH1 上升沿：每周期一记，bsw_bfsk_demod 读 CCR1 求瞬时频率 */
    if (HAL_TIM_IC_Start_IT(&s_htim1, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* CH2 下降沿：bit 边界检测（0/1 码元宽度） */
    if (HAL_TIM_IC_Start_IT(&s_htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* TIM1 计数器本身已经在 _tim1_init() 配好 16-bit ARR=65535，
     * 进入 IC 模式后计数器自由跑，到 0xFFFF 自动回零，
     * 不需要额外 HAL_TIM_Base_Start。 */
}

void mcal_timer_ic_stop(mcal_timer_id_t id)
{
    if (id != MCAL_TIMER_TIM1) {
        return;   /* 仅 TIM1 支持 IC */
    }
    (void)HAL_TIM_IC_Stop_IT(&s_htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_IC_Stop_IT(&s_htim1, TIM_CHANNEL_2);
}
