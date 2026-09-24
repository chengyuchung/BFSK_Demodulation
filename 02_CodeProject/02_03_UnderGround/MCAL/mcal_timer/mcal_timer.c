/**
 * @file    mcal_timer.c
 * @brief   定时器驱动 - MCAL 层实现
 * @note    仅实现 DS18B20 所需的微秒延时，基于 ARM CoreDebug DWT CYCCNT
 *          其他定时器（TIM2/TIM6）由 CubeMX 管理，APP 层直接调用 HAL API
 */

#include "mcal_timer.h"
#include "main.h"
#include "core_cm4.h"  /* CoreDebug + DWT 寄存器定义 */

/* ========== DWT (Debug Watchpoint and Trace) 宏 ========== */
/* DWT CYCCNT 是 ARM Cortex-M4 内核计数器，每 CPU 时钟周期 +1 */
/* STM32L476 主频 80MHz */

/* ========== 内部辅助 ========== */

/* DWT CYCCNT 使能（需先解锁） */
static void _dwt_enable(void)
{
    /* STM32L4 需要先写 DWT_LAR 解锁，否则读写出错 */
    DWT_LAR = 0xC5ACCE55;

    /* 使能 TRCENA */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 使能 CYCCNT */
    DWT_CTRL |= 0x01;
}

/* DWT 初始化（仅调用一次） */
static int s_dwt_inited = 0;

static void _dwt_init(void)
{
    if (s_dwt_inited) return;

    /* 检查 CYCCNT 是否已使能 */
    if ((DWT_CTRL & 0x01) == 0) {
        _dwt_enable();
    }
    DWT_CYCCNT = 0;
    s_dwt_inited = 1;
}

/**
 * @brief   微秒延时（阻塞）
 * @param   us   延时微秒数
 * @note    基于 DWT CPU 周期计数，精度最高
 */
void mcal_timer_delay_us(uint32_t us)
{
    _dwt_init();

    uint32_t start = DWT_CYCCNT;
    /* 每次循环 +1，时钟周期数 = us * SystemCoreClock / 1,000,000 */
    uint32_t cycles = us * (SystemCoreClock / 1000000U);

    /* 处理溢出情况 */
    while ((DWT_CYCCNT - start) < cycles) {
        __NOP();
    }
}

/* ================================================================ */
/*              以下接口暂不实现（CubeMX 已管理）                      */
/* ================================================================ */

void mcal_timer_init(void) {}

void mcal_timer_delay_ms(uint32_t ms)
{
    /* CubeMX 的 HAL_Delay() 已实现，直接调用 */
    HAL_Delay(ms);
}

void mcal_timer_start_once(uint8_t id, uint32_t period_us)
{
    (void)id;
    (void)period_us;
    /* TIM2/TIM6 由 CubeMX 配置，APP 层直接使用 HAL_TIM */
}

void mcal_timer_start_periodic(uint8_t id, uint32_t period_us)
{
    (void)id;
    (void)period_us;
}

void mcal_timer_stop(uint8_t id)
{
    (void)id;
}

__weak void mcal_timer_callback(uint8_t id)
{
    (void)id;
}
