/**
 * @file    mcal_timer.h
 * @brief   定时器驱动 - MCAL 层
 * @note    STM32L476 定时器统一管理：
 *              TIM1: 输入捕获（BFSK 信号测频）
 *              TIM2: 1ms 周期基础定时器
 *              TIM6: 10us 周期定时器，TRGO 触发 ADC1
 *
 *          本层仅做两件事：
 *              1) 硬件初始化（init）
 *              2) 提供 us/ms 级延时
 *              3) 暴露 HAL 句柄给 stm32l4xx_it.c 的 IRQ 入口
 *
 *          业务侧若需计时/调度，应基于本延时 + FreeRTOS 实现，
 *          而非再向 MCAL 加 start/stop 空壳。
 */

#ifndef MCAL_TIMER_H
#define MCAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 函数声明 ========== */

/**
 * @brief   初始化 TIM1（输入捕获）/ TIM2（1ms tick）/ TIM6（10us 触发 ADC）
 * @note    必须在 HAL_Init() 之后、调用 ADC 采集之前调用
 */
void mcal_timer_init(void);

/**
 * @brief   精确微秒级阻塞延时（基于 DWT->CYCCNT）
 * @param   us  延时微秒数
 * @note    延时期间 CPU 空转。短延时（us 级）专用，
 *          ms 级请用 mcal_timer_delay_ms()（走 HAL_Delay，可在 RTOS 下让出）。
 */
void mcal_timer_delay_us(uint32_t us);

/**
 * @brief   毫秒级阻塞延时（走 HAL_Delay）
 * @param   ms  延时毫秒数
 */
void mcal_timer_delay_ms(uint32_t ms);

/* ========== HAL 句柄访问器（仅供 stm32l4xx_it.c 的 IRQ 入口使用） ========== */
TIM_HandleTypeDef *mcal_timer_get_handle1(void);  /* TIM1: 输入捕获 */
TIM_HandleTypeDef *mcal_timer_get_handle2(void);  /* TIM2: 1ms 周期 */
/* TIM6 通过 TRGO 硬件触发 ADC，无需 CPU 中断，因此不导出句柄访问器 */

#ifdef __cplusplus
}
#endif

#endif /* MCAL_TIMER_H */
