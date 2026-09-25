/**
 * @file    mcal_timer.h
 * @brief   定时器驱动 - MCAL 层
 * @note    STM32L476 定时器统一管理：
 *              TIM1: 输入捕获（BFSK 信号测频，可选）
 *              TIM2: 1ms 周期基础定时器
 *              TIM6: 100kHz 周期定时器，TRGO 触发 ADC1
 */

#ifndef MCAL_TIMER_H
#define MCAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 定时器 ID ========== */
#define TIMER_ID_1    1
#define TIMER_ID_2    2
#define TIMER_ID_3    3

/* ========== 函数声明 ========== */

void mcal_timer_init(void);
void mcal_timer_delay_us(uint32_t us);
void mcal_timer_delay_ms(uint32_t ms);

void mcal_timer_start_once(uint8_t id, uint32_t period_us);
void mcal_timer_start_periodic(uint8_t id, uint32_t period_us);
void mcal_timer_stop(uint8_t id);

/* ========== Getter（给 stm32l4xx_it.c 用） ========== */
TIM_HandleTypeDef *mcal_timer_get_handle1(void);  /* 输入捕获 */
TIM_HandleTypeDef *mcal_timer_get_handle2(void);  /* 1ms 周期 */
TIM_HandleTypeDef *mcal_timer_get_handle6(void);  /* 100kHz 触发 ADC */

/* ========== 回调 ========== */
__weak void mcal_timer_callback(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_TIMER_H */
