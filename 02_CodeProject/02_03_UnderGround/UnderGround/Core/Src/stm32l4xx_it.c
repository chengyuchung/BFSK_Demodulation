/* USER CODE BEGIN Header */
/**
 * @file    stm32l4xx_it.c
 * @brief   Interrupt Service Routines
 * @note    所有 HAL 句柄已下沉到 MCAL 层
 *          ISR 通过 MCAL 提供的 getter 函数获取句柄
 *
 * @attention
 * Copyright (c) 2026
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_it.h"

/* ========== MCAL Getter 声明（替代原 extern 全局句柄） ========== */
#include "mcal_dma.h"      /* mcal_dma_get_handle_adc1 */
#include "mcal_adc.h"      /* mcal_adc_get_handle1/2   */
#include "mcal_timer.h"    /* mcal_timer_get_handle1/2 */
#include "mcal_uart.h"     /* mcal_uart_get_handle1    */

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* ========== Cortex-M4 异常处理（保留原样） ========== */
void NMI_Handler(void)
{
    while (1) { }
}

void HardFault_Handler(void)
{
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    while (1) { }
}

void UsageFault_Handler(void)
{
    while (1) { }
}

void DebugMon_Handler(void) { }

/* ========== 外设中断处理 ========== */

/* DMA1 channel1：ADC1 DMA */
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(mcal_dma_get_handle_adc1());
}

/* ADC1 & ADC2 */
void ADC1_2_IRQHandler(void)
{
    HAL_ADC_IRQHandler(mcal_adc_get_handle1());
    HAL_ADC_IRQHandler(mcal_adc_get_handle2());
}

/* TIM1 输入捕获 */
void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(mcal_timer_get_handle1());
}

/* TIM2 */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(mcal_timer_get_handle2());
}

/* USART1 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(mcal_uart_get_handle1());
}

/* 注意：项目使用 SysTick 作为 HAL 时间基（默认），不需要 TIM7 ISR。
 * 如果未来切换 TIM7 作为 timebase，需在 mcal_timer 模块定义 htim7 全局变量
 * 并在此处恢复 void TIM7_IRQHandler(void) { HAL_TIM_IRQHandler(&htim7); } */
