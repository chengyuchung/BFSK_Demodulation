/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_hal_msp.c
  * @brief   MSP 初始化（仅保留 Core 全局 + TIM）
  * @note    HAL_ADC / I2C / SPI / UART 的 MSP 已下沉到 MCAL 模块：
  *            - mcal_adc.c    -> HAL_ADC_MspInit / DeInit
  *            - mcal_i2c.c    -> HAL_I2C_MspInit / DeInit
  *            - mcal_spi.c    -> HAL_SPI_MspInit / DeInit
  *            - mcal_uart.c   -> HAL_UART_MspInit / DeInit
  *          TIM 的 MSP 暂保留在本文件（mcal_timer 后续可下沉）。
  ******************************************************************************
  * @attention
  * Copyright (c) 2026
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern DMA_HandleTypeDef hdma_adc1;

/* Private variables ---------------------------------------------------------*/
/*  ADC/I2C/SPI/UART 的时钟使能状态已下沉到对应 MCAL 模块内部 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* ============================================================
 *  Core 全局 MSP（保留）
 * ============================================================ */
void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* FreeRTOS 必需：PendSV 最低优先级 */
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

/* ============================================================
 *  TIM MSP（保留，待下沉到 mcal_timer）
 * ============================================================ */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim_ic)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (htim_ic->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM1 GPIO: PA8/PA9 -> CH1/CH2 */
        GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim_ic)
{
    if (htim_ic->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    } else if (htim_base->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TIM2_IRQn);
    } else if (htim_base->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_DISABLE();
    }
}

/* ============================================================
 *  以下 MSP 已下沉到 MCAL 层，本文件不再定义，避免多重定义。
 *  - HAL_ADC_MspInit/DeInit  -> mcal_adc.c
 *  - HAL_I2C_MspInit/DeInit  -> mcal_i2c.c
 *  - HAL_SPI_MspInit/DeInit  -> mcal_spi.c
 *  - HAL_UART_MspInit/DeInit -> mcal_uart.c
 * ============================================================ */
