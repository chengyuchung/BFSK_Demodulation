/**
 * @file    mcal_clock.c
 * @brief   系统时钟初始化 - MCAL 层实现
 * @note    从原 main.c 的 SystemClock_Config() / PeriphCommonClock_Config() 移植
 *          HSI 16MHz → PLL (×10, /7) → SYSCLK 80MHz
 *          PLLSAI1 → ADC 时钟源
 */

#include "mcal_clock.h"
#include "main.h"          /* RCC HAL 类型定义 */

/**
 * @brief  错误处理（由 main.c 提供，作为 HAL 库的标准错误出口）
 */
extern void Error_Handler(void);

/* ================================================================ */
/*                   系统主时钟配置（80MHz）                          */
/* ================================================================ */
static void _system_clock_config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 电压调节器输出电压：Scale1（CPU 80MHz 必须） */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    /* 配置 HSI + PLL：HSI 16MHz × 10 / 7 ≈ 80MHz */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 1;
    RCC_OscInitStruct.PLL.PLLN            = 10;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ            = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR            = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /* 配置总线时钟：SYSCLK = PLLCLK，所有总线 DIV1 */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*              外设专用时钟配置（PLLSAI1 → ADC）                     */
/* ================================================================ */
static void _periph_clock_config(void)
{
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* ADC 时钟源选择 PLLSAI1 */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection    = RCC_ADCCLKSOURCE_PLLSAI1;

    /* PLLSAI1 配置：与主 PLL 相同的 N/M */
    PeriphClkInit.PLLSAI1.PLLSAI1Source  = RCC_PLLSOURCE_HSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M       = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N       = 10;
    PeriphClkInit.PLLSAI1.PLLSAI1P       = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q       = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R       = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void mcal_clock_init(void)
{
    _system_clock_config();
    _periph_clock_config();
}
