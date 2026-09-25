/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   井下节点主程序入口
 * @note    本文件已精简：所有硬件初始化下沉到 MCAL 各模块
 *          - 系统时钟：mcal_clock_init()
 *          - 各外设：mcal_xxx_init()（在 APP 层 app_main_init() 中调用）
 *
 *          main.c 只保留：
 *          1. HAL 库初始化
 *          2. APP 入口
 *          3. FreeRTOS 调度器启动
 *          4. HAL 错误出口
 *          5. SysTick 回调
 *
 * @attention
 * Copyright (c) 2026
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "app_main.h"        /* APP 层入口 */

/* Private function prototypes -----------------------------------------------*/
extern void Error_Handler(void);   /* 实现见本文件末尾 */

/* ================================================================ */
/*                           主程序入口                               */
/* ================================================================ */
int main(void)
{
    /* 1. HAL 基础初始化（Flash 接口、SysTick 寄存器、NVIC 优先级分组等） */
    HAL_Init();

    /* 2. APP 层入口（内部按层级调用 mcal_clock_init → mcal_xxx_init → bsw_xxx_init → app_xxx_init） */
    app_main_init();

    /* 3. 启动 FreeRTOS 调度器（永不会返回） */
    osKernelStart();

    /* 正常情况下不会执行到这里 */
    while (1) { }
}

/* ================================================================ */
/*                  HAL SysTick 回调（保持原样）                     */
/* ================================================================ */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        HAL_IncTick();
    }
}

/* ================================================================ */
/*                  HAL 错误出口（保留）                              */
/* ================================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* 用户可在此打印文件名和行号 */
}
#endif /* USE_FULL_ASSERT */
