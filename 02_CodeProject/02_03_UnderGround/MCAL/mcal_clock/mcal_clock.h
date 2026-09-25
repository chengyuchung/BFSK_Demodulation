/**
 * @file    mcal_clock.h
 * @brief   系统时钟初始化 - MCAL 层
 * @note    替代 CubeMX 的 SystemClock_Config() + PeriphCommonClock_Config()
 *          配置 HSI+PLL → 80MHz SYSCLK，并启用 PLLSAI1 给 ADC
 */

#ifndef MCAL_CLOCK_H
#define MCAL_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   初始化系统时钟 + 外设时钟
 * @note    必须在所有 MCAL 模块初始化之前调用
 *          失败会调用 Error_Handler()（由 main.c 提供）
 */
void mcal_clock_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_CLOCK_H */
