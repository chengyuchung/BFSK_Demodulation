/**
 * @file    mcal_gpio.h
 * @brief   GPIO 驱动 - MCAL 层
 * @note    统一 GPIO 读写接口，供继电器/功放/DS18B20 等使用
 */

#ifndef MCAL_GPIO_H
#define MCAL_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义：引脚编号（按项目实际分配） ========== */
#define GPIO_PIN_RELAY        0    /* 继电器控制 */
#define GPIO_PIN_AMP_EN       1    /* 功放使能 */
#define GPIO_PIN_DS18B20      2    /* DS18B20 单总线数据脚 */
#define GPIO_PIN_LED_STATUS   3    /* 状态指示灯 */

/* ========== 宏定义：电平 ========== */
#define GPIO_LOW   0U
#define GPIO_HIGH  1U

/* ========== 函数声明 ========== */

/**
 * @brief   GPIO 初始化
 * @note    在 CubeMX 生成后调用，配置各引脚模式
 */
void mcal_gpio_init(void);

/**
 * @brief   设置引脚电平
 * @param   pin   引脚编号
 * @param   level GPIO_LOW / GPIO_HIGH
 */
void mcal_gpio_write(uint8_t pin, uint8_t level);

/**
 * @brief   读取引脚电平
 * @param   pin   引脚编号
 * @retval  GPIO_LOW / GPIO_HIGH
 */
uint8_t mcal_gpio_read(uint8_t pin);

/**
 * @brief   配置引脚为输出
 * @param   pin   引脚编号
 */
void mcal_gpio_set_output(uint8_t pin);

/**
 * @brief   配置引脚为开漏输出（1-Wire 等总线使用）
 * @param   pin   引脚编号
 * @note    开漏 + 内部上拉，可避免多设备总线冲突
 *          DS18B20 等 1-Wire 设备必须使用此模式
 */
void mcal_gpio_set_output_od(uint8_t pin);

/**
 * @brief   配置引脚为输入
 * @param   pin   引脚编号
 */
void mcal_gpio_set_input(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_GPIO_H */
