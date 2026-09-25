/**
 * @file    mcal_uart.h
 * @brief   UART 驱动 - MCAL 层
 * @note    用于调试打印
 */

#ifndef MCAL_UART_H
#define MCAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdarg.h>

/* ========== 宏定义 ========== */
#define UART_ID_DEBUG  1   /* 调试串口 */

/* ========== 函数声明 ========== */

/**
 * @brief   UART 初始化
 * @param   id   UART 号
 * @param   baud 波特率
 */
void mcal_uart_init(uint8_t id, uint32_t baud);

/**
 * @brief   UART 发送一个字节
 * @param   id    UART 号
 * @param   data  数据
 */
void mcal_uart_putc(uint8_t id, uint8_t data);

/**
 * @brief   UART 发送字符串
 * @param   id    UART 号
 * @param   str   字符串
 */
void mcal_uart_puts(uint8_t id, const char *str);

/**
 * @brief   UART printf（简易版，支持 %d %s %x %c）
 * @param   id    UART 号
 * @param   fmt   格式化字符串
 * @param   ...   可变参数
 */
void mcal_uart_printf(uint8_t id, const char *fmt, ...);

/* ========== Getter（给 stm32l4xx_it.c 用） ========== */
UART_HandleTypeDef *mcal_uart_get_handle1(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_UART_H */
