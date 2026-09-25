/**
 * @file    mcal_uart.h
 * @brief   UART 驱动 - MCAL 层
 * @note    MCAL 层只暴露字节流原语（putc/puts/write）；
 *          printf 风格的格式化输出属于 BSW 层（见 bsw_log.h）。
 */

#ifndef MCAL_UART_H
#define MCAL_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

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
 * @brief   UART 发送字符串（不含自动换行）
 * @param   id    UART 号
 * @param   str   字符串
 */
void mcal_uart_puts(uint8_t id, const char *str);

/**
 * @brief   UART 字节流发送（通用原语）
 * @param   id    UART 号
 * @param   buf   数据缓冲区
 * @param   len   长度
 * @note    BSW 层（如 bsw_log）应优先使用本接口。
 */
void mcal_uart_write(uint8_t id, const uint8_t *buf, uint16_t len);

/* ========== Getter（给 stm32l4xx_it.c 用） ========== */
UART_HandleTypeDef *mcal_uart_get_handle1(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_UART_H */
