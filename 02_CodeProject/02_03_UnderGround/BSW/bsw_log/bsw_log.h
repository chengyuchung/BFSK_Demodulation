/**
 * @file    bsw_log.h
 * @brief   日志服务 - BSW 层
 * @note    把格式化输出从 MCAL UART 抽到 BSW 层。
 *          MCAL 只暴露 mcal_uart_write() 这类字节流原语；
 *          printf 风格的格式化属于软件层职责。
 *
 * 用法:
 *      bsw_log("Hello %s, val=%d\n", "world", 42);
 *
 * 默认输出到 UART_ID_DEBUG，可用 bsw_log_set_uart() 切换。
 */

#ifndef BSW_LOG_H
#define BSW_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief   设置日志输出通道
 * @param   uart_id  UART 号（参见 mcal_uart.h 的 UART_ID_* 宏）
 */
void bsw_log_set_uart(uint8_t uart_id);

/**
 * @brief   格式化输出（printf 风格，支持 %d %u %x %s %c %% 等）
 * @note    内部缓冲 128 字节；超出部分截断。
 *          非线程安全，调用方需保证互斥（或在任务上下文调用）。
 */
void bsw_log(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* BSW_LOG_H */
