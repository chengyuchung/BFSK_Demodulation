/**
 * @file    bsw_log.c
 * @brief   日志服务 - BSW 层实现
 *
 * 把格式化 + 字符串处理放在 BSW 层；
 * 最终调 mcal_uart_write() 把字节流交给 MCAL 发送。
 */

#include "bsw_log.h"
#include "mcal_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ========== 内部配置 ========== */
#define BSW_LOG_BUF_SIZE  128

/* 默认输出到调试串口；运行时可通过 bsw_log_set_uart() 切换 */
static uint8_t s_uart_id = UART_ID_DEBUG;

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void bsw_log_set_uart(uint8_t uart_id)
{
    s_uart_id = uart_id;
}

void bsw_log(const char *fmt, ...)
{
    char     buf[BSW_LOG_BUF_SIZE];
    va_list  args;
    int      n;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n < 0) {
        return;  /* 格式化错误 */
    }
    if ((uint32_t)n >= sizeof(buf)) {
        n = (int)(sizeof(buf) - 1U);  /* 截断 */
    }

    mcal_uart_write(s_uart_id, (const uint8_t *)buf, (uint16_t)n);
}
