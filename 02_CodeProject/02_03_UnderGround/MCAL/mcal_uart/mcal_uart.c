/**
 * @file    mcal_uart.c
 * @brief   UART 驱动 - MCAL 层实现
 * @note    用于调试打印日志，阻塞发送
 */

#include "mcal_uart.h"
#include "usart.h"       /* CubeMX 生成的 huart1 */
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

/* ========== 内部宏 ========== */
#define MCAL_UART_MAX_CH  3   /* 支持最多 3 个串口通道 */

/* ========== 内部类型 ========== */
typedef struct {
    UART_HandleTypeDef *huart;   /* 指向 CubeMX 生成的 huart 句柄 */
    uint8_t             inited;  /* 是否已初始化 */
} mcal_uart_ch_t;

/* ========== 内部变量 ========== */
/* 串口通道映射表 */
static mcal_uart_ch_t s_uart_ch[MCAL_UART_MAX_CH] = {0};

/* ========== 内部函数声明 ========== */
static void _putc(uint8_t ch, uint8_t data);

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */

/**
 * @brief   UART 初始化 - 建立 ID 到硬件句柄的映射
 * @param   id    UART 通道号 (0, 1, 2)
 * @param   baud  波特率（校验用，实际参数由 CubeMX 配置）
 */
void mcal_uart_init(uint8_t id, uint32_t baud)
{
    (void)baud;  /* 波特率由 CubeMX 配置，此参数仅作预留 */

    if (id >= MCAL_UART_MAX_CH) {
        return;
    }

    switch (id) {
        case 0:
            s_uart_ch[id].huart = &huart1;
            break;
        /* 如果有第二个串口，在这里添加 case 1: ... */
        /* 如果有第三个串口，在这里添加 case 2: ... */
        default:
            return;
    }

    s_uart_ch[id].inited = 1;
}

/**
 * @brief   UART 发送一个字节
 * @param   id    UART 通道号
 * @param   data  数据
 */
void mcal_uart_putc(uint8_t id, uint8_t data)
{
    _putc(id, data);
}

/**
 * @brief   UART 发送字符串
 * @param   id    UART 通道号
 * @param   str   字符串（以 '\0' 结尾）
 */
void mcal_uart_puts(uint8_t id, const char *str)
{
    if (id >= MCAL_UART_MAX_CH || !s_uart_ch[id].inited || str == NULL) {
        return;
    }

    HAL_UART_Transmit(s_uart_ch[id].huart, (uint8_t *)str,
                      (uint16_t)strlen(str), HAL_MAX_DELAY);
}

/**
 * @brief   UART printf（支持 %d %u %x %c %s，不支持 %f）
 * @param   id    UART 通道号
 * @param   fmt   格式化字符串
 * @param   ...   可变参数
 * @note    不支持浮点打印（%f），如需浮点请在 Keil 中开启 MicroLIB
 */
void mcal_uart_printf(uint8_t id, const char *fmt, ...)
{
    char   buf[128];
    va_list args;

    if (id >= MCAL_UART_MAX_CH || !s_uart_ch[id].inited || fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(s_uart_ch[id].huart, (uint8_t *)buf,
                      (uint16_t)strlen(buf), HAL_MAX_DELAY);
}

/* ================================================================ */
/*                           内部函数实现                             */
/* ================================================================ */

/**
 * @brief   内部：发送单个字符
 */
static void _putc(uint8_t id, uint8_t data)
{
    if (id >= MCAL_UART_MAX_CH || !s_uart_ch[id].inited) {
        return;
    }

    HAL_UART_Transmit(s_uart_ch[id].huart, &data, 1, HAL_MAX_DELAY);
}
