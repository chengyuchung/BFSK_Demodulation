/**
 * @file    mcal_uart.c
 * @brief   UART 驱动 - MCAL 层实现
 * @note    原 main.c 的 MX_USART1_UART_Init() 已移植到本文件
 *          - USART1: 调试串口，默认 115200bps，TX:PA9 / RX:PA10
 */

#include "mcal_uart.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

extern void Error_Handler(void);

/* ========== 内部宏 ========== */
#define MCAL_UART_MAX_CH  3

/* ========== 内部类型 ========== */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             inited;
} mcal_uart_ch_t;

/* ========== 内部句柄 ========== */
static UART_HandleTypeDef s_huart1;

/* ========== Getter ========== */
UART_HandleTypeDef *mcal_uart_get_handle1(void) { return &s_huart1; }

/* ========== 通道表 ========== */
static mcal_uart_ch_t s_uart_ch[MCAL_UART_MAX_CH] = {0};

/* ========== HAL UART MSP 重写 ========== */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart == &s_huart1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitStruct.Pin       = GPIO_PIN_9 | GPIO_PIN_10;   /* PA9=TX, PA10=RX */
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* ================================================================ */
/*              原 MX_USART1_UART_Init 内容（移植）                  */
/* ================================================================ */
static void _usart1_init(uint32_t baud)
{
    s_huart1.Instance             = USART1;
    s_huart1.Init.BaudRate        = baud;
    s_huart1.Init.WordLength      = UART_WORDLENGTH_8B;
    s_huart1.Init.StopBits        = UART_STOPBITS_1;
    s_huart1.Init.Parity          = UART_PARITY_NONE;
    s_huart1.Init.Mode            = UART_MODE_TX_RX;
    s_huart1.Init.HwFlowCtl       = UART_HWCONTROL_NONE;
    s_huart1.Init.OverSampling    = UART_OVERSAMPLING_16;
    s_huart1.Init.OneBitSampling  = UART_ONE_BIT_SAMPLE_DISABLE;
    s_huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&s_huart1) != HAL_OK) {
        Error_Handler();
    }
}

/* ================================================================ */
/*                           公共接口实现                              */
/* ================================================================ */
void mcal_uart_init(uint8_t id, uint32_t baud)
{
    if (id >= MCAL_UART_MAX_CH) {
        return;
    }

    switch (id) {
        case UART_ID_DEBUG:
            _usart1_init(baud);
            s_uart_ch[id].huart  = &s_huart1;
            s_uart_ch[id].inited = 1;
            break;
        default:
            return;
    }
}

void mcal_uart_putc(uint8_t id, uint8_t data)
{
    if (id >= MCAL_UART_MAX_CH || !s_uart_ch[id].inited) {
        return;
    }
    HAL_UART_Transmit(s_uart_ch[id].huart, &data, 1, HAL_MAX_DELAY);
}

void mcal_uart_puts(uint8_t id, const char *str)
{
    if (id >= MCAL_UART_MAX_CH || !s_uart_ch[id].inited || str == NULL) {
        return;
    }
    HAL_UART_Transmit(s_uart_ch[id].huart, (uint8_t *)str,
                      (uint16_t)strlen(str), HAL_MAX_DELAY);
}

void mcal_uart_printf(uint8_t id, const char *fmt, ...)
{
    char     buf[128];
    va_list  args;

    if (id >= MCAL_UART_MAX_CH || !s_uart_ch[id].inited || fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    HAL_UART_Transmit(s_uart_ch[id].huart, (uint8_t *)buf,
                      (uint16_t)strlen(buf), HAL_MAX_DELAY);
}
