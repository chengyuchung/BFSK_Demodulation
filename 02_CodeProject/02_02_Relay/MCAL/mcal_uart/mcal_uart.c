/* mcal_uart.c */
#include "mcal_uart.h"
#include <stdio.h>
#include <string.h>

void mcal_uart_init(uint8_t id, uint32_t baud) {(void)id;(void)baud;}
void mcal_uart_putc(uint8_t id, uint8_t data) {(void)id;(void)data;}
void mcal_uart_puts(uint8_t id, const char *str) {(void)id;(void)str;}

void mcal_uart_printf(uint8_t id, const char *fmt, ...)
{
    (void)id;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    mcal_uart_puts(id, buf);
}
