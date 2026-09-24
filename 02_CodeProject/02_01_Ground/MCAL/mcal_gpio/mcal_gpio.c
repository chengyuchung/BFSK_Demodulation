/* mcal_gpio.c */
#include "mcal_gpio.h"

void mcal_gpio_init(void) {}
void mcal_gpio_write(uint8_t pin, uint8_t level) {(void)pin;(void)level;}
uint8_t mcal_gpio_read(uint8_t pin) {(void)pin; return 0;}
void mcal_gpio_set_output(uint8_t pin) {(void)pin;}
void mcal_gpio_set_input(uint8_t pin) {(void)pin;}
