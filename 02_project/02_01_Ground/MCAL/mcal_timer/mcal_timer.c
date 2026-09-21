/* mcal_timer.c */
#include "mcal_timer.h"

void mcal_timer_init(void) {}
void mcal_timer_delay_us(uint32_t us) {(void)us;}
void mcal_timer_delay_ms(uint32_t ms) {(void)ms;}
void mcal_timer_start_once(uint8_t id, uint32_t period_us) {(void)id;(void)period_us;}
void mcal_timer_start_periodic(uint8_t id, uint32_t period_us) {(void)id;(void)period_us;}
void mcal_timer_stop(uint8_t id) {(void)id;}
__weak void mcal_timer_callback(uint8_t id) {(void)id;}
