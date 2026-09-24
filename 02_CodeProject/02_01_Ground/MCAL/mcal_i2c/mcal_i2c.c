/* mcal_i2c.c */
#include "mcal_i2c.h"

void mcal_i2c_init(uint8_t id) {(void)id;}
int mcal_i2c_write_reg(uint8_t id, uint8_t dev_addr, uint8_t reg, uint8_t data) {
    (void)id;(void)dev_addr;(void)reg;(void)data; return 0;
}
uint8_t mcal_i2c_read_reg(uint8_t id, uint8_t dev_addr, uint8_t reg) {
    (void)id;(void)dev_addr;(void)reg; return 0;
}
void mcal_i2c_read_buf(uint8_t id, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    (void)id;(void)dev_addr;(void)reg;(void)buf;(void)len;
}
