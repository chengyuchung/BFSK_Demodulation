/* mcal_spi.c */
#include "mcal_spi.h"

void mcal_spi_init(uint8_t id) {(void)id;}
uint8_t mcal_spi_transfer(uint8_t id, uint8_t tx_data) {(void)id;(void)tx_data; return 0;}
void mcal_spi_transfer_buf(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len) {
    (void)id;(void)tx_buf;(void)rx_buf;(void)len;
}
