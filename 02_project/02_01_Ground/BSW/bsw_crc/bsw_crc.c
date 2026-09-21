/* bsw_crc.c */
#include "bsw_crc.h"

static uint16_t crc_table[256];

static void make_crc_table(void)
{
    /* 生成 CRC-16 CCITT-FALSE 查表表 */
    const uint16_t poly = 0x1021;
    const uint16_t init = 0xFFFF;
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ poly;
            else             crc <<= 1;
        }
        crc_table[i] = crc ^ init;
    }
}

uint16_t bsw_crc16(const uint8_t *data, uint16_t len)
{
    static int init_done = 0;
    if (!init_done) { make_crc_table(); init_done = 1; }
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = (crc << 8) ^ crc_table[idx];
    }
    return crc;
}
