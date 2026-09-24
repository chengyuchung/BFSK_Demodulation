/* ProtocolCRC.cpp */
#include "ProtocolCRC.h"

static quint16 crc_table[256];

static void make_crc_table(void)
{
    const quint16 poly = 0x1021;
    const quint16 init = 0xFFFF;
    for (int i = 0; i < 256; i++) {
        quint16 crc = i << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ poly;
            else              crc <<= 1;
        }
        crc_table[i] = crc ^ init;
    }
}

static int init_done = 0;

quint16 ProtocolCRC::calc16(const quint8 *data, quint16 len)
{
    if (!init_done) { make_crc_table(); init_done = 1; }
    quint16 crc = 0xFFFF;
    for (quint16 i = 0; i < len; i++) {
        int idx = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = (crc << 8) ^ crc_table[idx];
    }
    return crc;
}
