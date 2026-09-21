/**
 * @file    ProtocolCRC.h
 * @brief   CRC-16 CCITT-FALSE - 与 STM32 端 bsw_crc 完全一致
 */

#ifndef PROTOCOL_CRC_H
#define PROTOCOL_CRC_H

#include <QtGlobal>

class ProtocolCRC
{
public:
    static quint16 calc16(const quint8 *data, quint16 len);
};

#endif /* PROTOCOL_CRC_H */
