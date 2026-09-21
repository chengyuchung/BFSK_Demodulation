/**
 * @file    bsw_crc.h
 * @brief   CRC 校验 - BSW 层
 */

#ifndef BSW_CRC_H
#define BSW_CRC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 函数声明 ========== */

/**
 * @brief   CRC-16 CCITT-FALSE 计算
 * @param   data  数据指针
 * @param   len   数据长度
 * @retval  CRC-16 结果
 * @note    多项式 0x1021，初值 0xFFFF
 */
uint16_t bsw_crc16(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BSW_CRC_H */
