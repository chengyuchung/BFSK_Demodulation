/**
 * @file    mcal_spi.h
 * @brief   SPI 驱动 - MCAL 层
 * @note    用于 AD9833 DDS 芯片通信
 */

#ifndef MCAL_SPI_H
#define MCAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义 ========== */
#define SPI_ID_1   1   /* AD9833 使用的 SPI 号 */

/* ========== 函数声明 ========== */

/**
 * @brief   SPI 初始化
 */
void mcal_spi_init(uint8_t id);

/**
 * @brief   SPI 发送并接收一个字节
 * @param   id     SPI 号
 * @param   tx_data 发送数据
 * @retval  接收到的数据
 */
uint8_t mcal_spi_transfer(uint8_t id, uint8_t tx_data);

/**
 * @brief   SPI 发送多个字节
 * @param   id     SPI 号
 * @param   tx_buf 发送缓冲区
 * @param   rx_buf 接收缓冲区（可传 NULL）
 * @param   len   字节数
 */
void mcal_spi_transfer_buf(uint8_t id, uint8_t *tx_buf, uint8_t *rx_buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_SPI_H */
