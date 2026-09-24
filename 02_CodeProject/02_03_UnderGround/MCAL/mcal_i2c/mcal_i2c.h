/**
 * @file    mcal_i2c.h
 * @brief   I2C 驱动 - MCAL 层
 * @note    用于 BMP280 温压传感器
 */

#ifndef MCAL_I2C_H
#define MCAL_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 宏定义 ========== */
#define I2C_ID_1       1   /* BMP280 使用的 I2C 号 */

/* ========== 函数声明 ========== */

/**
 * @brief   I2C 初始化
 */
void mcal_i2c_init(uint8_t id);

/**
 * @brief   I2C 写寄存器
 * @param   id      I2C 号
 * @param   dev_addr 设备地址（7位）
 * @param   reg     寄存器地址
 * @param   data    要写入的数据
 * @retval  0=成功 其他=失败
 */
int mcal_i2c_write_reg(uint8_t id, uint8_t dev_addr, uint8_t reg, uint8_t data);

/**
 * @brief   I2C 读寄存器
 * @param   id      I2C 号
 * @param   dev_addr 设备地址（7位）
 * @param   reg     寄存器地址
 * @retval  读取到的数据
 */
uint8_t mcal_i2c_read_reg(uint8_t id, uint8_t dev_addr, uint8_t reg);

/**
 * @brief   I2C 读多个字节
 * @param   id      I2C 号
 * @param   dev_addr 设备地址
 * @param   reg     起始寄存器地址
 * @param   buf     接收缓冲区
 * @param   len     长度
 */
void mcal_i2c_read_buf(uint8_t id, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_I2C_H */
