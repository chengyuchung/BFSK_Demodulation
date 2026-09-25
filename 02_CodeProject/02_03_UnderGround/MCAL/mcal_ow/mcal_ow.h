/**
 * @file    mcal_ow.h
 * @brief   1-Wire (单总线) 时序驱动 - MCAL 层
 * @note    协议无关的位/字节读写原语。任何 1-Wire 设备
 *          (DS18B20 / DS2413 / DS2438 等) 都可基于此实现。
 *
 * @dependency  mcal_gpio (DQ 数据线读写，开漏)
 * @dependency  mcal_timer (微秒级延时)
 *
 * 当前项目固定使用 GPIO_PIN_DS18B20 作为 DQ 线；
 * 多总线场景下需将引脚参数化。
 */

#ifndef MCAL_OW_H
#define MCAL_OW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "main.h"

/* ========== 类型定义 ========== */
typedef enum {
    OW_OK          =  0,
    OW_ERR_NO_DEVICE = -1,   /* 总线无设备响应（无 presence pulse） */
} ow_ret_t;

/* ========== 函数声明 ========== */

/**
 * @brief   1-Wire 总线复位 + presence 检测
 * @retval  OW_OK (0)            有设备响应
 *          OW_ERR_NO_DEVICE (-1) 无设备响应
 */
ow_ret_t mcal_ow_reset(void);

/**
 * @brief   1-Wire 写一个比特
 * @param   bit  0 或 1
 */
void mcal_ow_write_bit(uint8_t bit);

/**
 * @brief   1-Wire 读一个比特
 * @retval  读到的位（0 或 1）
 */
uint8_t mcal_ow_read_bit(void);

/**
 * @brief   1-Wire 写一个字节（LSB 先发）
 * @param   data  要写入的字节
 */
void mcal_ow_write_byte(uint8_t data);

/**
 * @brief   1-Wire 读一个字节（LSB 先收）
 * @retval  读到的字节
 */
uint8_t mcal_ow_read_byte(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_OW_H */
