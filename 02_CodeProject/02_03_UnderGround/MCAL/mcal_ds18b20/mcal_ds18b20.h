/**
 * @file    mcal_ds18b20.h
 * @brief   DS18B20 单总线温度传感器驱动 - MCAL 层
 * @note    封装 1-Wire 协议，依赖 mcal_gpio + mcal_timer
 *
 * @dependency  mcal_gpio (数据线读写)
 * @dependency  mcal_timer (微秒延时)
 */

#ifndef MCAL_DS18B20_H
#define MCAL_DS18B20_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 类型定义 ========== */
typedef enum {
    DS18B20_OK = 0,
    DS18B20_ERR_NO_DEVICE = -1,  /* 总线无设备响应 */
    DS18B20_ERR_CRC = -2,        /* CRC8 校验失败（数据损坏/总线干扰） */
    DS18B20_ERR_TIMEOUT = -3,    /* 转换超时 */
    DS18B20_ERR_READ = -4        /* 读取异常（全 0xFF/0x00 等） */
} ds18b20_ret_t;

/* ========== 函数声明 ========== */

/**
 * @brief   DS18B20 初始化（总线复位检测）
 * @retval  DS18B20_OK / DS18B20_ERR_NO_DEVICE
 */
ds18b20_ret_t mcal_ds18b20_init(void);

/**
 * @brief   读取温度
 * @param   temp_out  输出温度值（扩大100倍，如 25.5°C 输出 2550）
 * @retval  DS18B20_OK / 错误码
 * @note    会发起一次温度转换（最多 750ms）并自动等待 + 读取
 *          包含 CRC8 校验
 */
ds18b20_ret_t mcal_ds18b20_read_temp(int16_t *temp_out);

/**
 * @brief   启动一次温度转换（不等待结果）
 * @retval  DS18B20_OK / 错误码
 */
ds18b20_ret_t mcal_ds18b20_start_convert(void);

/**
 * @brief   等待温度转换完成（轮询方式）
 * @param   timeout_ms  超时时间（建议 800ms，对应 12bit 分辨率）
 * @retval  DS18B20_OK / DS18B20_ERR_TIMEOUT
 * @note    DS18B20 转换期间 DQ 拉低，完成后释放变高
 *          通过读时隙轮询该状态
 */
ds18b20_ret_t mcal_ds18b20_wait_conversion(uint32_t timeout_ms);

/**
 * @brief   读取上一次转换结果（需在 start_convert + wait_conversion 后调用）
 * @param   temp_out  输出温度值（扩大100倍）
 * @retval  DS18B20_OK / DS18B20_ERR_CRC / 其他错误
 * @note    包含 9 字节 scratchpad 读取 + CRC8 校验
 */
ds18b20_ret_t mcal_ds18b20_read_result(int16_t *temp_out);

/* ========== 内部 1-Wire 操作（供 DS18B20 协议内部调用） ========== */

/**
 * @brief   1-Wire 复位脉冲
 * @retval  0=有设备响应 1=无设备
 */
int mcal_ds18b20_onewire_reset(void);

/**
 * @brief   1-Wire 写一个比特
 * @param   bit  0 或 1
 */
void mcal_ds18b20_onewire_write_bit(uint8_t bit);

/**
 * @brief   1-Wire 读一个比特
 * @retval  读到的值
 */
uint8_t mcal_ds18b20_onewire_read_bit(void);

/**
 * @brief   1-Wire 写一个字节
 * @param   data  要写入的字节
 */
void mcal_ds18b20_onewire_write_byte(uint8_t data);

/**
 * @brief   1-Wire 读一个字节
 * @retval  读到的字节
 */
uint8_t mcal_ds18b20_onewire_read_byte(void);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_DS18B20_H */
