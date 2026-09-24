/**
 * @file    mcal_adc.h
 * @brief   ADC 采样驱动 - MCAL 层
 * @note    双 ADC 架构：
 *            - ADC1: 定时器触发 + DMA 循环（高速采样，给 BFSK 用）
 *            - ADC2: 软件触发单次读（慢速监控）
 *
 * @dependency   HAL_ADC, HAL_DMA
 */

#ifndef MCAL_ADC_H
#define MCAL_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========== 类型定义 ========== */

/**
 * @brief ADC 设备编号
 */
typedef enum {
    MCAL_ADC_DEV1 = 0,   /**< ADC1: TIM6 TRGO + DMA 循环（高速） */
    MCAL_ADC_DEV2,       /**< ADC2: 软件触发单次读（慢速监控） */
    MCAL_ADC_DEV_MAX
} mcal_adc_dev_t;

/**
 * @brief ADC 错误码
 */
typedef enum {
    MCAL_ADC_OK = 0,
    MCAL_ADC_ERR_INVALID = -1,    /**< 参数非法 */
    MCAL_ADC_ERR_NOT_INIT = -2,   /**< 设备未初始化 */
    MCAL_ADC_ERR_TIMEOUT = -3,    /**< 转换超时 */
    MCAL_ADC_ERR_DMA = -4,        /**< DMA 启动失败 */
    MCAL_ADC_ERR_BUSY = -5        /**< 设备忙 */
} mcal_adc_ret_t;

/**
 * @brief DMA 完成回调
 * @param buf  当前 DMA 缓冲区
 * @param len  缓冲区长度
 */
typedef void (*mcal_adc_dma_cplt_cb_t)(uint16_t *buf, uint32_t len);

/**
 * @brief DMA 半传输回调
 */
typedef void (*mcal_adc_dma_half_cb_t)(uint16_t *buf, uint32_t len);

/* ========== 函数声明 ========== */

/**
 * @brief   初始化 ADC 设备（绑定 HAL 句柄）
 * @param   dev   设备编号
 * @param   hadc  HAL ADC 句柄指针（CubeMX 生成的 &hadc1 / &hadc2）
 * @retval  MCAL_ADC_OK / 错误码
 * @note    必须先调用 MX_ADCx_Init() 再调用本函数
 */
mcal_adc_ret_t mcal_adc_init(mcal_adc_dev_t dev, ADC_HandleTypeDef *hadc);

/**
 * @brief   单次轮询读取（阻塞，适用于 ADC2）
 * @param   dev    设备编号
 * @param   value  输出 12-bit ADC 值（0~4095）
 * @retval  MCAL_ADC_OK / 错误码
 */
mcal_adc_ret_t mcal_adc_read_poll(mcal_adc_dev_t dev, uint16_t *value);

/**
 * @brief   启动 DMA 连续采样（适用于 ADC1）
 * @param   dev  设备编号
 * @param   buf  数据缓冲区（uint16 数组）
 * @param   len  缓冲区长度
 * @retval  MCAL_ADC_OK / 错误码
 * @note    采样由 TIM6 触发，DMA 循环搬运到 buf
 *          完成后回调通知
 */
mcal_adc_ret_t mcal_adc_start_dma(mcal_adc_dev_t dev, uint16_t *buf, uint32_t len);

/**
 * @brief   停止 DMA 采样
 * @param   dev  设备编号
 */
mcal_adc_ret_t mcal_adc_stop_dma(mcal_adc_dev_t dev);

/**
 * @brief   注册 DMA 传输完成回调
 * @param   dev  设备编号
 * @param   cb   回调函数
 */
mcal_adc_ret_t mcal_adc_register_dma_cplt(mcal_adc_dev_t dev, mcal_adc_dma_cplt_cb_t cb);

/**
 * @brief   注册 DMA 半传输完成回调
 * @param   dev  设备编号
 * @param   cb   回调函数
 */
mcal_adc_ret_t mcal_adc_register_dma_half(mcal_adc_dev_t dev, mcal_adc_dma_half_cb_t cb);

#ifdef __cplusplus
}
#endif

#endif /* MCAL_ADC_H */
