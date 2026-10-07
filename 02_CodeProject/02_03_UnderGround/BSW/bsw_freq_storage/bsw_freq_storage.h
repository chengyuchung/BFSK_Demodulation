/**
 * @file    bsw_freq_storage.h
 * @brief   频点配置 Flash 存储模块
 * 
 * 功能：
 * - 将锁定的频对 (f0, f1) 保存到 Flash，用于掉电恢复
 * - 使用 STM32L4 的最后一页 Flash 作为存储区域
 * - 提供简单的魔术字校验，防止读取未初始化数据
 * 
 * 存储格式（8 字节）：
 *   [0-3]: 魔术字 0x46525151 ("FRQQ")
 *   [4-5]: f0_hz (uint16_t, 小端序)
 *   [6-7]: f1_hz (uint16_t, 小端序)
 */

#ifndef BSW_FREQ_STORAGE_H
#define BSW_FREQ_STORAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化频点存储模块
 * 
 * 必须在使用 save/load 之前调用一次
 */
void bsw_freq_storage_init(void);

/**
 * @brief 保存频对到 Flash
 * 
 * @param f0_hz  频点 0 (Hz)
 * @param f1_hz  频点 1 (Hz)
 * @return 0 成功，-1 失败
 * 
 * 注意：
 * - Flash 擦除/写入较慢（~20ms），不要在时间敏感路径调用
 * - 失败不影响系统运行，只是下次掉电无法快速恢复
 */
int32_t bsw_freq_storage_save(uint16_t f0_hz, uint16_t f1_hz);

/**
 * @brief 从 Flash 加载频对
 * 
 * @param[out] f0_hz  频点 0 (Hz)
 * @param[out] f1_hz  频点 1 (Hz)
 * @return 0 成功，-1 失败（Flash 未初始化或数据损坏）
 */
int32_t bsw_freq_storage_load(uint16_t *f0_hz, uint16_t *f1_hz);

/**
 * @brief 清除 Flash 中的频点配置
 * 
 * @return 0 成功，-1 失败
 * 
 * 用于测试或重置场景
 */
int32_t bsw_freq_storage_clear(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_FREQ_STORAGE_H */
