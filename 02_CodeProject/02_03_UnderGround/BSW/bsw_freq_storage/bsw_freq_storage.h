/**
 * @file    bsw_freq_storage.h
 * @brief   频点配置 Flash 存储模块
 *
 * 功能：
 * - 将锁定的上行 + 下行频对保存到 Flash，用于掉电恢复
 * - 使用 STM32L4 的最后一页 Flash 作为存储区域
 * - 提供简单的魔术字校验，防止读取未初始化数据
 *
 * 存储格式（16 字节，8 字节对齐）：
 *   [0-3]   : 魔术字 0x46525151 ("FRQQ")
 *   [4-5]   : 上行 f0_hz (uint16_t, 小端序)
 *   [6-7]   : 上行 f1_hz (uint16_t, 小端序)
 *   [8-9]   : 下行 f0_hz (uint16_t, 小端序)
 *   [10-11] : 下行 f1_hz (uint16_t, 小端序)
 *   [12-15]: 保留（对齐）
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
 * @brief 保存上行 + 下行频对到 Flash
 *
 * @param uplink_f0_hz    上行频点 0 (Hz)
 * @param uplink_f1_hz    上行频点 1 (Hz)
 * @param downlink_f0_hz  下行频点 0 (Hz)
 * @param downlink_f1_hz  下行频点 1 (Hz)
 * @return 0 成功，-1 失败
 *
 * 注意：
 * - Flash 擦除/写入较慢（~20ms），不要在时间敏感路径调用
 * - 失败不影响系统运行，只是下次掉电无法快速恢复
 */
int32_t bsw_freq_storage_save_all(uint16_t uplink_f0_hz,
                                  uint16_t uplink_f1_hz,
                                  uint16_t downlink_f0_hz,
                                  uint16_t downlink_f1_hz);

/**
 * @brief 从 Flash 加载上行 + 下行频对
 *
 * @param[out] uplink_f0_hz    上行频点 0 (Hz)（Flash 无效时输出 0）
 * @param[out] uplink_f1_hz    上行频点 1 (Hz)（Flash 无效时输出 0）
 * @param[out] downlink_f0_hz  下行频点 0 (Hz)（Flash 无效时输出 0）
 * @param[out] downlink_f1_hz  下行频点 1 (Hz)（Flash 无效时输出 0）
 * @return 0 成功（全部有效），-1 失败（Flash 未初始化或数据损坏）
 *
 * @note   失败时所有输出参数均被赋值为 0，便于调用方统一判定"频点无效"
 *         而无需额外维护标志位
 */
int32_t bsw_freq_storage_load_all(uint16_t *uplink_f0_hz,
                                  uint16_t *uplink_f1_hz,
                                  uint16_t *downlink_f0_hz,
                                  uint16_t *downlink_f1_hz);

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
