/**
 * @file    app_msg_freq.h
 * @brief   消息频率管理模块 - 频点定义和映射管理
 * 
 * @details
 * 功能：
 * - 定义 19 个通信频点（125Hz ~ 1025Hz，步进 50Hz）
 * - 频率到索引的映射
 * - 频率验证（是否在表内，±15Hz 容差）
 * - 频点查询和转换工具
 * 
 * 架构说明：
 * - 从 app_node_fsm 分离出频点表相关的通用工具
 * - app_scan_listen、app_scan、监听模块都依赖这个模块
 * - 职责单一：管理频点表，不涉及业务逻辑
 * 
 * 模块职责：
 * 1. 维护 19 个频点的静态数据
 * 2. 提供频率 ↔ 索引转换接口
 * 3. 提供频率验证接口
 * 4. 提供频点查询接口
 * 
 * 初始化：
 * - 本模块无初始化函数，可直接调用工具函数
 * 
 * 使用示例：
 * 1. 验证频率是否在表内：app_msg_freq_is_valid(f_hz)
 * 2. 频率转索引：app_msg_freq_to_index(f_hz, &index)
 * 3. 索引转频率：app_msg_freq_from_index(index)
 */

#ifndef APP_MSG_FREQ_H
#define APP_MSG_FREQ_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================ */
/* 公开常量定义                                                                  */
/* ============================================================================ */

/**
 * @brief 通信频点数量
 * @details 19 个频点：125, 175, 225, ..., 1025 Hz
 */
#define APP_MSG_FREQ_COUNT              (19U)

/**
 * @brief 通信频点起始频率（Hz）
 */
#define APP_MSG_FREQ_START              (125U)

/**
 * @brief 通信频点频率步进（Hz）
 */
#define APP_MSG_FREQ_STEP               (50U)

/**
 * @brief 通信频点频率容差（Hz）
 * @details 检测到的频率与标准频点的最大允许偏差
 */
#define APP_MSG_FREQ_TOLERANCE          (15U)

/**
 * @brief 无效索引标记
 */
#define APP_MSG_FREQ_INVALID_INDEX      (0xFFU)

/* ============================================================================ */
/* 公开 API 声明                                                                */
/* ============================================================================ */

/**
 * @brief 获取频点表中的所有频点数组
 * @param[out] p_freq_array 输出频点数组指针
 * @param[out] p_count 输出频点个数
 * @return 0 = 成功，-1 = 参数无效
 */
int32_t app_msg_freq_get_array(const uint16_t **p_freq_array, uint8_t *p_count);

/**
 * @brief 获取指定索引的频点
 * @param index 索引值（0~18）
 * @return 频点值（Hz），若索引无效则返回 0
 */
uint16_t app_msg_freq_from_index(uint8_t index);

/**
 * @brief 频率转索引
 * @details 将检测到的频率转换为表内的索引值
 *          包含容差检查和四舍五入逻辑
 * @param f_hz 输入频率（Hz）
 * @return 索引值（0~18），若频率无效则返回 APP_MSG_FREQ_INVALID_INDEX
 * @rationale:
 *   1. 检查频率是否在容差范围内
 *   2. 计算与起始频点的偏移
 *   3. 根据步进进行四舍五入
 *   4. 验证最终索引是否有效
 */
uint8_t app_msg_freq_to_index(uint16_t f_hz);

/**
 * @brief 校验频点是否在有效频点表内
 * @param f_hz 待校验的频点（Hz）
 * @return true = 在表内且容差范围内，false = 不在表内
 * @details 内部调用 app_msg_freq_to_index() 进行验证
 */
bool app_msg_freq_is_valid(uint16_t f_hz);

#ifdef __cplusplus
}
#endif

#endif /* APP_MSG_FREQ_H */
