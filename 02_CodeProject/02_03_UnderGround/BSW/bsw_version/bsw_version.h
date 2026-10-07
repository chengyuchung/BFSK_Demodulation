/**
 * @file    bsw_version.h
 * @brief   版本管理模块 - BSW 层
 *
 * 功能：
 *   - 存储和管理软件版本号（日期格式 YYYYMMDD）
 *   - 存储和管理硬件版本号（日期格式 YYYYMMDD）
 *   - 提供版本信息查询接口（供井上通过协议查询）
 *   - 可选：编译时间戳、Git commit ID 等扩展信息
 *
 * 版本号格式：
 *   - 软件版本：YYYYMMDD（例如 20261007 表示 2026年10月7日）
 *   - 硬件版本：YYYYMMDD（例如 20260801 表示 2026年8月1日）
 *   - 存储为 uint32_t，与协议帧 proto_version_payload_t 对齐
 *
 * 使用场景：
 *   - 井上设备通过 PROTO_QUERY_VERSION 查询井下节点版本号
 *   - 井下节点通过 PROTO_REPLY_VERSION 回复版本信息
 *   - 现场调试时快速确认固件版本
 *   - OTA 升级前的版本校验
 *
 * 协议集成：
 *   井上查询：PROTO_TYPE_QUERY + PROTO_QUERY_VERSION（无载荷）
 *   井下回复：PROTO_TYPE_REPLY + PROTO_REPLY_VERSION
 *            载荷格式：[sw_version_date(4)][hw_version_date(4)]
 *
 * @usage
 *   bsw_version_init();
 *   const bsw_version_info_t *info = bsw_version_get_info();
 *   printf("SW Ver: %lu (date)\n", info->sw_version_date);
 */

#ifndef BSW_VERSION_H
#define BSW_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ========== 版本号定义（日期格式 YYYYMMDD）========== */

/**
 * @brief   版本信息结构体（日期格式）
 * 
 * 版本号采用日期格式 YYYYMMDD，存储为 uint32_t
 *   - 例如：20261007 表示 2026年10月7日编译的版本
 *   - 方便现场快速识别版本新旧
 *   - 与协议帧 proto_version_payload_t 直接对齐
 * 
 * @example
 *   软件版本：20261007（2026年10月7日）
 *   硬件版本：20260801（2026年8月1日）
 */
typedef struct {
    uint32_t sw_version_date;   /**< 软件版本号（日期格式 YYYYMMDD）*/
    uint32_t hw_version_date;   /**< 硬件版本号（日期格式 YYYYMMDD）*/
    char     build_date[12];    /**< 编译日期字符串 "YYYY-MM-DD"（可选，用于日志）*/
    char     build_time[9];     /**< 编译时间字符串 "HH:MM:SS"（可选，用于日志）*/
} bsw_version_info_t;

/* ========== 函数声明（待实现）========== */

/**
 * @brief   初始化版本管理模块
 * @note    填充版本信息结构体，记录编译时间戳
 */
void bsw_version_init(void);

/**
 * @brief   获取版本信息
 * @return  版本信息结构体指针（只读）
 */
const bsw_version_info_t *bsw_version_get_info(void);

/**
 * @brief   获取软件版本号（日期格式）
 * @return  软件版本号 YYYYMMDD（例如 20261007）
 */
uint32_t bsw_version_get_sw_date(void);

/**
 * @brief   获取硬件版本号（日期格式）
 * @return  硬件版本号 YYYYMMDD（例如 20260801）
 */
uint32_t bsw_version_get_hw_date(void);

/**
 * @brief   版本比较（用于 OTA 升级判断）
 * @param   other_sw_date  对比的软件版本号（日期格式 YYYYMMDD）
 * @return  > 0 当前版本更新，= 0 版本相同，< 0 当前版本更旧
 */
int bsw_version_compare_sw(uint32_t other_sw_date);

#ifdef __cplusplus
}
#endif

#endif /* BSW_VERSION_H */
