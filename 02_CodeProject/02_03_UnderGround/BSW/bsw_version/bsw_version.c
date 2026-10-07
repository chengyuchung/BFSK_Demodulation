/**
 * @file    bsw_version.c
 * @brief   版本管理模块实现 - BSW 层
 *
 * 版本号管理：
 *   - 软件版本：日期格式 YYYYMMDD（例如 20261007）
 *   - 硬件版本：日期格式 YYYYMMDD（例如 20260801）
 *   - 编译时间戳：__DATE__ 和 __TIME__ 宏
 *
 * 协议集成：
 *   - 井上通过 PROTO_TYPE_QUERY + PROTO_QUERY_VERSION 查询版本
 *   - 井下返回 PROTO_TYPE_REPLY + PROTO_REPLY_VERSION 携带版本信息
 *   - 载荷格式：[sw_version_date(4)][hw_version_date(4)] 共 8 字节
 *
 * 版本号更新流程：
 *   1. 修改下方的 BSW_VERSION_SW_DATE 和 BSW_VERSION_HW_DATE
 *   2. 编译时 __DATE__ 和 __TIME__ 自动更新
 *   3. 通过协议查询即可获取最新版本号
 */

#include "bsw_version.h"

/* ========== 版本号定义（手动维护）========== */

/**
 * @brief   软件版本号（日期格式 YYYYMMDD）
 * @note    每次发布新版本时手动更新此值
 * @example 20261007 表示 2026年10月7日发布的版本
 */
#define BSW_VERSION_SW_DATE  20261007UL  /* TODO: 发布时更新此值 */

/**
 * @brief   硬件版本号（日期格式 YYYYMMDD）
 * @note    硬件设计定版日期，通常不频繁变更
 * @example 20260801 表示 2026年8月1日定版的硬件
 */
#define BSW_VERSION_HW_DATE  20260801UL  /* TODO: 硬件变更时更新此值 */

/* ========== 静态全局变量 ========== */

/** 版本信息结构体实例 */
static bsw_version_info_t s_version_info;

/* ========== 函数实现（待实现）========== */

/**
 * @brief   初始化版本管理模块
 * @note    填充版本信息，记录编译时间戳
 */
void bsw_version_init(void)
{
    uint32_t i;
    
    /* 填充版本号（日期格式）*/
    s_version_info.sw_version_date = BSW_VERSION_SW_DATE;
    s_version_info.hw_version_date = BSW_VERSION_HW_DATE;
    
    /* 记录编译时间戳（用于日志输出）*/
    /* __DATE__ 格式："Oct  7 2026" 或 "Oct 07 2026" */
    /* __TIME__ 格式："18:21:30" */
    
    /* 复制编译日期 */
    for (i = 0; i < sizeof(s_version_info.build_date) - 1; i++) {
        s_version_info.build_date[i] = __DATE__[i];
        if (__DATE__[i] == '\0') {
            break;
        }
    }
    s_version_info.build_date[sizeof(s_version_info.build_date) - 1] = '\0';
    
    /* 复制编译时间 */
    for (i = 0; i < sizeof(s_version_info.build_time) - 1; i++) {
        s_version_info.build_time[i] = __TIME__[i];
        if (__TIME__[i] == '\0') {
            break;
        }
    }
    s_version_info.build_time[sizeof(s_version_info.build_time) - 1] = '\0';
}

/**
 * @brief   获取版本信息
 * @return  版本信息结构体指针（只读）
 */
const bsw_version_info_t *bsw_version_get_info(void)
{
    return &s_version_info;
}

/**
 * @brief   获取软件版本号（日期格式）
 * @return  软件版本号 YYYYMMDD
 */
uint32_t bsw_version_get_sw_date(void)
{
    return s_version_info.sw_version_date;
}

/**
 * @brief   获取硬件版本号（日期格式）
 * @return  硬件版本号 YYYYMMDD
 */
uint32_t bsw_version_get_hw_date(void)
{
    return s_version_info.hw_version_date;
}

/**
 * @brief   版本比较（用于 OTA 升级判断）
 * @param   other_sw_date  对比的软件版本号（日期格式 YYYYMMDD）
 * @return  > 0 当前版本更新，= 0 版本相同，< 0 当前版本更旧
 */
int bsw_version_compare_sw(uint32_t other_sw_date)
{
    if (s_version_info.sw_version_date > other_sw_date) {
        return 1;   /* 当前版本更新 */
    } else if (s_version_info.sw_version_date < other_sw_date) {
        return -1;  /* 当前版本更旧 */
    } else {
        return 0;   /* 版本相同 */
    }
}
