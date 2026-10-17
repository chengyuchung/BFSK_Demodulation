/**
 * @file    app_link.c
 * @brief   应用层链路管理实现
 * 
 * @details
 * 负责存储和查询上下级节点的频点信息。
 * 提供简单的 Get/Set 接口，内部只维护两条链路：上行和下行。
 */

#include "app_link.h"
#include <string.h>

/* ============================================================================ */
/* 宏定义 - 频点状态标记                                                         */
/* ============================================================================ */

/**
 * @brief 频点未锁定标记
 * @details 当 f0_hz 或 f1_hz 为此值时，表示链路未建立
 * @rationale: 用于快速判断链路状态，避免多个标志变量
 */
#define APP_LINK_FREQ_NOT_LOCKED  (0U)

/**
 * @brief 节点地址未设置标记
 * @details 初始化时使用，用于区分"未设置"和"有效地址"
 */
#define APP_LINK_ADDR_NOT_SET     (0xFFU)

/**
 * @brief 总线静默超时时间（ms）
 * @details 如果超过这个时间没有收到任何信号，认为通信链路断裂
 */
#define APP_LINK_BUS_QUIET_TIMEOUT_MS (3000U)

/**
 * @brief 静默未启动标记
 * @details 表示当前没有统计总线静默时间
 */
#define APP_LINK_QUIET_NOT_STARTED (0U)

/* ============================================================================ */
/* 链路信息结构体 - 内部使用                                                    */
/* ============================================================================ */

/**
 * @brief 单条链路的内部存储
 * @details 包含对端地址和频点对
 */
typedef struct
{
    uint8_t peer_addr;  /* 对端节点地址 */
    uint16_t f0_hz;     /* 低频点（Hz）；0 表示未锁定 */
    uint16_t f1_hz;     /* 高频点（Hz）；0 表示未锁定 */
} link_info_t;

/**
 * @brief 全局链路管理上下文
 * @rationale: 状态需要跨函数调用持久化，必须是静态变量
 * @thread-safe: 本模块不支持多线程或 ISR 访问；调用方需外部同步
 */
static struct
{
    link_info_t uplink;        /* 上级链路信息 */
    link_info_t downlink;      /* 下级链路信息 */
    uint32_t quiet_start_tick; /* 总线静默开始时刻（ms）；0 表示未启动 */
    uint8_t is_initialized;    /* 初始化标志 */
} s_link_mgr = {0};

/* ============================================================================ */
/* 公开 API - 初始化                                                            */
/* ============================================================================ */

void app_link_init(void)
{
    /* 为什么：清零所有字段以重置为未建立状态 */
    memset(&s_link_mgr, 0, sizeof(s_link_mgr));
    
    /* 初始化完成标记 */
    s_link_mgr.is_initialized = 1;
}

/* ============================================================================ */
/* 公开 API - 上行链路管理                                                      */
/* ============================================================================ */

app_link_ret_t app_link_set_uplink(uint8_t peer_addr, uint16_t f0_hz, uint16_t f1_hz)
{
    /* 为什么：检查模块是否初始化，防止未初始化时的无效操作 */
    if (!s_link_mgr.is_initialized)
    {
        return APP_LINK_ERR;
    }
    
    /* 保存频点信息 */
    s_link_mgr.uplink.peer_addr = peer_addr;
    s_link_mgr.uplink.f0_hz = f0_hz;
    s_link_mgr.uplink.f1_hz = f1_hz;
    
    return APP_LINK_OK;
}

app_link_ret_t app_link_get_uplink(app_link_freq_pair_t *p_freq, uint8_t *p_addr)
{
    /* 为什么：检查模块是否初始化 */
    if (!s_link_mgr.is_initialized)
    {
        return APP_LINK_ERR;
    }
    
    /* 为什么：检查链路是否有效，避免返回无效频点 */
    if (!app_link_is_uplink_valid())
    {
        return APP_LINK_NOT_FOUND;
    }
    
    /* 输出频点信息（如果指针非 NULL） */
    if (p_freq != NULL)
    {
        p_freq->f0_hz = s_link_mgr.uplink.f0_hz;
        p_freq->f1_hz = s_link_mgr.uplink.f1_hz;
    }
    
    /* 输出节点地址（如果指针非 NULL） */
    if (p_addr != NULL)
    {
        *p_addr = s_link_mgr.uplink.peer_addr;
    }
    
    return APP_LINK_OK;
}

bool app_link_is_uplink_valid(void)
{
    /* 为什么：检查初始化状态 */
    if (!s_link_mgr.is_initialized)
    {
        return false;
    }
    
    /* 为什么：频点为 0 表示未锁定，需要两个频点都有效 */
    return (s_link_mgr.uplink.f0_hz != APP_LINK_FREQ_NOT_LOCKED && 
            s_link_mgr.uplink.f1_hz != APP_LINK_FREQ_NOT_LOCKED);
}

void app_link_clear_uplink(void)
{
    /* 为什么：检查初始化状态后再清除 */
    if (s_link_mgr.is_initialized)
    {
        /* 清零上级链路信息 */
        memset(&s_link_mgr.uplink, 0, sizeof(s_link_mgr.uplink));
    }
}

/* ============================================================================ */
/* 公开 API - 下行链路管理                                                      */
/* ============================================================================ */

app_link_ret_t app_link_set_downlink(uint8_t peer_addr, uint16_t f0_hz, uint16_t f1_hz)
{
    /* 为什么：检查模块是否初始化，防止未初始化时的无效操作 */
    if (!s_link_mgr.is_initialized)
    {
        return APP_LINK_ERR;
    }
    
    /* 保存频点信息 */
    s_link_mgr.downlink.peer_addr = peer_addr;
    s_link_mgr.downlink.f0_hz = f0_hz;
    s_link_mgr.downlink.f1_hz = f1_hz;
    
    return APP_LINK_OK;
}

app_link_ret_t app_link_get_downlink(app_link_freq_pair_t *p_freq, uint8_t *p_addr)
{
    /* 为什么：检查模块是否初始化 */
    if (!s_link_mgr.is_initialized)
    {
        return APP_LINK_ERR;
    }
    
    /* 为什么：检查链路是否有效，避免返回无效频点 */
    if (!app_link_is_downlink_valid())
    {
        return APP_LINK_NOT_FOUND;
    }
    
    /* 输出频点信息（如果指针非 NULL） */
    if (p_freq != NULL)
    {
        p_freq->f0_hz = s_link_mgr.downlink.f0_hz;
        p_freq->f1_hz = s_link_mgr.downlink.f1_hz;
    }
    
    /* 输出节点地址（如果指针非 NULL） */
    if (p_addr != NULL)
    {
        *p_addr = s_link_mgr.downlink.peer_addr;
    }
    
    return APP_LINK_OK;
}

bool app_link_is_downlink_valid(void)
{
    /* 为什么：检查初始化状态 */
    if (!s_link_mgr.is_initialized)
    {
        return false;
    }
    
    /* 为什么：频点为 0 表示未锁定，需要两个频点都有效 */
    return (s_link_mgr.downlink.f0_hz != APP_LINK_FREQ_NOT_LOCKED && 
            s_link_mgr.downlink.f1_hz != APP_LINK_FREQ_NOT_LOCKED);
}

void app_link_clear_downlink(void)
{
    /* 为什么：检查初始化状态后再清除 */
    if (s_link_mgr.is_initialized)
    {
        /* 清零下级链路信息 */
        memset(&s_link_mgr.downlink, 0, sizeof(s_link_mgr.downlink));
    }
}

/* ============================================================================ */
/* 公开 API - 全局链路管理                                                      */
/* ============================================================================ */

void app_link_clear_all(void)
{
    /* 为什么：检查初始化状态后再清除 */
    if (s_link_mgr.is_initialized)
    {
        /* 清零所有链路信息 */
        memset(&s_link_mgr.uplink, 0, sizeof(s_link_mgr.uplink));
        memset(&s_link_mgr.downlink, 0, sizeof(s_link_mgr.downlink));
    }
}

/* ============================================================================ */
/* 公开 API - 总线静默检测                                                      */
/* ============================================================================ */

void app_link_start_bus_quiet_monitor(uint32_t now_tick)
{
    /* 为什么：记录静默开始时刻，用于后续超时检测 */
    if (s_link_mgr.is_initialized)
    {
        s_link_mgr.quiet_start_tick = now_tick;
    }
}

void app_link_stop_bus_quiet_monitor(void)
{
    /* 为什么：收到信号后停止静默监控 */
    if (s_link_mgr.is_initialized)
    {
        s_link_mgr.quiet_start_tick = APP_LINK_QUIET_NOT_STARTED;
    }
}

bool app_link_is_bus_quiet_timeout(uint32_t now_tick)
{
    /* 为什么：检查初始化状态 */
    if (!s_link_mgr.is_initialized)
    {
        return false;
    }
    
    /* 为什么：判断总线是否静默超时 */
    return (s_link_mgr.quiet_start_tick != APP_LINK_QUIET_NOT_STARTED)
           && ((now_tick - s_link_mgr.quiet_start_tick) >= APP_LINK_BUS_QUIET_TIMEOUT_MS);
}

