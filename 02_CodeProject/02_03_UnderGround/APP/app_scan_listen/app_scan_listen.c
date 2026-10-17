/**
 * @file    app_scan_listen.c
 * @brief   扫频监听模块实现
 * 
 * @details
 * 负责被动监听上级节点的扫频信号，并从 19 个频点中决选出最佳频对。
 */

#include "app_scan_listen.h"
#include "app_msg_freq.h"
#include "bsw_adc_ringbuf.h"
#include "mcal_timer.h"
#include "bsw_led.h"
#include <string.h>

/* ============================================================================ */
/* 宏定义                                                                       */
/* ============================================================================ */

/**
 * @brief 扫频频点数量
 * @details 19 个频点：125Hz ~ 1025Hz，步进 50Hz
 */
#define SWEEP_FREQ_COUNT                (19U)

/**
 * @brief 扫频起始频率（Hz）
 */
#define SWEEP_FREQ_START                (125U)

/**
 * @brief 扫频频率步进（Hz）
 */
#define SWEEP_FREQ_STEP                 (50U)

/**
 * @brief 扫频频率容差（Hz）
 * @details 检测到的频率与标准频点的最大允许偏差
 */
#define SWEEP_FREQ_TOLERANCE_HZ         (15U)

/**
 * @brief 扫频会话超时时间（ms）
 * @details 从首次检测到信号开始，850ms 后触发决选
 */
#define SWEEP_SESSION_TIMEOUT_MS        (850U)

/**
 * @brief 幅度有效门限
 * @details 低于此值的信号视为无效噪声
 */
#define SWEEP_AMP_VALID_THRESHOLD       (100U)

/**
 * @brief 频对最小间隔（Hz）
 * @details 两个频点之间至少相距 150Hz 才能有效区分
 */
#define SWEEP_MIN_FREQ_GAP_HZ           (150U)

/**
 * @brief 未启动标记
 */
#define SWEEP_NOT_STARTED               (0U)

/**
 * @brief 无效索引标记
 */
#define SWEEP_FREQ_INDEX_INVALID        (0xFFU)

/**
 * @brief 强度门限百分比（%）
 * @details 选择幅值≥max_amp的70%的频点作为强频点
 */
#define SWEEP_AMP_STRENGTH_THRESHOLD_PERCENT (70U)

/**
 * @brief 最少有效频点数
 * @details 必须至少检测到这么多个强频点才能决选
 */
#define SWEEP_MIN_VALID_FREQ_CNT        (10U)

/* ============================================================================ */
/* 内部数据结构                                                                  */
/* ============================================================================ */

/**
 * @brief 扫频监听上下文
 * @rationale: 状态需要跨函数调用持久化，必须是静态变量
 * @thread-safe: 本模块不支持多线程或 ISR 访问；调用方需外部同步
 */
static struct
{
    uint16_t amp_table[SWEEP_FREQ_COUNT];   /* 19 个频点的幅度表 */
    uint32_t start_tick;                     /* 扫频开始时刻（ms） */
    uint32_t timeout_tick;                   /* 扫频超时时刻（ms） */
    uint8_t  is_active;                      /* 是否正在监听 */
    uint8_t  is_initialized;                 /* 初始化标志 */
} s_ctx = {0};

/* ============================================================================ */
/* 公开 API 实现                                                                 */
/* ============================================================================ */

/* ============================================================================ */
/* 内部函数声明                                                                  */
/* ============================================================================ */

/**
 * @brief 更新幅度表（保留各频点的最大幅值）
 * @param idx 频点索引（0~18）
 * @param amplitude 当前检测到的幅度
 * 
 * @details
 *  策略：只记录最大值，不淘汰
 *  用于扫频过程中累积各频点的信号强度
 */
static void _update_amp_table(uint8_t idx, uint16_t amplitude)
{
    /* 索引范围检查 */
    if (idx >= SWEEP_FREQ_COUNT)
    {
        return;
    }
    
    /* 只更新更大的幅值 */
    if (amplitude > s_ctx.amp_table[idx])
    {
        s_ctx.amp_table[idx] = amplitude;
    }
}

/**
 * @brief 频对决选算法
 * @param[out] p_out_f0 选出的低频点（输出）
 * @param[out] p_out_f1 选出的高频点（输出）
 * @return 0 = 成功选出频对，-1 = 有效频点不足，-2 = 找不到满足间距的频对
 * 
 * @details
 *  算法步骤：
 *  1. 找出幅值最大的频点
 *  2. 计算强度门限 = max_amp * 70%
 *  3. 筛选超过门限的频点
 *  4. 在强频点中找幅值最接近的两个，且满足最小间隔要求（≥150Hz）
 *  
 *  决策依据：
 *  - 幅值相近的频点更可能是同一对信号源
 *  - 最小间隔保证频点可区分
 */
static int32_t _select_freq_pair(uint16_t *p_out_f0, uint16_t *p_out_f1)
{
    /* 参数检查 */
    if (p_out_f0 == NULL || p_out_f1 == NULL)
    {
        return -1;
    }

    /* 步骤 1：找到幅值最大的频点 */
    uint16_t max_amp = 0;
    
    for (uint8_t i = 0; i < APP_MSG_FREQ_COUNT; ++i)
    {
        if (s_ctx.amp_table[i] > max_amp)
        {
            max_amp = s_ctx.amp_table[i];
        }
    }
    
    if (max_amp == 0)
    {
        return -1;  /* 无有效信号 */
    }
    
    /* 步骤 2：计算强度门限 = max_amp * 70% */
    uint16_t strength_threshold = (max_amp * SWEEP_AMP_STRENGTH_THRESHOLD_PERCENT) / 100u;
    
    /* 步骤 3：筛选超过门限的频点 */
    uint8_t strong_idx[APP_MSG_FREQ_COUNT];
    uint8_t strong_cnt = 0;
    
    for (uint8_t i = 0; i < APP_MSG_FREQ_COUNT; ++i)
    {
        if (s_ctx.amp_table[i] >= strength_threshold)
        {
            strong_idx[strong_cnt++] = i;
        }
    }
    
    if (strong_cnt < SWEEP_MIN_VALID_FREQ_CNT)
    {
        return -1;  /* 强度足够的频点不足 */
    }
    
    /* 步骤 4：在强频点中找幅值最接近的两个，且满足最小间隔 */
    uint8_t  best_f0_idx = APP_MSG_FREQ_INVALID_INDEX;
    uint8_t  best_f1_idx = APP_MSG_FREQ_INVALID_INDEX;
    uint16_t best_amp_diff = 0xFFFFU;  /* 幅值差异最小为目标 */
    
    for (uint8_t i = 0; i < strong_cnt; ++i)
    {
        for (uint8_t j = i + 1; j < strong_cnt; ++j)
        {
            uint8_t  idx_a = strong_idx[i];
            uint8_t  idx_b = strong_idx[j];
            uint16_t amp_a = s_ctx.amp_table[idx_a];
            uint16_t amp_b = s_ctx.amp_table[idx_b];
            
            /* 检查频率间隔是否满足最小要求 */
            uint8_t  gap_idx = (idx_b > idx_a) ? (idx_b - idx_a) : (idx_a - idx_b);
            uint16_t gap_hz  = (uint16_t)gap_idx * APP_MSG_FREQ_STEP;
            
            if (gap_hz < SWEEP_MIN_FREQ_GAP_HZ)
            {
                continue;  /* 间隔不足，跳过 */
            }
            
            /* 计算幅值差异 */
            uint16_t amp_diff = (amp_a > amp_b) ? (amp_a - amp_b) : (amp_b - amp_a);
            
            /* 选择差异最小的那一对 */
            if (amp_diff < best_amp_diff)
            {
                best_amp_diff = amp_diff;
                
                /* f0 为幅值较大的，f1 为幅值较小的 */
                if (amp_a >= amp_b)
                {
                    best_f0_idx = idx_a;
                    best_f1_idx = idx_b;
                }
                else
                {
                    best_f0_idx = idx_b;
                    best_f1_idx = idx_a;
                }
            }
        }
    }
    
    if (best_f0_idx == APP_MSG_FREQ_INVALID_INDEX)
    {
        return -2;  /* 找不到满足间隔要求的频点对 */
    }

    /* 输出选中的频点 */
    *p_out_f0 = (uint16_t)APP_MSG_FREQ_START + (uint16_t)best_f0_idx * APP_MSG_FREQ_STEP;
    *p_out_f1 = (uint16_t)APP_MSG_FREQ_START + (uint16_t)best_f1_idx * APP_MSG_FREQ_STEP;
    return 0;
}

/* ============================================================================ */
/* 公开 API 实现                                                                 */
/* ============================================================================ */

void app_scan_listen_init(void)
{
    /* 清零所有字段 */
    memset(&s_ctx, 0, sizeof(s_ctx));
    
    /* 初始化完成标记 */
    s_ctx.is_initialized = 1;
}

void app_scan_listen_start(uint32_t now_tick)
{
    (void)now_tick;  /* 暂时不使用，预留给未来扩展 */
    
    /* 清空幅度表 */
    memset(s_ctx.amp_table, 0, sizeof(s_ctx.amp_table));
    
    /* 重置计时器 */
    s_ctx.start_tick   = 0;
    s_ctx.timeout_tick = 0;
    
    /* 标记为活动状态 */
    s_ctx.is_active = 1;
    
    /* 硬件配置：启用 ADC 接收 */
    bsw_adc_ringbuf_enable();
    
    /* 硬件配置：停止定时器输入捕获（监听模式不需要解调） */
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    
    /* 用户反馈：LED 显示监听状态 */
    bsw_led_set_state(LED_STATE_LISTEN);
}

void app_scan_listen_stop(void)
{
    /* 标记为非活动状态 */
    s_ctx.is_active = 0;
    
    /* 硬件配置：禁用 ADC 环形缓冲 */
    bsw_adc_ringbuf_disable();
}

uint8_t app_scan_listen_get_valid_freq_count(void)
{
    uint8_t cnt = 0;
    
    for (uint8_t i = 0; i < APP_MSG_FREQ_COUNT; ++i)
    {
        if (s_ctx.amp_table[i] > APP_SCAN_LISTEN_AMP_VALID_THRESHOLD)
        {
            ++cnt;
        }
    }
    
    return cnt;
}

void app_scan_listen_update_amp_table(uint8_t idx, uint16_t amplitude)
{
    _update_amp_table(idx, amplitude);
}

int32_t app_scan_listen_select_freq_pair(uint16_t *p_out_f0, uint16_t *p_out_f1)
{
    return _select_freq_pair(p_out_f0, p_out_f1);
}
