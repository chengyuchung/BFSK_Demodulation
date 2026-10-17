/**
 * @file    app_scan.c
 * @brief   主动扫频模块实现
 * 
 * @details
 * 负责向下级节点发送 19 个扫频信号，并监听下级的 1010 回应波形，
 * 最后进行 ACK 握手。
 */

#include "app_scan.h"
#include "app_msg_freq.h"
#include "bsw_adc_ringbuf.h"
#include "mcal_timer.h"
#include "bsw_ad9833.h"
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
 * @brief 每个频点的发送时长（ms）
 */
#define SCAN_FREQ_DURATION_MS           (50U)

/**
 * @brief 扫频会话超时时间（ms）
 * @details 从发送第一个频点开始，850ms 后停止发送并进入等待回应
 */
#define SWEEP_SESSION_TIMEOUT_MS        (850U)

/**
 * @brief 未启动标记
 */
#define SWEEP_NOT_STARTED               (0U)

/**
 * @brief 等待 1010 回应的超时时间（ms）
 */
#define SCAN_WAIT_REPLY_TIMEOUT_MS      (2000U)

/**
 * @brief ACK 确认超时时间（ms）
 * @details 发送 ACK 后，800ms 内无重复 1010 表示下级已收到
 */
#define SCAN_ACK_CONFIRM_TIMEOUT_MS     (800U)

/**
 * @brief 未启动标记
 */
#define SWEEP_NOT_STARTED               (0U)

/**
 * @brief 1010 模式的最小交替次数
 */
#define REPLY_1010_MIN_TOGGLE_COUNT     (6U)

/**
 * @brief 频率容差（Hz）
 */
#define REPLY_FREQ_TOLERANCE_HZ         (30U)

/**
 * @brief 两个频率之间的最小差异（Hz）
 */
#define REPLY_FREQ_DIFF_MIN_HZ          (100U)

/**
 * @brief 频对最小间隔（Hz）
 */
#define SWEEP_MIN_FREQ_GAP_HZ           (150U)

/**
 * @brief 幅度有效门限
 */
#define SWEEP_AMP_VALID_THRESHOLD       (100U)

/**
 * @brief 回复最小幅度边际
 */
#define REPLY_MIN_AMPLITUDE_MARGIN      (0U)

/**
 * @brief 频率未锁定标记
 */
#define FREQ_NOT_LOCKED                 (0U)

/**
 * @brief 标志被设置
 */
#define FLAG_SET                        (1U)

/**
 * @brief 标志被清除
 */
#define FLAG_CLEARED                    (0U)

/**
 * @brief 未启动标记
 */
#define QUIET_NOT_STARTED               (0U)

/**
 * @brief 频率未锁定标记
 */
#define FREQ_NOT_LOCKED                 (0U)

/**
 * @brief SCAN_WAIT_REPLY 阶段：等待 1010 波形
 */
#define SCAN_WAIT_1010                  (0U)

/* ============================================================================ */
/* 内部数据结构                                                                  */
/* ============================================================================ */

/**
 * @brief 主动扫频上下文
 * @rationale: 状态需要跨函数调用持久化，必须是静态变量
 * @thread-safe: 本模块不支持多线程或 ISR 访问；调用方需外部同步
 */
static struct
{
    uint32_t sweep_start_tick;              /* 扫频会话开始时刻（ms） */
    uint32_t sweep_timeout_tick;            /* 扫频会话超时时刻（ms） */
    uint8_t  scan_freq_index;               /* 当前发送的频点索引（0~18） */
    uint32_t scan_freq_next_tick;           /* 下一次频点切换时刻（ms） */
    
    uint32_t scan_wait_reply_start_tick;    /* 等待回应的开始时刻（ms） */
    uint8_t  scan_wait_reply_phase;         /* PHASE 阶段标记 */
    uint32_t scan_ack_sent_tick;            /* ACK 发送时刻（ms） */
    uint8_t  scan_ack_retry_count;          /* ACK 重发次数 */
    uint32_t scan_1010_detected_tick;       /* 1010 检测时刻（ms） */
    
    uint16_t reply_detected_f0;             /* 检测到的低频点（Hz） */
    uint16_t reply_detected_f1;             /* 检测到的高频点（Hz） */
    uint16_t reply_last_freq;               /* 上一次检测的频率（Hz） */
    uint8_t  reply_toggle_count;            /* 1010 交替切换计数 */
    
    uint16_t downlink_f0_hz;                /* 最终的下行频点对 f0 */
    uint16_t downlink_f1_hz;                /* 最终的下行频点对 f1 */
    
    uint8_t  is_active;                     /* 是否正在扫频 */
    uint8_t  is_initialized;                /* 初始化标志 */
} s_ctx = {0};

/* ============================================================================ */
/* 公开 API 实现                                                                 */
/* ============================================================================ */

void app_scan_init(void)
{
    /* 清零所有字段 */
    memset(&s_ctx, 0, sizeof(s_ctx));
    
    /* 初始化完成标记 */
    s_ctx.is_initialized = 1;
}

void app_scan_start(uint32_t now_tick)
{
    /* 硬件配置：禁用 ADC（发送模式不需要接收） */
    bsw_adc_ringbuf_disable();
    
    /* 硬件配置：停止定时器输入捕获（发送模式不需要解调） */
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    
    /* 启动扫频会话计时器（首次扫频时初始化） */
    if (s_ctx.sweep_start_tick == SWEEP_NOT_STARTED)
    {
        s_ctx.sweep_start_tick   = now_tick;
        s_ctx.sweep_timeout_tick = now_tick + SWEEP_SESSION_TIMEOUT_MS;
    }
    
    /* 初始化扫频参数 */
    s_ctx.scan_freq_index     = 0;
    s_ctx.scan_freq_next_tick = now_tick + SCAN_FREQ_DURATION_MS;
    
    /* 硬件配置：唤醒 AD9833 */
    (void)bsw_ad9833_sleep(false);
    
    /* 发送第一个频点 */
    uint16_t first_freq = APP_MSG_FREQ_START;
    (void)bsw_ad9833_set_freq(BSW_AD9833_REG_0, first_freq);
    (void)bsw_ad9833_select(BSW_AD9833_REG_0);
    
    /* 标记为活动状态 */
    s_ctx.is_active = 1;
    
    /* 用户反馈：LED 显示扫频状态 */
    bsw_led_set_state(LED_STATE_SCAN);
}

void app_scan_stop(void)
{
    /* 硬件配置：停止 AD9833（进入睡眠） */
    (void)bsw_ad9833_sleep(true);
    
    /* 标记为非活动状态 */
    s_ctx.is_active = 0;
}

void app_scan_enter_wait_reply(uint32_t now_tick)
{
    /* 硬件配置：停止 AD9833 发送（进入休眠） */
    (void)bsw_ad9833_sleep(true);
    
    /* 硬件配置：启用 ADC 环形缓冲（接收模式） */
    bsw_adc_ringbuf_enable();
    
    /* 初始化回应检测参数 */
    s_ctx.scan_wait_reply_start_tick = now_tick;
    s_ctx.scan_wait_reply_phase      = SCAN_WAIT_1010;
    s_ctx.scan_ack_sent_tick         = QUIET_NOT_STARTED;
    s_ctx.scan_ack_retry_count       = 0;
    s_ctx.scan_1010_detected_tick    = QUIET_NOT_STARTED;
    s_ctx.reply_last_freq            = FREQ_NOT_LOCKED;
    s_ctx.reply_detected_f0          = FREQ_NOT_LOCKED;
    s_ctx.reply_detected_f1          = FREQ_NOT_LOCKED;
    s_ctx.reply_toggle_count         = 0;
}
