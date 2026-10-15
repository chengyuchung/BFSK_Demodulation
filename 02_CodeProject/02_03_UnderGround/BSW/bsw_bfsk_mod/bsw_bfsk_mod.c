/**
 * @file    bsw_bfsk_mod.c
 * @brief   BFSK 调制器 - BSW 层实现
 *
 * 调制链路：
 *   1. 字节流 → bit 流（MSB first）
 *   2. bit 流 → 频率序列（bit=0 → f0，bit=1 → f1）
 *   3. 每个 bit 持续 bit_period_ms，通过定时器控制
 *   4. 调用 bsw_ad9833_select() 切换频率
 *
 * 功放时序管理：
 *   ┌─────────────────────────────────────────────────────────────────┐
 *   │ PA_OFF  │ [10ms预热] │ SENDING (N*40ms) │ [10ms冷却] │ PA_OFF  │
 *   │         │  PA_ON     │ (逐bit切频率)    │ PA_OFF     │         │
 *   └─────────────────────────────────────────────────────────────────┘
 *        ↑                                          ↑
 *   bsw_bfsk_mod_send()                     完全关闭
 *   状态转 WARMUP                           状态转 IDLE
 *
 * 状态机：
 *   IDLE → 调用 send() → WARMUP（开启功放，等待预热）
 *   WARMUP → 预热完成 → SENDING（逐 bit 发送，定时器驱动）
 *   SENDING → 所有 bit 完成 → COOLDOWN（等待冷却）
 *   COOLDOWN → 冷却完成 → IDLE（关闭功放）
 *   任何状态 → 调用 stop() → IDLE（立即停止，关闭功放）
 *
 * @dependency  bsw_ad9833     (频率切换)
 *              bsw_power_ctrl (功放控制)
 *              mcal_timer     (定时器，控制 bit 速率)
 */

#include "bsw_bfsk_mod.h"
#include "bsw_ad9833.h"
#include "bsw_power_ctrl.h"

#include <string.h>

/* ========== 硬件时序配置 ========== */

/**
 * @brief 功放预热时间（ms）
 * @details 从功放使能到可以开始发送的等待时间
 *          需留足时间让功放输出稳定
 */
#define BSW_BFSK_MOD_PA_WARMUP_MS    (10U)

/**
 * @brief 功放冷却时间（ms）
 * @details 最后一个 bit 发送完后，到关闭功放的等待时间
 *          用于防止频率切换残留和功放的软启动
 */
#define BSW_BFSK_MOD_PA_COOLDOWN_MS  (10U)

/* ========== 内部状态 ========== */

/**
 * @brief BFSK 调制器上下文结构体
 * @details 存储调制状态和缓冲区
 * 
 * 生命周期：
 * - 初始化：bsw_bfsk_mod_init() 清零所有字段
 * - 使用：state 字段在状态间流转
 * - 清理：bsw_bfsk_mod_stop() 重置为 IDLE
 */
typedef struct
{
    uint16_t f0_hz;                         /* 低频点（bit=0） */
    uint16_t f1_hz;                         /* 高频点（bit=1） */
    uint32_t bit_period_ms;                 /* 每 bit 持续时间 */
    
    uint8_t  tx_buf[BSW_BFSK_MOD_MAX_BYTES]; /* 待发送字节缓冲 */
    uint16_t tx_len;                        /* 字节数 */
    uint16_t tx_bit_count;                  /* 当前字节的总 bit 数 */
    
    uint16_t current_bit_idx;               /* 当前正在发送的 bit 索引 */
    uint32_t bit_start_ms;                  /* 当前 bit 开始时刻 */
    
    uint32_t phase_start_ms;                /* 阶段开始时刻（用于预热/冷却计时） */
    
    bsw_bfsk_mod_state_t state;             /* 状态机 */
} bsw_bfsk_mod_ctx_t;

/**
 * @brief 调制器全局上下文
 * @rationale: 必须是静态全局，因为 ISR 和应用层都需要访问状态
 * @thread-safe: 应用层在发送前检查状态为 IDLE；bsw_bfsk_mod_run() 在应用主循环中单线程调用
 * @isr: bsw_bfsk_mod_on_bit_timer() 可能在硬件定时器 ISR 中调用（尚未启用）
 */
static bsw_bfsk_mod_ctx_t s_ctx = {0};

/* ========== 内部函数 ========== */

/**
 * @brief   从字节缓冲中提取第 bit_idx 个 bit（MSB first）
 * @param   byte_buf   字节缓冲
 * @param   bit_idx    第几个 bit（从 0 开始，全局计数）
 * @retval  0 或 1
 */
static uint8_t _get_bit(const uint8_t *byte_buf, uint16_t bit_idx)
{
    uint16_t byte_idx = bit_idx / 8u;
    uint8_t bit_pos = 7u - (bit_idx % 8u);  /* MSB first */
    return (byte_buf[byte_idx] >> bit_pos) & 1u;
}

/**
 * @brief   发送当前 bit（切换 AD9833 频率）
 * @note    根据当前 bit 值切换到 f0 或 f1
 */
static void _send_current_bit(void)
{
    if (s_ctx.current_bit_idx >= s_ctx.tx_bit_count) {
        return;  /* 已发送完所有 bit */
    }
    
    uint8_t bit_val = _get_bit(s_ctx.tx_buf, s_ctx.current_bit_idx);
    bsw_ad9833_reg_t reg = (bit_val == 0) ? BSW_AD9833_REG_0 : BSW_AD9833_REG_1;
    
    bsw_ad9833_select(reg);  /* 切换频率 */
}

/* ========== 公共 API 实现 ========== */

bsw_bfsk_mod_ret_t bsw_bfsk_mod_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = BSW_BFSK_MOD_IDLE;
    return BSW_BFSK_MOD_OK;
}

bsw_bfsk_mod_ret_t bsw_bfsk_mod_set_freq(uint16_t f0_hz,
                                         uint16_t f1_hz,
                                         uint32_t bit_period_ms)
{
    if (f0_hz == 0 || f1_hz == 0 || bit_period_ms == 0) {
        return BSW_BFSK_MOD_ERR_PARAM;
    }
    
    s_ctx.f0_hz = f0_hz;
    s_ctx.f1_hz = f1_hz;
    s_ctx.bit_period_ms = bit_period_ms;
    
    /* 装载频率到 AD9833 */
    if (bsw_ad9833_set_freq(BSW_AD9833_REG_0, f0_hz) != BSW_AD9833_OK) {
        return BSW_BFSK_MOD_ERR_HW;
    }
    if (bsw_ad9833_set_freq(BSW_AD9833_REG_1, f1_hz) != BSW_AD9833_OK) {
        return BSW_BFSK_MOD_ERR_HW;
    }
    
    return BSW_BFSK_MOD_OK;
}

bsw_bfsk_mod_ret_t bsw_bfsk_mod_send(const uint8_t *data, uint16_t len)
{
    /* 参数有效性检查 */
    if (data == NULL || len == 0 || len > BSW_BFSK_MOD_MAX_BYTES)
    {
        return BSW_BFSK_MOD_ERR_PARAM;
    }
    
    /* 为什么必须检查状态：
     * 防止重复发送导致：
     * 1. 功放状态错乱（已开启功放再次开启）
     * 2. 正在发送的数据被覆盖
     * 3. 冷却阶段被打断，功放提前关闭
     */
    if (s_ctx.state != BSW_BFSK_MOD_IDLE)
    {
        return BSW_BFSK_MOD_ERR_STATE;  /* 正在发送或出错中 */
    }
    
    /* 拷贝数据到内部缓冲 */
    memcpy(s_ctx.tx_buf, data, len);
    s_ctx.tx_len = len;
    s_ctx.tx_bit_count = len * 8u;
    
    /* 初始化发送状态 */
    s_ctx.current_bit_idx = 0u;
    s_ctx.bit_start_ms = 0u;
    s_ctx.phase_start_ms = 0u;  /* 等 run() 更新为当前时刻 */
    
    /* 为什么先开功放再发送：
     * 硬件时序要求：
     * 1. 功放从关闭到输出稳定需要 10ms
     * 2. 立即切换频率会导致振幅不稳定
     * 3. 接收端可能无法正确解调
     * 解决方案：状态进入 WARMUP，等待 10ms 后再进入 SENDING
     */
    if (bsw_power_ctrl_on(POWER_CH_PA) != POWER_CTRL_OK)
    {
        return BSW_BFSK_MOD_ERR_HW;  /* 功放控制失败 */
    }
    
    /* 状态切换为预热中（先不发送，等功放稳定） */
    s_ctx.state = BSW_BFSK_MOD_WARMUP;
    
    return BSW_BFSK_MOD_OK;
}

void bsw_bfsk_mod_run(uint32_t now_tick)
{
    switch (s_ctx.state)
    {
        case BSW_BFSK_MOD_WARMUP:
        {
            /* 为什么分两步检查：
             * 1. 第一次进入 WARMUP 时 phase_start_ms == 0
             * 2. 需要记录本阶段的起点时刻
             * 3. 后续周期通过与此时刻比对判断预热是否完成
             */
            if (s_ctx.phase_start_ms == 0)
            {
                s_ctx.phase_start_ms = now_tick;
                return;
            }
            
            /* 检查预热是否完成 */
            uint32_t warmup_elapsed = now_tick - s_ctx.phase_start_ms;
            if (warmup_elapsed < BSW_BFSK_MOD_PA_WARMUP_MS)
            {
                return;  /* 预热未完成，继续等待 */
            }
            
            /* 为什么立即发送第 0 个 bit：
             * 预热完成后，功放已稳定，可以立即开始发送
             * 第一个 bit 的频率决定了通信信号，不能延迟
             */
            s_ctx.state = BSW_BFSK_MOD_SENDING;
            s_ctx.bit_start_ms = now_tick;  /* 初始化 bit 发送起点 */
            s_ctx.current_bit_idx = 0;
            _send_current_bit();  /* 立即发送第 0 个 bit 对应的频率 */
            break;
        }
        
        case BSW_BFSK_MOD_SENDING:
        {
            /* 为什么按毫秒计时而非逐 bit 计数：
             * 1. 系统时间戳（now_tick）提供可靠的时间基准
             * 2. 通过时间差与 bit_period_ms 比对，计算应发送到第几个 bit
             * 3. 允许在定时误差范围内自动调整，比严格递增更鲁棒
             */
            uint32_t elapsed_ms = now_tick - s_ctx.bit_start_ms;
            uint32_t target_bit_idx = elapsed_ms / s_ctx.bit_period_ms;
            
            /* 未到切换时刻 */
            if (target_bit_idx == s_ctx.current_bit_idx)
            {
                return;
            }
            
            /* 切换到下一个 bit */
            s_ctx.current_bit_idx = target_bit_idx;
            
            /* 为什么完成后进入 COOLDOWN 而非直接 IDLE：
             * 1. 硬件要求：频率切换后需稳定 10ms 才能关闭功放
             * 2. 防止残留：防止频率叠加、功放不稳定
             * 3. 硬件保护：功放平稳关闭，延长寿命
             */
            if (s_ctx.current_bit_idx >= s_ctx.tx_bit_count)
            {
                s_ctx.state = BSW_BFSK_MOD_COOLDOWN;
                s_ctx.phase_start_ms = now_tick;  /* 记录冷却开始时刻 */
                return;
            }
            
            /* 发送当前 bit */
            _send_current_bit();
            break;
        }
        
        case BSW_BFSK_MOD_COOLDOWN:
        {
            /* 为什么需要冷却阶段：
             * 最后一个 bit 停止发送后，功放还在振荡
             * 立即关闭可能导致：
             * - 频率尖峰
             * - 接收端解调失败
             * 等待 10ms 确保功放输出稳定后再关
             */
            uint32_t cooldown_elapsed = now_tick - s_ctx.phase_start_ms;
            if (cooldown_elapsed < BSW_BFSK_MOD_PA_COOLDOWN_MS)
            {
                return;  /* 冷却未完成，继续等待 */
            }
            
            /* 冷却完成，关闭功放，回到空闲状态 */
            bsw_power_ctrl_off(POWER_CH_PA);
            s_ctx.state = BSW_BFSK_MOD_IDLE;
            s_ctx.current_bit_idx = 0;
            s_ctx.tx_len = 0;
            break;
        }
        
        case BSW_BFSK_MOD_IDLE:
        case BSW_BFSK_MOD_DONE_OK:
        default:
            /* 空闲或已完成，无需处理 */
            break;
    }
}

bsw_bfsk_mod_state_t bsw_bfsk_mod_get_state(void)
{
    return s_ctx.state;
}

bsw_bfsk_mod_ret_t bsw_bfsk_mod_stop(void)
{
    /* 为什么立即关闭功放而非等待冷却：
     * 应用层调用 stop() 表示：
     * 1. 需要立即中断发送（异常恢复）
     * 2. FSM 状态切换，不能继续占用硬件
     * 3. 安全起见立即断电，不需要正常关闭时序
     */
    bsw_power_ctrl_off(POWER_CH_PA);
    
    /* 清空所有状态 */
    s_ctx.state = BSW_BFSK_MOD_IDLE;
    s_ctx.current_bit_idx = 0u;
    s_ctx.tx_len = 0u;
    s_ctx.phase_start_ms = 0u;
    s_ctx.bit_start_ms = 0u;
    
    return BSW_BFSK_MOD_OK;
}

/* ========== 定时器回调 ========== */

/**
 * @brief   定时器回调：每个 bit_period_ms 触发一次
 * @param   now_tick ISR 中获取的当前时刻（xTaskGetTickCountFromISR() 或等效值）
 * @note    由 mcal_timer ISR 注册
 * @details 本函数负责：取下一个 bit → 切换 AD9833 频率 → 推进状态机
 * 
 * @rationale: 为什么接收时间参数而非内部获取
 * - ISR 中禁止调用 HAL_GetTick()（与 FreeRTOS SysTick 冲突）
 * - 时间由调用者（mcal_timer ISR）从 FreeRTOS API 获取
 * - 确保与应用层时间戳来源一致
 */
void bsw_bfsk_mod_on_bit_timer(uint32_t now_tick)
{
    /* 该回调由 mcal_timer 在每个 bit_period_ms 时刻调用
     * 若采用硬定时器精确控制，可以在这里推进状态
     */
    
    if (s_ctx.state != BSW_BFSK_MOD_SENDING)
    {
        return;
    }
    
    /* 在 SENDING 阶段，推进到下一个 bit */
    s_ctx.current_bit_idx++;
    
    if (s_ctx.current_bit_idx >= s_ctx.tx_bit_count)
    {
        /* 为什么在 ISR 中记录冷却开始时刻：
         * 最后一个 bit 刚好被处理，需要立即开始计时冷却阶段
         * 使用参数传入的 now_tick，避免调用 HAL_GetTick()
         */
        s_ctx.state = BSW_BFSK_MOD_COOLDOWN;
        s_ctx.phase_start_ms = now_tick;  /* 记录冷却开始时刻 */
        return;
    }
    
    _send_current_bit();
}
