/**
 * @file    bsw_bfsk_demod.c
 * @brief   BFSK 解调器 - BSW 层实现（半周期测频法）
 *
 * 原理：
 *   - TIM1 CH1 捕获上升沿（时刻 t_rise）
 *   - TIM1 CH2 捕获下降沿（时刻 t_fall）
 *   - 半周期 Δt = t_fall - t_rise（单位：TIM1 tick，12.5ns @ 80MHz）
 *   - 频率估算：f_est ≈ 80_000_000 / (2 × Δt) Hz
 *   - 频率判定：
 *       |f_est - f0| < tolerance → bit = 0
 *       |f_est - f1| < tolerance → bit = 1
 *       否则 → 噪声/错误，丢弃并复位
 *
 * 职责边界（重构后）：
 *   物理层：频率 → bit 流（只输出 bit，不解析帧结构）
 *   协议层：bit 流 → 帧对象（由 bsw_proto 负责）
 *
 * @dependency  bsw_bfsk_demod.h
 *              mcal_timer.h        (TIM1 IC 启停)
 *              stm32l4xx_hal_tim.h (HAL_TIM_IC_CaptureCallback 重写)
 */

#include "bsw_bfsk_demod.h"

#include <stddef.h>   /* NULL 定义 */
#include <string.h>   /* memset */
#include "main.h"     /* HAL_GetTick, TIM_HandleTypeDef */
#include "mcal_timer.h"
#include "mcal_gpio.h"  /* 比较器电路使能控制 */

/* ========== 比较器电路控制宏 ========== */
#define COMPARATOR_OPEN()   mcal_gpio_write(GPIO_PIN_COMP_EN, GPIO_HIGH)  /* 使能比较器电路 */
#define COMPARATOR_CLOSE()  mcal_gpio_write(GPIO_PIN_COMP_EN, GPIO_LOW)   /* 关闭比较器电路（节省功耗） */

/* ========== 解调配置参数 ========== */
#define DEMOD_TIM1_CLK_HZ         10000000U   /* TIM1 时钟频率 (80MHz / 8 分频) */
#define DEMOD_ARR_OVERFLOW        65536UL     /* TIM1 计数器回绕阈值 */
#define DEMOD_FREQ_TOLERANCE_PCT  20U         /* 频率容差 ±20% */
#define DEMOD_BIT_BUF_SIZE        256U        /* bit 缓冲区大小（256 bit = 32 字节）*/
#define DEMOD_MIN_BIT_DURATION_MS 25U         /* bit 最小持续时间（ms），低于此值视为噪声/偏移残留 */
#define DEMOD_BITS_PER_BYTE       8U          /* 每字节位数 */
#define DEMOD_FREQ_NOT_LOCKED     0U          /* 频率未锁定标志 */
/* 注：
 * - bit_period = 40ms，使用 25ms 作为门槛（62.5%）
 * - 低于 25ms → 要么是噪声，要么是启动偏移的尾部残留 → 丢弃
 * - ≥ 25ms → 足够长，说明是真实的 bit → push
 * - 不再依赖周期数判据（1025Hz 一个周期才 1ms，周期数判据不可靠）
 * 
 * f0/f1 最小间距由 app_node_fsm._select_freq_pair() 保证 ≥ 150 Hz，解调器不重复校验 */

/* ========== 静态上下文 ========== */

/* 边沿捕获状态机（完整周期验证） */
typedef enum {
    EDGE_WAIT_RISE1,   /* 等待第一个上升沿（周期起点） */
    EDGE_WAIT_FALL,    /* 等待下降沿（高电平结束） */
    EDGE_WAIT_RISE2,   /* 等待第二个上升沿（低电平结束，周期完整） */
} edge_state_t;

/* 频率识别结果（单个周期） */
typedef enum {
    FREQ_UNKNOWN = 0,  /* 未识别 / 无效 */
    FREQ_IS_F0   = 1,  /* 识别为 f0 */
    FREQ_IS_F1   = 2,  /* 识别为 f1 */
} freq_id_t;

typedef struct {
    bsw_bfsk_demod_state_t state;
    
    /* 频率配置（从 app_node_fsm 注入） */
    uint16_t f0_hz;              /* 低频点（bit = 0） */
    uint16_t f1_hz;              /* 高频点（bit = 1） */
    uint32_t f0_half_period_ticks;  /* f0 半周期标称 tick 数 */
    uint32_t f1_half_period_ticks;  /* f1 半周期标称 tick 数 */
    uint32_t tolerance_ticks;       /* 容差窗口（tick） */
    
    /* 边沿捕获状态机 */
    edge_state_t edge_state;
    uint32_t t_rise1;            /* 第一个上升沿时刻 */
    uint32_t t_fall;             /* 下降沿时刻 */
    uint32_t t_rise2;            /* 第二个上升沿时刻 */
    
    /* 时间窗口解调（基于 bit_period_ms + 频率切换同步） */
    uint32_t bit_period_ms;      /* 每个 bit 持续时间（协议层配置） */
    uint32_t window_start_tick;  /* 当前窗口起点（HAL_GetTick） */
    freq_id_t window_current_freq;  /* 当前窗口主导频率（用于切换检测） */
    uint16_t window_stable_count;   /* 当前频率连续稳定的周期数 */
    uint8_t  window_bit_pushed;     /* 当前窗口是否已 push bit（防重复） */
    
    /* bit 流缓冲（物理层职责：只输出 bit，不解析帧） */
    uint8_t  bit_buf[DEMOD_BIT_BUF_SIZE];  /* bit 流缓冲 */
    uint16_t bit_count;                     /* 当前累积的 bit 数量 */
    
    /* 超时管理 */
    uint32_t timeout_tick;       /* HAL_GetTick() 超时阈值 */
    
    /* 解调结果 */
    bsw_bfsk_demod_result_t result;
    
} demod_ctx_t;

static demod_ctx_t s_ctx;  /* 全局静态变量自动零初始化 */

/* ========== 内部辅助函数（前向声明） ========== */
static void _push_bit(uint8_t bit);
static void _reset_bit_buffer(void);
static void _process_freq_result(freq_id_t freq);

/* ========== 内部辅助函数（实现） ========== */

/**
 * @brief   复位 bit 缓冲区（检测到异常频率时调用）
 */
static void _reset_bit_buffer(void)
{
    s_ctx.edge_state = EDGE_WAIT_RISE1;  /* 复位边沿状态机 */
    s_ctx.window_start_tick = HAL_GetTick();
    s_ctx.window_current_freq = FREQ_UNKNOWN;
    s_ctx.window_stable_count = 0;
    s_ctx.window_bit_pushed = 0;
    
    /* 清空 bit 缓冲区 */
    s_ctx.bit_count = 0;
}

/**
 * @brief   处理单个周期的频率判定结果（时间窗口累积 + 频率切换同步）
 * @param   freq  本周期识别出的频率 (FREQ_IS_F0 / FREQ_IS_F1 / FREQ_UNKNOWN)
 * @note    双重保底机制（解决时序对齐问题）：
 *          
 *          机制 1：40ms 时间窗口到期
 *            - 窗口内持续 ≥25ms 同一频率 → push bit
 *            - 重置窗口，继续下一个 bit
 *          
 *          机制 2：频率切换检测（时序同步触发器）
 *            - 检测到频率从 f0→f1 或 f1→f0
 *            - 说明前面的窗口起点偏移了（波形早到了）
 *            - 立即：
 *              1) 如果前面频率持续 ≥25ms → push 该 bit
 *              2) 重置窗口起点为当前时刻（重新对齐）
 *              3) 开始新频率的计时
 *          
 *          为什么用时间而不是周期数？
 *            - f0 = 125Hz：1 周期 = 8ms，3 周期 = 24ms ✓
 *            - f1 = 1025Hz：1 周期 = 1ms，3 周期 = 3ms ❌（太短！）
 *            - 使用统一的时间门槛（25ms）更可靠、更简洁
 *          
 *          示例（波形已来 10ms 才启动）：
 *            [0-10ms]: f0 波形（未捕获）
 *            [10ms]: 启动，window_start = 10ms
 *            [10-40ms]: 持续捕获 f0，duration = 30ms ≥ 25ms ✓
 *            [40ms]: 检测到 f1（频率切换！）
 *              → push bit=0（前面的 f0，持续了 30ms）
 *              → window_start = 40ms（重新对齐）
 *              → 开始新的 f1 计时
 *            [40-80ms]: 持续捕获 f1，duration = 40ms ≥ 25ms ✓
 *            [80ms]: 窗口到期（40ms）
 *              → push bit=1
 *            
 *          对齐成功！后续时序正确。
 */
static void _process_freq_result(freq_id_t freq)
{
    uint32_t now_tick = HAL_GetTick();
    
    if (freq == FREQ_UNKNOWN) {
        /* 无效周期（对称性或频率不匹配） → 复位 */
        _reset_bit_buffer();
        return;
    }
    
    /* ---- 机制 2：频率切换检测（同步触发器） ---- */
    if (s_ctx.window_current_freq != FREQ_UNKNOWN 
        && s_ctx.window_current_freq != freq) {
        /* 检测到频率切换！ */
        
        /* 计算前一个频率的持续时间 */
        uint32_t window_elapsed = now_tick - s_ctx.window_start_tick;
        
        /* 如果持续时间 ≥25ms，说明是真实 bit，push 它 */
        if (window_elapsed >= DEMOD_MIN_BIT_DURATION_MS
            && (s_ctx.window_bit_pushed == 0u)) {
            uint8_t bit = (s_ctx.window_current_freq == FREQ_IS_F0) ? 0u : 1u;
            _push_bit(bit);
            s_ctx.window_bit_pushed = 1u;
        }
        /* else: 持续时间 < 25ms，要么是噪声，要么是启动偏移的尾部残留 → 丢弃 */
        
        /* 重置窗口起点（重新对齐时序） */
        s_ctx.window_start_tick = now_tick;
        s_ctx.window_current_freq = freq;
        s_ctx.window_stable_count = 1u;  /* 新频率的第一个周期 */
        s_ctx.window_bit_pushed = 0u;
        return;
    }
    
    /* ---- 同一频率连续出现，累积稳定性计数 ---- */
    if (s_ctx.window_current_freq == FREQ_UNKNOWN) {
        /* 窗口刚启动，记录第一个频率 */
        s_ctx.window_current_freq = freq;
        s_ctx.window_stable_count = 1;
    } else {
        /* 同频连续，累加计数（用于调试/统计，判决逻辑只看时间） */
        ++s_ctx.window_stable_count;
    }
    
    /* ---- 机制 1：40ms 时间窗口到期 ---- */
    uint32_t window_elapsed = now_tick - s_ctx.window_start_tick;
    
    if (window_elapsed >= s_ctx.bit_period_ms) {
        /* 窗口到期，检查持续时间是否达标 */
        if (window_elapsed >= DEMOD_MIN_BIT_DURATION_MS
            && (s_ctx.window_bit_pushed == 0u)) {
            uint8_t bit = (s_ctx.window_current_freq == FREQ_IS_F0) ? 0u : 1u;
            _push_bit(bit);
        }
        /* else: 持续时间不足（< 25ms），说明是噪声或启动偏移残留 → 跳过 */
        
        /* 开始新窗口 */
        s_ctx.window_start_tick = now_tick;
        s_ctx.window_current_freq = freq;  /* 继承当前频率到新窗口 */
        s_ctx.window_stable_count = 1u;
        s_ctx.window_bit_pushed = 0u;
    }
}

/**
 * @brief   向 bit 缓冲区追加一个 bit（物理层职责：只累积 bit 流）
 * @param   bit  0 或 1
 */
static void _push_bit(uint8_t bit)
{
    /* 检查缓冲区是否已满 */
    if (s_ctx.bit_count >= DEMOD_BIT_BUF_SIZE) {
        /* 缓冲区满，停止接收（标记为错误） */
        s_ctx.state = BSW_BFSK_DEMOD_DONE_ERR;
        mcal_timer_ic_stop(MCAL_TIMER_TIM1);
        COMPARATOR_CLOSE();
        return;
    }
    
    /* 累积 bit 到缓冲区 */
    s_ctx.bit_buf[s_ctx.bit_count++] = bit ? 1u : 0u;
}

/* ========== 公共 API 实现 ========== */

bsw_bfsk_demod_ret_t bsw_bfsk_demod_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = BSW_BFSK_DEMOD_IDLE;
    return BSW_BFSK_DEMOD_OK;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_start(uint16_t f0_hz,
                                         uint16_t f1_hz,
                                         uint32_t bit_period_ms,
                                         uint32_t timeout_ms,
                                         uint8_t local_addr)
{
    /* 参数合法性检查（仅最基本项） */
    if ((f0_hz == DEMOD_FREQ_NOT_LOCKED) || (f1_hz == DEMOD_FREQ_NOT_LOCKED)) {
        return BSW_BFSK_DEMOD_ERR_PARAM;
    }
    if (f0_hz == f1_hz) {
        /* 频点不能相同，否则无法区分 bit 0/1 */
        return BSW_BFSK_DEMOD_ERR_PARAM;
    }
    if (f0_hz > f1_hz) {
        /* 强制 f0 < f1（协议约定：f0 低频 = bit 0，f1 高频 = bit 1） */
        uint16_t tmp = f0_hz;
        f0_hz = f1_hz;
        f1_hz = tmp;
    }
    if (bit_period_ms == DEMOD_FREQ_NOT_LOCKED) {
        return BSW_BFSK_DEMOD_ERR_PARAM;
    }
    /* 注：f0/f1 最小间距由 app_node_fsm._select_freq_pair() 保证 ≥ 150 Hz，
     * 解调器不再重复校验 */

    if (s_ctx.state != BSW_BFSK_DEMOD_IDLE) {
        return BSW_BFSK_DEMOD_ERR_STATE;
    }

    /* 计算半周期标称值（TIM1 tick @ 10MHz）：
     *   T = 1/f，半周期 = T/2 = 1/(2f)
     *   tick 数 = (TIM1_CLK / 2) / f_hz
     */
    s_ctx.f0_hz = f0_hz;
    s_ctx.f1_hz = f1_hz;
    s_ctx.bit_period_ms = bit_period_ms;
    s_ctx.f0_half_period_ticks = (DEMOD_TIM1_CLK_HZ / 2) / f0_hz;
    s_ctx.f1_half_period_ticks = (DEMOD_TIM1_CLK_HZ / 2) / f1_hz;

    /* 容差窗口：取两个半周期中较小的那个 × 30%（用较小的容差更严格，
     * 防止频率过近时误判）。但协议规定最小间距 150 Hz，此容差已远小于间距。
     * 此处取较小半周期 × 30% 以确保判据稳健。 */
    uint32_t min_half_period = (s_ctx.f0_half_period_ticks < s_ctx.f1_half_period_ticks)
                               ? s_ctx.f0_half_period_ticks
                               : s_ctx.f1_half_period_ticks;
    s_ctx.tolerance_ticks = (min_half_period * 30U) / 100U;

    /* 清空 bit 缓冲 */
    _reset_bit_buffer();
    
    /* 设置超时 */
    s_ctx.timeout_tick = HAL_GetTick() + timeout_ms;

    /* 使能比较器电路（正弦波→方波转换） */
    COMPARATOR_OPEN();

    /* 启动 TIM1 IC */
    mcal_timer_ic_start(MCAL_TIMER_TIM1);
    
    s_ctx.state = BSW_BFSK_DEMOD_RUNNING;
    return BSW_BFSK_DEMOD_OK;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_stop(void)
{
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
    
    /* 关闭比较器电路（节省功耗） */
    COMPARATOR_CLOSE();
    
    _reset_bit_buffer();
    s_ctx.state = BSW_BFSK_DEMOD_IDLE;
    return BSW_BFSK_DEMOD_OK;
}

void bsw_bfsk_demod_run(uint32_t now_tick)
{
    if (s_ctx.state != BSW_BFSK_DEMOD_RUNNING) {
        return;
    }
    
    /* 超时检查 */
    if (now_tick >= s_ctx.timeout_tick) {
        mcal_timer_ic_stop(MCAL_TIMER_TIM1);
        COMPARATOR_CLOSE();
        s_ctx.state = BSW_BFSK_DEMOD_DONE_TIMEOUT;
    }
    
    /* 检查 bit 缓冲区是否接收到足够数据（可选：提前完成条件）
     * 注：这里可以添加启发式判断，比如接收到一定数量的 bit 后
     * 就认为一帧可能已完成，提前结束接收。当前简化处理：等超时或缓冲区满。
     */
}

bsw_bfsk_demod_state_t bsw_bfsk_demod_get_state(void)
{
    return s_ctx.state;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_take_result(bsw_bfsk_demod_result_t *out)
{
    if (out == NULL) {
        return BSW_BFSK_DEMOD_ERR_PARAM;
    }
    
    if (s_ctx.state != BSW_BFSK_DEMOD_DONE_OK) {
        return BSW_BFSK_DEMOD_ERR_STATE;
    }
    
    /* 填充结果（bit 流 + 频率信息） */
    out->f0_hz = s_ctx.f0_hz;
    out->f1_hz = s_ctx.f1_hz;
    out->bit_count = s_ctx.bit_count;
    out->timestamp_ms = HAL_GetTick();
    
    /* 拷贝 bit 流 */
    memcpy(out->bit_buf, s_ctx.bit_buf, s_ctx.bit_count);
    
    /* 消费后回到 IDLE，允许下一轮监听 */
    s_ctx.state = BSW_BFSK_DEMOD_IDLE;
    
    return BSW_BFSK_DEMOD_OK;
}

/* ========== TIM1 IC 中断回调（HAL 层） ========== */

/**
 * @brief   HAL TIM1 输入捕获中断回调（重写 HAL 的 __weak 函数）
 * @param   htim  TIM 句柄
 * @note    完整周期验证：测高半周期 + 低半周期，对称性检查
 *          状态机流程：
 *            EDGE_WAIT_RISE1 → CH1上升沿 → EDGE_WAIT_FALL
 *            EDGE_WAIT_FALL  → CH2下降沿 → EDGE_WAIT_RISE2
 *            EDGE_WAIT_RISE2 → CH1上升沿 → 完整周期判定 → 回到 EDGE_WAIT_FALL
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1) {
        return;
    }
    
    if (s_ctx.state != BSW_BFSK_DEMOD_RUNNING) {
        return;
    }
    
    /* CH1: 上升沿捕获 */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t ccr1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        if (s_ctx.edge_state == EDGE_WAIT_RISE1) {
            /* 第一个上升沿：周期起点 */
            s_ctx.t_rise1 = ccr1;
            s_ctx.edge_state = EDGE_WAIT_FALL;
        }
        else if (s_ctx.edge_state == EDGE_WAIT_RISE2) {
            /* 第二个上升沿：周期结束，开始完整周期验证 */
            s_ctx.t_rise2 = ccr1;
            
            /* 计算高半周期、低半周期、完整周期（处理溢出） */
            uint32_t t_high, t_low, t_full;
            
            if (s_ctx.t_fall >= s_ctx.t_rise1) {
                t_high = s_ctx.t_fall - s_ctx.t_rise1;
            } else {
                t_high = (DEMOD_ARR_OVERFLOW + s_ctx.t_fall) - s_ctx.t_rise1;
            }
            
            if (s_ctx.t_rise2 >= s_ctx.t_fall) {
                t_low = s_ctx.t_rise2 - s_ctx.t_fall;
            } else {
                t_low = (DEMOD_ARR_OVERFLOW + s_ctx.t_rise2) - s_ctx.t_fall;
            }
            
            if (s_ctx.t_rise2 >= s_ctx.t_rise1) {
                t_full = s_ctx.t_rise2 - s_ctx.t_rise1;
            } else {
                t_full = (DEMOD_ARR_OVERFLOW + s_ctx.t_rise2) - s_ctx.t_rise1;
            }
            
            /* 对称性检查：高半周期和低半周期必须接近 */
            int32_t asymmetry = (int32_t)t_high - (int32_t)t_low;
            if (asymmetry < 0) asymmetry = -asymmetry;
            
            /* 容差 = 完整周期的 25%（允许一定波形畸变） */
            uint32_t symmetry_tolerance = t_full / 4;
            
            if ((uint32_t)asymmetry > symmetry_tolerance) {
                /* 波形不对称，可能是噪声毛刺 → 丢弃，从下降沿重新开始 */
                s_ctx.edge_state = EDGE_WAIT_FALL;
                s_ctx.t_rise1 = s_ctx.t_rise2;  /* 当前上升沿作为新起点 */
                return;
            }
            
            /* 对称性检查通过，开始频率判定（用完整周期的一半） */
            uint32_t half_period = t_full / 2;
            
            int32_t diff_f0 = (int32_t)half_period - (int32_t)s_ctx.f0_half_period_ticks;
            int32_t diff_f1 = (int32_t)half_period - (int32_t)s_ctx.f1_half_period_ticks;
            
            if (diff_f0 < 0) diff_f0 = -diff_f0;
            if (diff_f1 < 0) diff_f1 = -diff_f1;
            
            freq_id_t detected_freq = FREQ_UNKNOWN;
            
            if ((uint32_t)diff_f0 <= s_ctx.tolerance_ticks) {
                detected_freq = FREQ_IS_F0;
            }
            else if ((uint32_t)diff_f1 <= s_ctx.tolerance_ticks) {
                detected_freq = FREQ_IS_F1;
            }
            
            /* 交给时间窗口解调逻辑处理（累积到当前窗口，窗口到期时多数表决） */
            _process_freq_result(detected_freq);
            
            /* 继续下一周期 */
            s_ctx.edge_state = EDGE_WAIT_FALL;
            s_ctx.t_rise1 = s_ctx.t_rise2;
        }
    }
    /* CH2: 下降沿捕获 */
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        if (s_ctx.edge_state != EDGE_WAIT_FALL) {
            return;  /* 没有对应的上升沿，丢弃 */
        }
        
        uint32_t ccr2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        s_ctx.t_fall = ccr2;
        s_ctx.edge_state = EDGE_WAIT_RISE2;  /* 等待第二个上升沿 */
    }
}

/* ========== 兼容旧接口（占位） ========== */

void bsw_bfsk_demod_on_ic_edge(uint32_t tick_us)
{
    /* 此函数已废弃，实际解调逻辑在 HAL_TIM_IC_CaptureCallback() 中实现 */
    (void)tick_us;
}
