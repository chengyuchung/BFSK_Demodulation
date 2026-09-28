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
 * 帧格式（暂定，待协议确认）：
 *   帧头(8 bit) + 数据(N bit) + CRC(8 bit)
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

/* ========== 解调配置参数 ========== */
#define DEMOD_TIM1_CLK_HZ         10000000U   /* TIM1 时钟频率 (80MHz / 8 分频) */
#define DEMOD_ARR_OVERFLOW        65536UL     /* TIM1 计数器回绕阈值 */
#define DEMOD_FREQ_TOLERANCE_PCT  20U         /* 频率容差 ±20% */
#define DEMOD_MAX_PAYLOAD_LEN     10U         /* 载荷域最大长度（字节） */
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

/* 帧域状态机（边解调边过滤） */
typedef enum {
    FIELD_WAIT_HEAD,      /* 等待帧头 0xFF（8 个连续 1） */
    FIELD_RECV_ADDR,      /* 接收地址域（8 bit：DST(4) + SRC(4)） */
    FIELD_RECV_TYPE_INFO, /* 接收类型域（8 bit：TYPE(4) + MSG_NUM(4)） */
    FIELD_RECV_SEQ,       /* 接收序号域（8 bit） */
    FIELD_RECV_LEN,       /* 接收长度域（8 bit） */
    FIELD_RECV_PAYLOAD,   /* 接收载荷域（N 字节，由 LEN 决定） */
    FIELD_RECV_CRC16,     /* 接收 CRC16（2 字节） */
    FIELD_RECV_TAIL,      /* 接收帧尾 0x00（8 bit） */
    FIELD_DONE,           /* 完整帧接收完成 */
} field_state_t;

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
    
    /* 域级状态机（边解调边过滤） */
    field_state_t field_state;
    uint8_t  byte_buf;           /* 当前字节缓冲（累积 8 bit） */
    uint8_t  byte_bit_count;     /* 当前字节已累积的 bit 数（0~7） */
    uint8_t  local_addr;         /* 本机地址（用于 ADDR 域过滤） */
    
    /* 帧域缓冲（接收后的完整字段） */
    uint8_t  frame_addr;         /* ADDR 域（DST + SRC） */
    uint8_t  frame_type_info;    /* TYPE_INFO 域 */
    uint8_t  frame_seq;          /* SEQ 域 */
    uint8_t  frame_len;          /* LEN 域（PAYLOAD 字节数） */
    uint8_t  frame_payload[DEMOD_MAX_PAYLOAD_LEN];  /* PAYLOAD 域（最大 10 字节） */
    uint8_t  frame_crc16[2];     /* CRC16 域（2 字节） */
    uint16_t payload_recv_count; /* PAYLOAD 已接收字节数 */
    uint8_t  crc_recv_count;     /* CRC16 已接收字节数 */
    
    /* 超时管理 */
    uint32_t timeout_tick;       /* HAL_GetTick() 超时阈值 */
    
    /* 解调结果 */
    bsw_bfsk_demod_result_t result;
    
} demod_ctx_t;

static demod_ctx_t s_ctx;  /* 全局静态变量自动零初始化 */

/* ========== 内部辅助函数（前向声明） ========== */
static void _push_bit(uint8_t bit);
static void _reset_bit_buffer(void);
static void _reset_field_parser(void);
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
    
    /* 复位域状态机 */
    _reset_field_parser();
}

/**
 * @brief   复位域解析状态机（丢弃当前帧，重新搜索帧头）
 */
static void _reset_field_parser(void)
{
    s_ctx.field_state = FIELD_WAIT_HEAD;
    s_ctx.byte_buf = 0;
    s_ctx.byte_bit_count = 0;
    s_ctx.payload_recv_count = 0;
    s_ctx.crc_recv_count = 0;
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
 * @brief   向字节缓冲区追加一个 bit（核心：域级状态机 + 早期过滤）
 * @param   bit  0 或 1
 */
static void _push_bit(uint8_t bit)
{
    /* 累积当前字节（LSB first）*/
    if (bit) {
        s_ctx.byte_buf |= (1U << s_ctx.byte_bit_count);
    }
    s_ctx.byte_bit_count++;
    
    /* 字节未满 8 bit，继续累积 */
    if (s_ctx.byte_bit_count < 8) {
        return;
    }
    
    /* 字节累积完成（8 bit），处理当前域 */
    uint8_t byte = s_ctx.byte_buf;
    uint8_t dst_addr;  /* 提前声明，避免 switch 内声明导致的警告 */
    
    switch (s_ctx.field_state) {
        case FIELD_WAIT_HEAD:
            /* 搜索帧头 0xFF（8 个连续 1） */
            if (byte == 0xFF) {
                /* 找到帧头，进入 ADDR 域 */
                s_ctx.field_state = FIELD_RECV_ADDR;
            }
            /* 否则丢弃，继续搜索下一个字节 */
            break;
            
        case FIELD_RECV_ADDR:
            /* 接收地址域：DST(高 4 bit) + SRC(低 4 bit) */
            s_ctx.frame_addr = byte;
            dst_addr = (byte >> 4) & 0x0F;
            
            /* 早期过滤：检查目的地址 */
            if (dst_addr != s_ctx.local_addr) {
                /* 地址不匹配，丢弃此帧，重新搜索帧头 */
                _reset_field_parser();
                break;
            }
            
            /* 地址匹配，继续接收 TYPE_INFO 域 */
            s_ctx.field_state = FIELD_RECV_TYPE_INFO;
            break;
            
        case FIELD_RECV_TYPE_INFO:
            /* 接收类型域：TYPE(高 4 bit) + MSG_NUM(低 4 bit) */
            s_ctx.frame_type_info = byte;
            s_ctx.field_state = FIELD_RECV_SEQ;
            break;
            
        case FIELD_RECV_SEQ:
            /* 接收序号域 */
            s_ctx.frame_seq = byte;
            s_ctx.field_state = FIELD_RECV_LEN;
            break;
            
        case FIELD_RECV_LEN:
            /* 接收长度域 */
            s_ctx.frame_len = byte;
            
            /* 长度合法性检查 */
            if (s_ctx.frame_len > DEMOD_MAX_PAYLOAD_LEN) {
                /* 长度非法，丢弃此帧 */
                _reset_field_parser();
                break;
            }
            
            /* 如果 PAYLOAD 长度为 0，直接进入 CRC16 域 */
            if (s_ctx.frame_len == 0) {
                s_ctx.field_state = FIELD_RECV_CRC16;
            } else {
                s_ctx.field_state = FIELD_RECV_PAYLOAD;
                s_ctx.payload_recv_count = 0;
            }
            break;
            
        case FIELD_RECV_PAYLOAD:
            /* 接收载荷域（逐字节） */
            s_ctx.frame_payload[s_ctx.payload_recv_count++] = byte;
            
            /* 检查是否接收完 */
            if (s_ctx.payload_recv_count >= s_ctx.frame_len) {
                s_ctx.field_state = FIELD_RECV_CRC16;
                s_ctx.crc_recv_count = 0;
            }
            break;
            
        case FIELD_RECV_CRC16:
            /* 接收 CRC16（2 字节，低字节先） */
            s_ctx.frame_crc16[s_ctx.crc_recv_count++] = byte;
            
            /* 检查是否接收完 */
            if (s_ctx.crc_recv_count >= 2) {
                s_ctx.field_state = FIELD_RECV_TAIL;
            }
            break;
            
        case FIELD_RECV_TAIL:
            /* 接收帧尾 0x00 */
            if (byte == 0x00) {
                /* 帧接收完成！ */
                s_ctx.field_state = FIELD_DONE;
                s_ctx.state = BSW_BFSK_DEMOD_DONE_OK;
                
                /* 停止硬件 */
                mcal_timer_ic_stop(MCAL_TIMER_TIM1);
                
                /* 填充结果 */
                s_ctx.result.f0_hz = s_ctx.f0_hz;
                s_ctx.result.f1_hz = s_ctx.f1_hz;
                s_ctx.result.frame_addr = s_ctx.frame_addr;
                s_ctx.result.frame_type_info = s_ctx.frame_type_info;
                s_ctx.result.frame_seq = s_ctx.frame_seq;
                s_ctx.result.frame_len = s_ctx.frame_len;
                memcpy(s_ctx.result.frame_payload, s_ctx.frame_payload, s_ctx.frame_len);
                memcpy(s_ctx.result.frame_crc16, s_ctx.frame_crc16, 2);
            } else {
                /* 帧尾错误，丢弃此帧 */
                _reset_field_parser();
            }
            break;
            
        case FIELD_DONE:
            /* 已完成，不再接收（需调用 take_result 消费） */
            break;
    }
    
    /* 复位字节缓冲，准备接收下一个字节 */
    s_ctx.byte_buf = 0;
    s_ctx.byte_bit_count = 0;
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
    s_ctx.local_addr = local_addr & 0x0F;  /* 限制为 4 位 */
    s_ctx.f0_half_period_ticks = (DEMOD_TIM1_CLK_HZ / 2) / f0_hz;
    s_ctx.f1_half_period_ticks = (DEMOD_TIM1_CLK_HZ / 2) / f1_hz;

    /* 容差窗口：取两个半周期中较小的那个 × 30%（用较小的容差更严格，
     * 防止频率过近时误判）。但协议规定最小间距 150 Hz，此容差已远小于间距。
     * 此处取较小半周期 × 30% 以确保判据稳健。 */
    uint32_t min_half_period = (s_ctx.f0_half_period_ticks < s_ctx.f1_half_period_ticks)
                               ? s_ctx.f0_half_period_ticks
                               : s_ctx.f1_half_period_ticks;
    s_ctx.tolerance_ticks = (min_half_period * 30U) / 100U;

    /* 清空 bit 缓冲 + 域状态机 */
    _reset_bit_buffer();
    
    /* 设置超时 */
    s_ctx.timeout_tick = HAL_GetTick() + timeout_ms;
    
    /* 启动 TIM1 IC */
    mcal_timer_ic_start(MCAL_TIMER_TIM1);
    
    s_ctx.state = BSW_BFSK_DEMOD_RUNNING;
    return BSW_BFSK_DEMOD_OK;
}

bsw_bfsk_demod_ret_t bsw_bfsk_demod_stop(void)
{
    mcal_timer_ic_stop(MCAL_TIMER_TIM1);
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
        s_ctx.state = BSW_BFSK_DEMOD_DONE_TIMEOUT;
    }
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
    
    /* 拷贝结果 */
    memcpy(out, &s_ctx.result, sizeof(bsw_bfsk_demod_result_t));
    
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
