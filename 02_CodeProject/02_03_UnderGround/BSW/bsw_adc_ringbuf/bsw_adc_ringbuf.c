/**
 * @file    bsw_adc_ringbuf.c
 * @brief   ADC1 环形缓冲区 - BSW 层实现
 *
 * 模块自带的 512 点环形 buffer 直接作为 ADC1 DMA 的目标地址：
 *
 *   HAL_ADC_Start_DMA(s_buf, 512)
 *   ↓
 *   硬件按 TIM6 TRGO 节奏（10 kHz）持续写：
 *     s_buf[0..255]   → ConvHalfCplt 中断 → _dma_half_cb → s_write_idx = 256
 *     s_buf[256..511] → ConvCplt 中断     → _dma_cplt_cb → s_write_idx = 0 (wrap)
 *   ↓
 *   上层 snapshot()/read() 关中断读 s_buf 拷贝到 out。
 *
 * 关键设计：
 *   - s_buf 必须 32 位对齐（STM32L4 DMA source/dest 硬要求），用 aligned(4)。
 *   - s_write_idx 是 volatile，但不互斥：写者唯一（DMA 中断），
 *     读者关中断独享拷贝临界区。
 *   - s_data_ready 在首次 cplt_cb 时置 1，表示 buffer 内至少有过一次
 *     完整 512 点有效数据（本版本暂未对 reader 暴露，仅用于未来扩展）。
 *
 * @dependency   mcal_adc, bsw_log
 */

#include "bsw_adc_ringbuf.h"
#include "mcal_adc.h"
#include "mcal_timer.h"
#include "bsw_log.h"

#include <string.h>

/* ========== 编译期断言（用 #if/#error 实现，兼容 ARMCC 5.06 的 C90） ========== */

#if (((BSW_ADC_RINGBUF_CAPACITY) & ((BSW_ADC_RINGBUF_CAPACITY) - 1u)) != 0u)
#error "BSW_ADC_RINGBUF_CAPACITY must be a power of 2"
#endif

#if ((BSW_ADC_RINGBUF_WINDOW_LEN) > (BSW_ADC_RINGBUF_CAPACITY))
#error "BSW_ADC_RINGBUF_WINDOW_LEN must not exceed BSW_ADC_RINGBUF_CAPACITY"
#endif

#if ((BSW_ADC_RINGBUF_CAPACITY) > (2u * (BSW_ADC_RINGBUF_WINDOW_LEN)))
#error "snapshot() assumes CAPACITY <= 2*WINDOW (single wrap covered by 2 memcpy)"
#endif

/* ========== 私有状态 ========== */

/**
 * DMA 直接写入的目标 buffer。
 * C99 静态数组已天然 ≥ 4 字节对齐（uint16_t 数组 ≥ uint16_t 对齐，
 * 通常为 2 字节），但 DMA 要求 4 字节对齐，显式声明。
 */
static uint16_t s_buf[BSW_ADC_RINGBUF_CAPACITY] __attribute__((aligned(4)));

/**
 * 下一次 DMA 将写入的位置（指向"待写入"格，写后递增）。
 * 由 DMA 半传输/全传输回调更新，snapshot()/read() 关中断读取。
 */
static volatile uint16_t s_write_idx = 0;

/** 首次全传输完成标志（为后续上层"数据是否可信"判断保留）。 */
static volatile uint8_t  s_data_ready = 0;

/** 模块初始化标志。 */
static volatile uint8_t  s_inited = 0;

/** 采样运行标志（enable/disable 幂等控制）。 */
static volatile uint8_t  s_running = 0;

/* ========== DMA 回调（中断上下文） ========== */

static void _dma_half_cb(uint16_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
    /* DMA 已写到 s_buf[256]，下一个待写位置就是 256。 */
    s_write_idx = (uint16_t)(BSW_ADC_RINGBUF_CAPACITY >> 1);   /* = 256 */
}

static void _dma_cplt_cb(uint16_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
    /* DMA 已写完 s_buf[511] 并回环到 s_buf[0]，下一个待写位置 0。 */
    s_write_idx = 0;
    s_data_ready = 1;
}

/* ================================================================ */
/*                           公共接口实现                            */
/* ================================================================ */

bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_init(void)
{
    if (s_inited) {
        return BSW_ADC_RINGBUF_OK;
    }

    /* 先注册回调（DMA 启动由 enable() 统一控制，实现状态机按需启停） */
    if (mcal_adc_register_dma_half(MCAL_ADC_DEV1, _dma_half_cb) != MCAL_ADC_OK) {
        return BSW_ADC_RINGBUF_ERR_STATE;
    }
    if (mcal_adc_register_dma_cplt(MCAL_ADC_DEV1, _dma_cplt_cb) != MCAL_ADC_OK) {
        return BSW_ADC_RINGBUF_ERR_STATE;
    }

    s_write_idx  = 0;
    s_data_ready = 0;
    s_running    = 0;
    s_inited     = 1;

    bsw_log("[ADC_RINGBUF] init OK (hw not started, use enable() to start sampling)\r\n");

    return BSW_ADC_RINGBUF_OK;
}

bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_deinit(void)
{
    if (!s_inited) {
        return BSW_ADC_RINGBUF_OK;
    }

    /* 先停硬件（disable），再注销回调 */
    (void)mcal_timer_base_stop(MCAL_TIMER_TIM6);
    (void)mcal_adc_stop_dma(MCAL_ADC_DEV1);
    (void)mcal_adc_register_dma_half(MCAL_ADC_DEV1, (mcal_adc_dma_half_cb_t)NULL);
    (void)mcal_adc_register_dma_cplt(MCAL_ADC_DEV1, (mcal_adc_dma_cplt_cb_t)NULL);

    s_write_idx  = 0;
    s_data_ready = 0;
    s_running    = 0;
    s_inited     = 0;

    bsw_log("[ADC_RINGBUF] deinit OK\r\n");
    return BSW_ADC_RINGBUF_OK;
}

bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_enable(void)
{
    if (!s_inited) {
        return BSW_ADC_RINGBUF_ERR_STATE;
    }
    if (s_running) {
        return BSW_ADC_RINGBUF_OK;   /* 幂等 */
    }

    /* 清空 buffer，避免上电尖峰残留值污染首窗口（协议 §一） */
    memset(s_buf, 0, sizeof(s_buf));
    s_write_idx  = 0;
    s_data_ready = 0;

    /* 重启 DMA（复位 DMA 内部状态，确保从头写入 s_buf） */
    if (mcal_adc_start_dma(MCAL_ADC_DEV1, s_buf, BSW_ADC_RINGBUF_CAPACITY)
        != MCAL_ADC_OK) {
        return BSW_ADC_RINGBUF_ERR_STATE;
    }

    /* 启动 TIM6 触发源（10 kHz TRGO → ADC1） */
    mcal_timer_base_start(MCAL_TIMER_TIM6);

    s_running = 1;
    bsw_log("[ADC_RINGBUF] enabled (DMA + TIM6 started, fs=10 kHz)\r\n");
    return BSW_ADC_RINGBUF_OK;
}

bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_disable(void)
{
    if (!s_inited) {
        return BSW_ADC_RINGBUF_OK;
    }
    if (!s_running) {
        return BSW_ADC_RINGBUF_OK;   /* 幂等 */
    }

    /* 顺序：先停触发源（TIM6），再停搬运（DMA） */
    mcal_timer_base_stop(MCAL_TIMER_TIM6);
    (void)mcal_adc_stop_dma(MCAL_ADC_DEV1);

    s_running = 0;
    bsw_log("[ADC_RINGBUF] disabled (TIM6 + DMA stopped)\r\n");
    return BSW_ADC_RINGBUF_OK;
}

bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_snapshot(uint16_t *out)
{
    if (!s_inited) {
        return BSW_ADC_RINGBUF_ERR_STATE;
    }
    if (out == NULL) {
        return BSW_ADC_RINGBUF_ERR_PARAM;
    }

    /* 关中断拷贝临界区。
     * 拷贝量：最多 800 字节（单段）或 800 字节（双段）。
     * STM32L4 @ 80 MHz memcpy 实测 < 5 μs，对 10 kHz 采样节奏无感。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* write_idx 指向"下一个待写"位置 → 最新一点是 write_idx - 1
     * 起点（最旧一点）= write_idx - WINDOW_LEN */
    const uint16_t write_idx = s_write_idx;
    const uint16_t start     = (uint16_t)((write_idx
                                          + (uint16_t)BSW_ADC_RINGBUF_CAPACITY
                                          - (uint16_t)BSW_ADC_RINGBUF_WINDOW_LEN)
                                         & BSW_ADC_RINGBUF_CAPACITY_MASK);

    /* 拷贝 [start, start+WINDOW_LEN) 区间，按时间正序 → out[0..WINDOW_LEN)
     * CAPACITY=512, WINDOW_LEN=400：起点到 buffer 末尾最多 512，WINDOW_LEN=400，
     * 故最多跨一次 wrap，用最多两段 memcpy 即可。 */
    const uint32_t dist_to_end = (uint32_t)BSW_ADC_RINGBUF_CAPACITY - (uint32_t)start;
    const uint16_t first_chunk = (dist_to_end < (uint32_t)BSW_ADC_RINGBUF_WINDOW_LEN)
                                 ? (uint16_t)dist_to_end
                                 : (uint16_t)BSW_ADC_RINGBUF_WINDOW_LEN;

    memcpy(out,
           &s_buf[start],
           (size_t)first_chunk * sizeof(uint16_t));

    if (first_chunk < (uint16_t)BSW_ADC_RINGBUF_WINDOW_LEN) {
        const uint16_t second_chunk = (uint16_t)BSW_ADC_RINGBUF_WINDOW_LEN - first_chunk;
        memcpy(out + first_chunk,
               &s_buf[0],
               (size_t)second_chunk * sizeof(uint16_t));
    }

    __set_PRIMASK(primask);

    return BSW_ADC_RINGBUF_OK;
}

bsw_adc_ringbuf_ret_t bsw_adc_ringbuf_read(uint32_t index_back, uint16_t *out)
{
    if (!s_inited) {
        return BSW_ADC_RINGBUF_ERR_STATE;
    }
    if (out == NULL) {
        return BSW_ADC_RINGBUF_ERR_PARAM;
    }
    if (index_back >= BSW_ADC_RINGBUF_CAPACITY) {
        return BSW_ADC_RINGBUF_ERR_PARAM;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint16_t write_idx = s_write_idx;
    /* 最新一点在 write_idx - 1，往回数 index_back 个点 = write_idx - 1 - index_back */
    const uint16_t src = (uint16_t)((write_idx
                                    + (uint16_t)BSW_ADC_RINGBUF_CAPACITY
                                    - 1u
                                    - (uint16_t)index_back)
                                   & BSW_ADC_RINGBUF_CAPACITY_MASK);

    *out = s_buf[src];

    __set_PRIMASK(primask);

    return BSW_ADC_RINGBUF_OK;
}

uint32_t bsw_adc_ringbuf_calc_rms_sq(void)
{
    if (!s_inited) {
        return 0;
    }

    /* 栈上 800 字节：FSM run() 上下文，调频约 25 Hz，安全 */
    uint16_t snapshot[BSW_ADC_RINGBUF_WINDOW_LEN];
    if (bsw_adc_ringbuf_snapshot(snapshot) != BSW_ADC_RINGBUF_OK) {
        return 0;
    }

    /* 1) 估 DC（窗口均值） */
    uint64_t sum = 0;
    for (uint32_t i = 0; i < BSW_ADC_RINGBUF_WINDOW_LEN; ++i) {
        sum += snapshot[i];
    }
    uint32_t mean = (uint32_t)(sum / BSW_ADC_RINGBUF_WINDOW_LEN);

    /* 2) 去 DC 后均方值（不开方） */
    uint64_t sum_sq = 0;
    for (uint32_t i = 0; i < BSW_ADC_RINGBUF_WINDOW_LEN; ++i) {
        int32_t dev = (int32_t)snapshot[i] - (int32_t)mean;
        sum_sq += (uint64_t)((int64_t)dev * dev);
    }
    return (uint32_t)(sum_sq / BSW_ADC_RINGBUF_WINDOW_LEN);
}
