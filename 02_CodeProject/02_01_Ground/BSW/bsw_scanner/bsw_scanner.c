/* bsw_scanner.c */
#include "bsw_scanner.h"
#include "bsw_bfsk.h"
#include "mcal_timer.h"

static bsw_scan_state_t g_state = SCAN_STATE_IDLE;
static uint32_t        g_freq_idx = 0;
static uint32_t        g_tick_count = 0;

void bsw_scanner_init(void)
{
    g_state = SCAN_STATE_IDLE;
    g_freq_idx = 0;
}

void bsw_scanner_start_as_transmitter(void)
{
    g_state = SCAN_STATE_TRANSMIT;
    g_freq_idx = 0;
    bsw_bfsk_amp_enable();
}

void bsw_scanner_start_as_receiver(void)
{
    g_state = SCAN_STATE_LISTEN;
    g_freq_idx = 0;
}

void bsw_scanner_stop(void)
{
    g_state = SCAN_STATE_IDLE;
    bsw_bfsk_amp_disable();
}

bsw_scan_state_t bsw_scanner_get_state(void)
{
    return g_state;
}

bsw_scan_state_t bsw_scanner_tick(bsw_scan_result_t *result)
{
    (void)result;
    /* TODO: 扫频状态机
     * TRANSMIT: 切换到当前频点，发射 200ms，tick_count++
     *           tick_count >= 5（1s）后切换下一频点，g_freq_idx++
     * LISTEN:   切换到当前频点，监听 200ms
     *           检测到有效信号 → LOCKED
     *           tick_count >= 5 后切换下一频点
     * g_freq_idx >= SCAN_FREQ_COUNT → FAILED
     */
    return g_state;
}
