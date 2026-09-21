/* bsw_frame.c */
#include "bsw_frame.h"

static frame_state_t g_state = FRAME_STATE_IDLE;
static frame_t       g_frame;
static uint8_t       g_buf[FRAME_MAX_PAYLOAD + 8];
static uint16_t      g_buf_idx = 0;

void bsw_frame_init(void)
{
    g_state = FRAME_STATE_IDLE;
    g_buf_idx = 0;
}

void bsw_frame_pack(
    uint8_t dst, uint8_t src, uint8_t type, uint8_t ctrl, uint8_t seq,
    const uint8_t *payload, uint16_t len,
    uint8_t *out_buf, uint16_t *out_len)
{
    (void)dst;(void)src;(void)type;(void)ctrl;(void)seq;
    (void)payload;(void)len;(void)out_buf;(void)out_len;
    /* TODO:
     * 1. 填充帧头：dst/src/type/ctrl/seq
     * 2. 追加 payload
     * 3. 计算 CRC-16
     * 4. 字节填充（0x7E → 0x7D 0x5E，0x7D → 0x7D 0x5D）
     * 5. 头尾加 FLAG 0x7E
     */
}

frame_t *bsw_frame_input_byte(uint8_t byte)
{
    (void)byte;
    g_frame.is_valid = 0;
    /* TODO: 实现帧解析状态机
     * IDLE: 收到 0x7E 切换到 DATA
     * DATA: 读到 0x7D 则进入 ESCAPE
     * ESCAPE: 异或还原，跳回 DATA
     * 读到 0x7E: 计算 CRC，去转义，解析帧头，返回 &g_frame
     */
    return NULL;
}

__weak void bsw_frame_received_callback(const frame_t *frame)
{
    (void)frame;
}
