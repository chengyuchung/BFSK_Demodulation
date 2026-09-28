/**
 * @file    bsw_proto.c
 * @brief   BFSK 链路统一协议帧 - BSW 层实现
 *
 * 依据 01_docs/02_design/统一帧协议.md。
 *
 * 实现要点：
 *   - CRC-16/CCITT-FALSE：poly=0x1021, init=0xFFFF, refin/refout=false, xorout=0x0000
 *     按位计算，0 ROM 开销。
 *   - 转义：仅对 PAYLOAD 和 CRC16 字段做 0xFE / 01/02/03 替换；
 *     HEAD/ADDR/TYPE_INFO/SEQ/LEN/TAIL 直接原样写入。
 *   - pack 顺序：HEAD → ADDR → TYPE_INFO → SEQ → LEN → 转义 PAYLOAD
 *                → 转义 CRC16(低字节先) → TAIL
 *   - unpack：校验 HEAD → 读固定 4 字节 → 解转义 PAYLOAD+CRC16（按 LEN 切分）
 *             → 计算 CRC16 校验 → 校验 TAIL
 *
 * @dependency  bsw_proto.h
 */

#include "bsw_proto.h"

#include <string.h>     /* 当前未使用，预留 */

/* ========== ESC 转义标记字节（仅 pack/unpack 内部用）========== */
/*
 *  0x01 → 0xFF
 *  0x02 → 0x00
 *  0x03 → 0xFE
 */
#define PROTO_ESC_MARK_HEAD  0x01u
#define PROTO_ESC_MARK_TAIL  0x02u
#define PROTO_ESC_MARK_ESC   0x03u

/* ========== CRC-16/CCITT-FALSE（按位计算版）========== */

uint16_t proto_crc_calc(const uint8_t *data, uint32_t len)
{
    /* poly 0x1021, init 0xFFFF, MSB-first, refin=false, refout=false, xorout=0 */
    uint16_t crc = 0xFFFFu;
    if (data == NULL) {
        return crc;
    }
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ========== 地址 / TYPE_INFO 编码 ========== */

uint8_t proto_addr_pack(const proto_addr_t *a)
{
    if (a == NULL) {
        return 0;
    }
    return (uint8_t)(((a->dst & PROTO_ADDR_NIBBLE_MASK) << 4)
                    | (a->src & PROTO_ADDR_NIBBLE_MASK));
}

void proto_addr_unpack(uint8_t packed, proto_addr_t *a)
{
    if (a == NULL) {
        return;
    }
    a->dst = (uint8_t)((packed >> 4) & PROTO_ADDR_NIBBLE_MASK);
    a->src = (uint8_t)(packed & PROTO_ADDR_NIBBLE_MASK);
}

uint8_t proto_type_info_pack(const proto_type_info_t *t)
{
    if (t == NULL) {
        return 0;
    }
    return (uint8_t)(((uint8_t)t->type << 4)
                    | (t->msg_num & PROTO_ADDR_NIBBLE_MASK));
}

void proto_type_info_unpack(uint8_t packed, proto_type_info_t *t)
{
    if (t == NULL) {
        return;
    }
    t->type    = (proto_type_t)((packed >> 4) & PROTO_ADDR_NIBBLE_MASK);
    t->msg_num = (uint8_t)(packed & PROTO_ADDR_NIBBLE_MASK);
}

/* ========== 转义（内部）========== */

/**
 * @brief   单字节转义
 * @param[in]  in    原始字节
 * @param[out] out   输出（最多 2 字节）
 * @return 写入字节数（1 或 2）
 */
static uint32_t proto_escape_byte(uint8_t in, uint8_t *out)
{
    switch (in) {
        case PROTO_HEAD:                                    /* 0xFF */
            out[0] = PROTO_ESC;
            out[1] = PROTO_ESC_MARK_HEAD;
            return 2u;
        case PROTO_TAIL:                                    /* 0x00 */
            out[0] = PROTO_ESC;
            out[1] = PROTO_ESC_MARK_TAIL;
            return 2u;
        case PROTO_ESC:                                     /* 0xFE */
            out[0] = PROTO_ESC;
            out[1] = PROTO_ESC_MARK_ESC;
            return 2u;
        default:
            out[0] = in;
            return 1u;
    }
}

/**
 * @brief   单字节去转义（需已知当前字节是 ESC 之后的引导字节）
 * @retval PROTO_ERR_OK / PROTO_ERR_BAD_ESCAPE
 */
static proto_err_t proto_unescape_byte(uint8_t marker, uint8_t *out)
{
    switch (marker) {
        case PROTO_ESC_MARK_HEAD: *out = PROTO_HEAD; return PROTO_ERR_OK;
        case PROTO_ESC_MARK_TAIL: *out = PROTO_TAIL; return PROTO_ERR_OK;
        case PROTO_ESC_MARK_ESC:  *out = PROTO_ESC;  return PROTO_ERR_OK;
        default:                        return PROTO_ERR_BAD_ESCAPE;
    }
}

/* ========== 帧 pack ========== */

proto_err_t proto_frame_pack(uint8_t *buf, uint32_t buf_size,
                             const proto_frame_t *frame,
                             uint32_t *out_len)
{
    if (buf == NULL || frame == NULL || out_len == NULL) {
        return PROTO_ERR_BUF_TOO_SMALL;
    }
    if (!PROTO_ADDR_IS_VALID(frame->addr.dst) ||
        !PROTO_ADDR_IS_VALID(frame->addr.src)) {
        return PROTO_ERR_ADDR_OUT_OF_RANGE;
    }
    if (frame->payload_len > PROTO_PAYLOAD_MAX) {
        return PROTO_ERR_PAYLOAD_TOO_LONG;
    }

    /* 计算 CRC16（基于"原始未转义"字节，ADDR 起算，含 SEQ/LEN/PAYLOAD）*/
    uint8_t crc_buf[PROTO_FRAME_MAX];
    uint32_t crc_len = 0u;
    crc_buf[crc_len++] = proto_addr_pack(&frame->addr);
    crc_buf[crc_len++] = proto_type_info_pack(&frame->type_info);
    crc_buf[crc_len++] = frame->seq;
    crc_buf[crc_len++] = frame->payload_len;
    for (uint8_t i = 0; i < frame->payload_len; ++i) {
        crc_buf[crc_len++] = frame->payload[i];
    }
    uint16_t crc = proto_crc_calc(crc_buf, crc_len);

    /* 写入线缆字节流 */
    uint32_t idx = 0u;

    /* HEAD */
    if (idx >= buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    buf[idx++] = PROTO_HEAD;

    /* ADDR */
    if (idx >= buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    buf[idx++] = proto_addr_pack(&frame->addr);

    /* TYPE_INFO */
    if (idx >= buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    buf[idx++] = proto_type_info_pack(&frame->type_info);

    /* SEQ */
    if (idx >= buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    buf[idx++] = frame->seq;

    /* LEN（原始 PAYLOAD 字节数，不含转义）*/
    if (idx >= buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    buf[idx++] = frame->payload_len;

    /* PAYLOAD（转义后）*/
    for (uint8_t i = 0; i < frame->payload_len; ++i) {
        if (idx + PROTO_CRC16_SIZE > buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
        idx += proto_escape_byte(frame->payload[i], &buf[idx]);
    }

    /* CRC16（低字节先，转义后）*/
    uint8_t crc_l = (uint8_t)(crc & 0xFFu);
    uint8_t crc_h = (uint8_t)((crc >> 8) & 0xFFu);
    if (idx + PROTO_CRC16_SIZE > buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    idx += proto_escape_byte(crc_l, &buf[idx]);
    if (idx + PROTO_CRC16_SIZE > buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    idx += proto_escape_byte(crc_h, &buf[idx]);

    /* TAIL */
    if (idx >= buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    buf[idx++] = PROTO_TAIL;

    *out_len = idx;
    return PROTO_ERR_OK;
}

/* ========== 帧 unpack ========== */

proto_err_t proto_frame_unpack(const uint8_t *buf, uint32_t buf_len,
                               proto_frame_t *frame)
{
    if (buf == NULL || frame == NULL) {
        return PROTO_ERR_BUF_TOO_SMALL;
    }
    if (buf_len < PROTO_FRAME_MIN) {
        return PROTO_ERR_TOO_SHORT;
    }

    /* 清零 frame，避免残留数据 */
    frame->payload_len = 0u;
    frame->payload[0]  = 0u;   /* 满足静态分析器 */

    uint32_t idx = 0u;

    /* HEAD */
    if (buf[idx] != PROTO_HEAD) {
        return PROTO_ERR_BAD_HEAD;
    }
    idx++;

    /* ADDR / TYPE_INFO / SEQ / LEN（这 4 字节不转义，连续读 4 字节）*/
    if (idx + 4u > buf_len) {
        return PROTO_ERR_LEN_MISMATCH;
    }
    uint8_t addr_b       = buf[idx++];
    uint8_t type_info_b  = buf[idx++];
    uint8_t seq_b        = buf[idx++];
    uint8_t len_b        = buf[idx++];

    proto_addr_unpack(addr_b, &frame->addr);
    proto_type_info_unpack(type_info_b, &frame->type_info);
    frame->seq         = seq_b;
    uint8_t need_len   = len_b;

    /* TYPE 合法性校验 */
    if (!PROTO_TYPE_IS_VALID(frame->type_info.type)) {
        return PROTO_ERR_UNKNOWN_TYPE;
    }
    if (!PROTO_MSG_NUM_IN_RANGE(frame->type_info.msg_num)) {
        return PROTO_ERR_UNKNOWN_MSG;
    }

    /* 扫描到 TAIL，确定 PAYLOAD+CRC16 字段范围 */
    uint32_t scan = idx;
    while (scan < buf_len && buf[scan] != PROTO_TAIL) {
        scan++;
    }
    if (scan >= buf_len) {
        return PROTO_ERR_BAD_TAIL;   /* 找不到 TAIL */
    }
    uint32_t esc_field_end = scan;   /* [idx, esc_field_end) = PAYLOAD+CRC16 转义后区间 */

    /* 解转义 PAYLOAD（先填 need_len 字节）和 CRC16（再读 2 字节）*/
    uint32_t payload_filled = 0u;
    uint8_t  crc_decoded[2]  = { 0u, 0u };
    uint32_t crc_filled      = 0u;
    uint32_t s               = idx;

    while (s < esc_field_end) {
        uint8_t decoded;
        if (buf[s] == PROTO_ESC) {
            if (s + 1u >= esc_field_end) {
                return PROTO_ERR_BAD_ESCAPE;
            }
            proto_err_t e = proto_unescape_byte(buf[s + 1u], &decoded);
            if (e != PROTO_ERR_OK) {
                return e;
            }
            s += 2u;
        } else {
            decoded = buf[s++];
        }

        if (payload_filled < need_len) {
            frame->payload[payload_filled++] = decoded;
        } else if (crc_filled < PROTO_CRC16_SIZE) {
            crc_decoded[crc_filled++] = decoded;
        } else {
            return PROTO_ERR_PAYLOAD_TOO_LONG;
        }
    }

    if (payload_filled != need_len) {
        return PROTO_ERR_LEN_MISMATCH;
    }
    if (crc_filled != PROTO_CRC16_SIZE) {
        return PROTO_ERR_LEN_MISMATCH;
    }
    frame->payload_len = (uint8_t)payload_filled;

    /* 校验 CRC16（小端：低字节先）*/
    uint16_t received_crc = (uint16_t)((uint16_t)crc_decoded[0]
                                      | ((uint16_t)crc_decoded[1] << 8));

    /* 重新构造 CRC 覆盖的"原始字节"序列：ADDR + TYPE_INFO + SEQ + LEN + PAYLOAD */
    uint8_t crc_buf[PROTO_FRAME_MAX];
    uint32_t crc_len = 0u;
    crc_buf[crc_len++] = addr_b;
    crc_buf[crc_len++] = type_info_b;
    crc_buf[crc_len++] = seq_b;
    crc_buf[crc_len++] = len_b;
    for (uint32_t i = 0; i < payload_filled; ++i) {
        crc_buf[crc_len++] = frame->payload[i];
    }
    uint16_t computed_crc = proto_crc_calc(crc_buf, crc_len);

    if (computed_crc != received_crc) {
        return PROTO_ERR_BAD_CRC;
    }

    /* TAIL 校验（scan 已经在前面确认是 TAIL 了，这里再次显式）*/
    if (buf[scan] != PROTO_TAIL) {
        return PROTO_ERR_BAD_TAIL;
    }

    return PROTO_ERR_OK;
}

/* ========== 帧 validate（轻量）========== */

proto_err_t proto_frame_validate(const uint8_t *buf, uint32_t buf_len)
{
    if (buf == NULL) {
        return PROTO_ERR_BUF_TOO_SMALL;
    }
    if (buf_len < PROTO_FRAME_MIN) {
        return PROTO_ERR_TOO_SHORT;
    }
    if (buf[0] != PROTO_HEAD) {
        return PROTO_ERR_BAD_HEAD;
    }
    if (buf[buf_len - 1u] != PROTO_TAIL) {
        return PROTO_ERR_BAD_TAIL;
    }

    /* TYPE_INFO 大类合法性（buf[2]）*/
    uint8_t type_nibble = (uint8_t)((buf[2] >> 4) & PROTO_ADDR_NIBBLE_MASK);
    if (!PROTO_TYPE_IS_VALID((proto_type_t)type_nibble)) {
        return PROTO_ERR_UNKNOWN_TYPE;
    }

    /* LEN 范围（buf[4]）*/
    uint8_t len = buf[4];
    if (len > PROTO_PAYLOAD_MAX) {
        return PROTO_ERR_PAYLOAD_TOO_LONG;
    }

    /* 注意：完整 CRC 校验交给 proto_frame_unpack */
    return PROTO_ERR_OK;
}

/* ========== 接收侧状态机（Bit 流 → 帧对象）========== */

/**
 * @brief   接收上下文（内部静态）
 */
typedef struct {
    proto_rx_state_t state;
    uint8_t rx_buf[PROTO_FRAME_MAX];   /**< 字节缓冲（接收线缆字节流） */
    uint16_t rx_len;                   /**< 已接收字节数 */
    proto_frame_t frame;               /**< 解析结果 */
    proto_err_t last_error;            /**< 最后一次错误码 */
    uint8_t local_addr;                /**< 本机地址（用于早期过滤）*/
} proto_rx_ctx_t;

static proto_rx_ctx_t s_rx_ctx;

/**
 * @brief   Bit 流转字节（LSB first：bit_buf[0] 的 bit0 是第一个 bit）
 * @param   bit_buf     bit 缓冲
 * @param   start_bit   起始 bit 索引
 * @return  转换后的字节（bit 0~7 → byte LSB~MSB）
 */
static uint8_t _proto_bits_to_byte(const uint8_t *bit_buf, uint16_t start_bit)
{
    uint8_t byte = 0u;
    for (uint8_t i = 0; i < 8; ++i) {
        uint16_t bit_idx = start_bit + i;
        uint16_t byte_idx = bit_idx / BITS_PER_BYTE;
        uint8_t  bit_pos  = bit_idx % BITS_PER_BYTE;
        
        if (bit_buf[byte_idx] & (1u << bit_pos)) {
            byte |= (1u << i);  /* LSB first */
        }
    }
    return byte;
}

proto_err_t proto_rx_init(uint8_t local_addr)
{
    s_rx_ctx.state = PROTO_RX_IDLE;
    s_rx_ctx.rx_len = 0u;
    s_rx_ctx.last_error = PROTO_ERR_OK;
    s_rx_ctx.local_addr = local_addr & PROTO_ADDR_NIBBLE_MASK;  /* 限制为 4 位 */
    return PROTO_ERR_OK;
}

proto_rx_state_t proto_rx_get_state(void)
{
    return s_rx_ctx.state;
}

proto_err_t proto_rx_get_last_error(void)
{
    return s_rx_ctx.last_error;
}

proto_err_t proto_rx_feed_bits(const uint8_t *bit_buf, uint16_t bit_count)
{
    if (bit_buf == NULL) {
        return PROTO_ERR_BUF_TOO_SMALL;
    }
    if ((bit_count == 0) || ((bit_count % BITS_PER_BYTE) != 0)) {
        return PROTO_ERR_LEN_MISMATCH;  /* bit 数必须是 8 的倍数 */
    }
    
    uint16_t byte_count = bit_count / BITS_PER_BYTE;
    uint8_t bytes[PROTO_FRAME_MAX];
    
    /* 步骤 1：bit 流 → 字节流 */
    if (byte_count > PROTO_FRAME_MAX) {
        byte_count = PROTO_FRAME_MAX;  /* 截断保护 */
    }
    
    for (uint16_t i = 0; i < byte_count; ++i) {
        bytes[i] = _proto_bits_to_byte(bit_buf, i * 8u);
    }
    
    /* 步骤 2：状态机处理 */
    for (uint16_t i = 0; i < byte_count; ++i) {
        uint8_t byte = bytes[i];
        
        switch (s_rx_ctx.state) {
            case PROTO_RX_IDLE:
            case PROTO_RX_SYNCING:
                /* 搜索帧头 0xFF */
                if (byte == PROTO_HEAD) {
                    s_rx_ctx.state = PROTO_RX_RECEIVING;
                    s_rx_ctx.rx_buf[0] = PROTO_HEAD;
                    s_rx_ctx.rx_len = 1u;
                }
                break;
                
            case PROTO_RX_RECEIVING:
                /* 累积字节 */
                if (s_rx_ctx.rx_len < PROTO_FRAME_MAX) {
                    s_rx_ctx.rx_buf[s_rx_ctx.rx_len++] = byte;
                    
                    /* 早期过滤：检查 ADDR 字段（第 2 字节，索引 1）*/
                    if (s_rx_ctx.rx_len == PROTO_OFFSET_ADDR) {  /* HEAD 后第 1 字节 = ADDR */
                        /* ADDR = DST(高 4 位) | SRC(低 4 位) */
                        uint8_t dst_addr = (byte >> 4) & PROTO_ADDR_NIBBLE_MASK;
                        
                        /* 如果目的地址不是本机，丢弃此帧 */
                        if (dst_addr != s_rx_ctx.local_addr) {
                            /* 丢弃：复位到 IDLE，继续搜索下一个 HEAD */
                            s_rx_ctx.state = PROTO_RX_IDLE;
                            s_rx_ctx.rx_len = 0u;
                            /* 继续处理后续字节（可能包含新的 HEAD）*/
                            continue;
                        }
                    }
                    
                    /* 检测到 TAIL → 尝试解析 */
                    if (byte == PROTO_TAIL) {
                        proto_err_t err = proto_frame_unpack(s_rx_ctx.rx_buf,
                                                             s_rx_ctx.rx_len,
                                                             &s_rx_ctx.frame);
                        if (err == PROTO_ERR_OK) {
                            s_rx_ctx.state = PROTO_RX_DONE;
                            s_rx_ctx.last_error = PROTO_ERR_OK;
                            return PROTO_ERR_OK;
                        } else {
                            /* 解析失败（CRC 错误等）→ 复位，继续搜索下一个 HEAD */
                            s_rx_ctx.state = PROTO_RX_IDLE;
                            s_rx_ctx.last_error = err;
                            s_rx_ctx.rx_len = 0u;
                            /* 不返回错误，继续接收（可能有下一帧）*/
                        }
                    }
                } else {
                    /* 缓冲溢出 → 复位 */
                    s_rx_ctx.state = PROTO_RX_IDLE;
                    s_rx_ctx.last_error = PROTO_ERR_PAYLOAD_TOO_LONG;
                    s_rx_ctx.rx_len = 0u;
                }
                break;
                
            case PROTO_RX_DONE:
            case PROTO_RX_ERROR:
                /* 已完成或出错，不再接收（需调用 proto_rx_take_frame 消费）*/
                break;
        }
    }
    
    return PROTO_ERR_OK;
}

proto_err_t proto_rx_take_frame(proto_frame_t *out)
{
    if (out == NULL) {
        return PROTO_ERR_BUF_TOO_SMALL;
    }
    
    if (s_rx_ctx.state != PROTO_RX_DONE) {
        return PROTO_ERR_LEN_MISMATCH;  /* 状态不对 */
    }
    
    /* 拷贝帧 */
    *out = s_rx_ctx.frame;
    
    /* 复位状态机，准备接收下一帧 */
    s_rx_ctx.state = PROTO_RX_IDLE;
    s_rx_ctx.rx_len = 0u;
    
    return PROTO_ERR_OK;
}
