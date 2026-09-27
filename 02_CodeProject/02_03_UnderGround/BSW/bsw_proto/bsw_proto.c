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
        if (idx + 2u > buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
        idx += proto_escape_byte(frame->payload[i], &buf[idx]);
    }

    /* CRC16（低字节先，转义后）*/
    uint8_t crc_l = (uint8_t)(crc & 0xFFu);
    uint8_t crc_h = (uint8_t)((crc >> 8) & 0xFFu);
    if (idx + 2u > buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
    idx += proto_escape_byte(crc_l, &buf[idx]);
    if (idx + 2u > buf_size) { return PROTO_ERR_BUF_TOO_SMALL; }
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
        } else if (crc_filled < 2u) {
            crc_decoded[crc_filled++] = decoded;
        } else {
            return PROTO_ERR_PAYLOAD_TOO_LONG;
        }
    }

    if (payload_filled != need_len) {
        return PROTO_ERR_LEN_MISMATCH;
    }
    if (crc_filled != 2u) {
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
