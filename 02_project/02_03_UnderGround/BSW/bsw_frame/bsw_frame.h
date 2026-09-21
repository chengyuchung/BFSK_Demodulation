/**
 * @file    bsw_frame.h
 * @brief   帧解析与组帧 - BSW 层
 * @note    处理字节填充、CRC-16、转义/还原、解析状态机
 *
 * @dependency  bsw_crc
 */

#ifndef BSW_FRAME_H
#define BSW_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bsw_crc.h"

/* ========== 宏定义 ========== */
#define FRAME_FLAG       0x7E    /* 帧标志字节 */
#define FRAME_ESC       0x7D    /* 转义前缀 */
#define FRAME_STUFF_XOR  0x20    /* 转义异或值 */

#define FRAME_MAX_PAYLOAD  240   /* 最大载荷长度 */

/* ========== 类型定义 ========== */

/* 帧解析状态机 */
typedef enum {
    FRAME_STATE_IDLE,       /* 等待 FLAG */
    FRAME_STATE_DATA,       /* 接收数据 */
    FRAME_STATE_ESCAPE,     /* 转义处理 */
    FRAME_STATE_COMPLETE     /* 完整帧 */
} frame_state_t;

/* 帧结构 */
typedef struct {
    uint8_t  dst_addr;      /* 目标地址 */
    uint8_t  src_addr;      /* 源地址 */
    uint8_t  msg_type;      /* 消息类型 */
    uint8_t  control;       /* 控制字段 */
    uint8_t  seq;           /* 序号 */
    uint8_t  payload[FRAME_MAX_PAYLOAD];
    uint16_t payload_len;
    uint16_t crc_recv;      /* 接收到的 CRC */
    int      is_valid;      /* 解析是否完整 */
} frame_t;

/* ========== 函数声明 ========== */

/* ---- 组帧 ---- */

/**
 * @brief   打包一帧数据（加 CRC + 字节填充 + FLAG）
 * @param   dst     目标地址
 * @param   src     源地址
 * @param   type    消息类型
 * @param   ctrl    控制字段
 * @param   seq     序号
 * @param   payload 载荷数据
 * @param   len     载荷长度
 * @param   out_buf 输出缓冲区
 * @param   out_len 输出长度
 */
void bsw_frame_pack(
    uint8_t dst, uint8_t src, uint8_t type, uint8_t ctrl, uint8_t seq,
    const uint8_t *payload, uint16_t len,
    uint8_t *out_buf, uint16_t *out_len
);

/**
 * @brief   CRC 计算（内部用）
 */
static inline uint16_t bsw_frame_calc_crc16(const uint8_t *data, uint16_t len)
{
    return bsw_crc16(data, len);
}

/* ---- 解帧 ---- */

/**
 * @brief   初始化帧解析器
 */
void bsw_frame_init(void);

/**
 * @brief   逐字节输入，解帧状态机
 * @param   byte    输入字节
 * @retval  指向完整帧的指针（NULL 表示帧未完整）
 * @note    返回的 frame_t 缓存在解析器内部，下次调用会被覆盖
 */
frame_t *bsw_frame_input_byte(uint8_t byte);

/* ========== 回调 ========== */
__weak void bsw_frame_received_callback(const frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* BSW_FRAME_H */
