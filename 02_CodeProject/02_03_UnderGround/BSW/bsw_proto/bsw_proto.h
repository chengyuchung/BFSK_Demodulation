/**
 * @file    bsw_proto.h
 * @brief   BFSK 链路统一协议帧 - BSW 层
 *
 * 依据 01_docs/02_design/统一帧协议.md 实现。
 * 是 BFSK 链路所有节点共享的"协议宪法"，井下 / 中继 / 井上 都按此编码。
 *
 * ==================== 帧格式（线缆上）====================
 *
 *      ┌──────┬──────────┬────────────┬──────┬──────┬──────────────┬──────────┬──────┐
 *      │ HEAD │  ADDR    │ TYPE_INFO  │ SEQ  │ LEN  │   PAYLOAD    │  CRC16   │ TAIL │
 *      │ 0xFF │ DST<<4|SRC│ TYPE<<4|MN│0..255│0..LEN│   LEN bytes  │ L,H 字节 │ 0x00 │
 *      │ 1B   │   1B     │    1B      │ 1B   │ 1B   │  转义后数据  │ 转义后 2B│  1B  │
 *      └──────┴──────────┴────────────┴──────┴──────┴──────────────┴──────────┴──────┘
 *
 *      - HEAD       = 0xFF（固定，起始标记，不转义）
 *      - ADDR       = DST(4 bit) << 4 | SRC(4 bit)
 *      - TYPE_INFO  = TYPE(4 bit) << 4 | MSG_NUM(4 bit)
 *      - SEQ        = 帧序号 0~255 循环
 *      - LEN        = PAYLOAD 原始字节数（不是转义后）
 *      - PAYLOAD    = 载荷（按 §3 转义规则）
 *      - CRC16      = CRC-16/CCITT-FALSE，低字节先，按 §7 覆盖范围计算
 *      - TAIL       = 0x00（固定，结束标记，不转义）
 *
 * ==================== 字节序 ====================
 *
 *      所有多字节字段小端（little-endian）。CRC16 低字节先发。
 *
 * ==================== 转义（§3）====================
 *
 *      仅 PAYLOAD 和 CRC16 字段生效，HEAD/ADDR/TYPE_INFO/SEQ/LEN/TAIL 不转义。
 *
 *          原始 0xFF  →  0xFE 0x01
 *          原始 0x00  →  0xFE 0x02
 *          原始 0xFE  →  0xFE 0x03   （ESC 自身也要转义）
 *
 * ==================== CRC 覆盖范围（§7）====================
 *
 *      从 ADDR（第 1 字节）到 PAYLOAD 末尾的"原始字节"。
 *      - 包含：ADDR、TYPE_INFO、SEQ、LEN、PAYLOAD（原始未转义）
 *      - 不包含：HEAD、CRC16 自身、TAIL
 *
 * ==================== 帧类型（TYPE，4 位）====================
 *
 *      0x0  QUERY     查询帧（请求对方上报数据）
 *      0x1  CONTROL   控制帧（设置参数/状态）
 *      0x2  ASK       询问帧（传输前握手）
 *      0x3  REPLY     回复帧（应答 ASK 或业务）
 *      0x4  ALARM     警报帧（主动告警）
 *      0x5~0xC  保留
 *      0xD  SCAN      扫频/锁频专用
 *      0xE~0xF  保留
 *
 *      每种 TYPE 下独立计数 MSG_NUM（4 位，0~15）。
 *
 * ==================== 模块依赖 ====================
 *
 *      应用层    app_node_fsm    →  构造帧 / 消费解调事件
 *      链路层    bsw_bfsk_demod  →  字节流来源（接收）/ 字节流去向（发送）
 *      协议层    bsw_proto  ◀── 本模块：字节流 ↔ 帧对象
 *      地址表    bsw_node_id     →  地址常量复用
 *
 * @todo    可选：CRC-16 查表实现（节省 CPU）；当前为按位计算，~8×CPU 换 0 ROM
 */

#ifndef BSW_PROTO_H
#define BSW_PROTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "bsw_node_id.h"   /* BSW_NODE_ADDR_* 地址常量 */

/* ========== 帧边界常量 ========== */

#define PROTO_HEAD              0xFFu   /* 帧起始固定字节 */
#define PROTO_TAIL              0x00u   /* 帧结束固定字节 */
#define PROTO_ESC               0xFEu   /* 转义引导字节 */

/* ========== 帧长限制 ========== */

#define PROTO_PAYLOAD_MAX       32u     /* 单帧 PAYLOAD 最大字节数（原始，未转义）*/
#define PROTO_ADDR_NIBBLE_MASK  0x0Fu   /* 4 位地址掩码 */
#define PROTO_SEQ_MAX           255u

/* ========== 协议字段大小常量 ========== */
#define PROTO_CRC16_SIZE        2u      /* CRC16 字段字节数 */
#define PROTO_HEADER_SIZE       5u      /* HEAD + ADDR + TYPE_INFO + SEQ + LEN */
#define PROTO_FIXED_SIZE        (PROTO_HEADER_SIZE + PROTO_CRC16_SIZE + 1u)  /* +1 为 TAIL */
#define BITS_PER_BYTE           8u      /* 每字节位数 */

/* ========== 协议字段偏移量（用于早期过滤）========== */
#define PROTO_OFFSET_ADDR       2u      /* rx_len == 2 时完整接收到 ADDR 字段（索引 1）*/

/** 整帧线缆字节上限（最坏情况：PAYLOAD 和 CRC16 每个字节都被转义成 2 字节）
 *      HEAD(1) + ADDR(1) + TYPE_INFO(1) + SEQ(1) + LEN(1)
 *    + PAYLOAD 转义后(2 * PROTO_PAYLOAD_MAX = 64)
 *    + CRC16 转义后(4)
 *    + TAIL(1)
 *  = 74，按 8 对齐到 80 */
#define PROTO_FRAME_MAX         80u

/** 整帧最小字节数（HEAD + ADDR + TYPE_INFO + SEQ + LEN=0 + CRC16 + TAIL = 8）*/
#define PROTO_FRAME_MIN         8u

/* ========== TYPE（4 位大类）========== */

typedef enum {
    PROTO_TYPE_QUERY      = 0x0,
    PROTO_TYPE_CONTROL    = 0x1,
    PROTO_TYPE_ASK        = 0x2,
    PROTO_TYPE_REPLY      = 0x3,
    PROTO_TYPE_ALARM      = 0x4,
    PROTO_TYPE_SCAN       = 0xD,
    PROTO_TYPE_RESERVED   = 0xF,
} proto_type_t;

/* ========== MSG_NUM（每种 TYPE 独立计数，4 位）========== */

/** QUERY - 查询帧（§6.1） */
typedef enum {
    PROTO_QUERY_TEMP_PRESS = 0x0,   /**< 触发对方上报温压 */
    PROTO_QUERY_BATTERY    = 0x1,   /**< 触发对方上报电池电压/电量 */
    PROTO_QUERY_FAULT      = 0x2,   /**< 触发对方上报传感器故障 */
    PROTO_QUERY_VERSION    = 0x3,   /**< 触发对方上报软硬件版本号（日期格式）*/
} proto_query_msg_t;

/** CONTROL - 控制帧（§6.2） */
typedef enum {
    PROTO_CONTROL_SET_SLEEP_DUR    = 0x0,   /**< 载荷 [sleep_sec(2)] 设置下次休眠时长 */
    PROTO_CONTROL_SLEEP            = 0x1,   /**< (空) 立刻进入休眠 */
    PROTO_CONTROL_WAKE             = 0x2,   /**< (空) 强制唤醒（中继/井上） */
    PROTO_CONTROL_SET_TEMP_THRESH  = 0x3,   /**< 载荷 [temp_low(2)][temp_high(2)] 0.01°C */
    PROTO_CONTROL_SET_PRESS_THRESH = 0x4,   /**< 载荷 [press_low(2)][press_high(2)] 0.01MPa */
} proto_control_msg_t;

/** ASK - 询问帧（§6.3） */
typedef enum {
    PROTO_ASK_IDLE_CONFIRM = 0x0,   /**< (空) 你能接收数据吗？ */
} proto_ask_msg_t;

/** REPLY - 回复帧（§6.4） */
typedef enum {
    PROTO_REPLY_FREQ_PAIR = 0x0,    /**< 载荷 [f0_hz(2)][f1_hz(2)] 频点确认回复（扫频后建链） */
    PROTO_REPLY_READY     = 0x1,    /**< (空) ASK 应答：当前有空 */
    PROTO_REPLY_ACK       = 0x2,    /**< (空) 业务帧接收成功 */
    PROTO_REPLY_NACK      = 0x3,    /**< 载荷 [err_code(1)] 接收失败，详见 proto_nack_code_t */
    PROTO_REPLY_RETRY_REQ = 0x4,    /**< (空) 请求重传 */
    PROTO_REPLY_VERSION   = 0x5,    /**< 载荷 [sw_ver(4)][hw_ver(4)] 版本号回复（日期格式 YYYYMMDD）*/
} proto_reply_msg_t;

/** ALARM - 警报帧（§6.5） */
typedef enum {
    PROTO_ALARM_LOW_BATT    = 0x0,  /**< 载荷 [batt_pct(1)] 电量低 */
    PROTO_ALARM_TEMP_OVER   = 0x1,  /**< 载荷 [current_temp(2)][limit_type(1)] 温度超限 */
    PROTO_ALARM_PRESS_OVER  = 0x2,  /**< 载荷 [current_press(2)][limit_type(1)] 压力超限 */
    PROTO_ALARM_SENSOR_FAULT = 0x3, /**< 载荷 [fault_code(1)][fault_data(2)] 传感器故障 */
    PROTO_ALARM_LINK_FAIL   = 0x4,  /**< 载荷 [retry_count(1)] 链路失败 */
} proto_alarm_msg_t;

/** SCAN - 扫频专用（具体子表待定，§未列出，预留） */
typedef enum {
    PROTO_SCAN_SWEEP_START  = 0x0,   /**< 井上发起的扫频会话 */
    PROTO_SCAN_SWEEP_DONE   = 0x1,   /**< 扫频会话结束标记 */
} proto_scan_msg_t;

/* ========== 错误码（API 返回值）========== */

typedef enum {
    PROTO_ERR_OK                = 0x00,
    PROTO_ERR_BAD_HEAD          = 0x01,  /**< HEAD ≠ 0xFF */
    PROTO_ERR_BAD_TAIL          = 0x02,  /**< TAIL ≠ 0x00 或未找到 */
    PROTO_ERR_BAD_CRC           = 0x03,  /**< CRC16 校验失败 */
    PROTO_ERR_LEN_MISMATCH      = 0x04,  /**< LEN 与解出的 PAYLOAD 字节数不匹配 */
    PROTO_ERR_BAD_ESCAPE        = 0x05,  /**< 转义序列非法（ESC 后续字节不是 01/02/03）*/
    PROTO_ERR_UNKNOWN_TYPE      = 0x06,  /**< TYPE 字段未知 */
    PROTO_ERR_UNKNOWN_MSG       = 0x07,  /**< MSG_NUM 字段未知（TYPE 已知）*/
    PROTO_ERR_ADDR_OUT_OF_RANGE = 0x08,  /**< DST/SRC 字段 > 0x0F */
    PROTO_ERR_PAYLOAD_TOO_LONG  = 0x09,  /**< LEN 超过 PROTO_PAYLOAD_MAX 或解出的字节数超限 */
    PROTO_ERR_BUF_TOO_SMALL     = 0x0A,  /**< 缓冲过小或入参 NULL */
    PROTO_ERR_TOO_SHORT         = 0x0B,  /**< 整帧字节数 < PROTO_FRAME_MIN */
} proto_err_t;

/* ========== NACK err_code（协议 §6.4 列出，独立于 API 错误码）========== */

typedef enum {
    PROTO_NACK_CRC_FAIL         = 0x01,  /**< CRC 校验失败 */
    PROTO_NACK_LEN_BAD          = 0x02,  /**< 长度异常 */
    PROTO_NACK_TYPE_UNKNOWN     = 0x03,  /**< TYPE/MSG_NUM 未知 */
    PROTO_NACK_ADDR_MISMATCH    = 0x04,  /**< 目标地址不匹配 */
    PROTO_NACK_SEQ_DUP          = 0x05,  /**< SEQ 序号重复/过期 */
    PROTO_NACK_BUSY             = 0x06,  /**< 业务忙（正在处理其他事务）*/
    PROTO_NACK_PARAM_OUT_RANGE  = 0x07,  /**< 参数越界 */
} proto_nack_code_t;

/* ========== ALARM sensor fault_code（协议 §6.5）========== */

typedef enum {
    PROTO_FAULT_PT100_OPEN        = 0x01,    /**< PT100 断路 */
    PROTO_FAULT_PT100_SHORT       = 0x02,    /**< PT100 短路 */
    PROTO_FAULT_PRESS_NO_RESPOND  = 0x03,    /**< 压力传感器无响应 */
    PROTO_FAULT_ADC_BAD           = 0x04,    /**< ADC 异常 */
    PROTO_FAULT_RTC_BAD           = 0x05,    /**< RTC 异常 */
} proto_alarm_fault_t;

/** ALARM limit_type */
typedef enum {
    PROTO_LIMIT_BELOW_LOW  = 0x01,
    PROTO_LIMIT_ABOVE_HIGH = 0x02,
} proto_alarm_limit_t;

/* ========== 版本号载荷结构（日期格式 YYYYMMDD）========== */

/**
 * @brief   版本号载荷（用于 PROTO_REPLY_VERSION）
 * 
 * 版本号使用日期格式：YYYYMMDD
 *   - 例如：20261007 表示 2026年10月7日编译的版本
 *   - 存储为 4 字节（uint32_t），小端传输
 * 
 * 载荷格式（共 8 字节）：
 *   [sw_version_date(4)] + [hw_version_date(4)]
 * 
 * @example
 *   软件版本：20261007（2026年10月7日）
 *   硬件版本：20260801（2026年8月1日）
 */
typedef struct {
    uint32_t sw_version_date;  /**< 软件版本号（日期格式 YYYYMMDD）*/
    uint32_t hw_version_date;  /**< 硬件版本号（日期格式 YYYYMMDD）*/
} proto_version_payload_t;

/** 版本号载荷字节数 */
#define PROTO_VERSION_PAYLOAD_SIZE  8u

/* ========== 解码后的帧对象 ========== */

/** 地址对（解码/编码前的人类可读形式）*/
typedef struct {
    uint8_t dst;       /**< 目的地址 0..15（见 bsw_node_id.h） */
    uint8_t src;       /**< 源地址   0..15 */
} proto_addr_t;

/** 类型/子类型对 */
typedef struct {
    proto_type_t type; /**< 大类 */
    uint8_t      msg_num; /**< 子类型 0..15，对应 proto_*_msg_t 枚举 */
} proto_type_info_t;

/** 完整帧对象（解码后）*/
typedef struct {
    proto_addr_t      addr;
    proto_type_info_t type_info;
    uint8_t           seq;
    uint8_t           payload_len;
    uint8_t           payload[PROTO_PAYLOAD_MAX];
} proto_frame_t;

/* ========== 地址/类型编解码 ========== */

/**
 * @brief   把 DST/SRC 4 位对打包到 1 字节
 * @return  (dst << 4) | (src & 0x0F)
 */
uint8_t proto_addr_pack(const proto_addr_t *a);

/**
 * @brief   从 1 字节 ADDR 解码出 DST/SRC
 */
void proto_addr_unpack(uint8_t packed, proto_addr_t *a);

/**
 * @brief   把 TYPE/MSG_NUM 4 位对打包到 1 字节
 * @return  ((uint8_t)type << 4) | (msg_num & 0x0F)
 */
uint8_t proto_type_info_pack(const proto_type_info_t *t);

/**
 * @brief   从 1 字节 TYPE_INFO 解码出 TYPE 和 MSG_NUM
 */
void proto_type_info_unpack(uint8_t packed, proto_type_info_t *t);

/* ========== CRC-16/CCITT-FALSE（poly=0x1021, init=0xFFFF）========== */

/**
 * @brief   计算 CRC-16/CCITT-FALSE
 * @param   data  输入数据
 * @param   len   字节数
 * @return  16 位校验值（按位计算版本，0 ROM 开销）
 * @note    嵌入式友好：~8× CPU 换 0 ROM（井下节点 Flash 紧张）。
 *          若实测瓶颈可换查表（512 B ROM 表）。
 */
uint16_t proto_crc_calc(const uint8_t *data, uint32_t len);

/* ========== 帧 pack / unpack / validate ========== */

/**
 * @brief   把 frame 编码为带 HEAD/TAIL/转义/CRC16 的线缆字节流
 *
 * @param[out] buf        输出缓冲（≥ PROTO_FRAME_MAX）
 * @param      buf_size   缓冲容量
 * @param[in]  frame      待编码帧（payload_len ≤ PROTO_PAYLOAD_MAX）
 * @param[out] out_len    实际写入字节数
 *
 * @retval PROTO_ERR_OK
 * @retval PROTO_ERR_BUF_TOO_SMALL / PROTO_ERR_PAYLOAD_TOO_LONG / PROTO_ERR_ADDR_OUT_OF_RANGE
 */
proto_err_t proto_frame_pack(uint8_t *buf, uint32_t buf_size,
                             const proto_frame_t *frame,
                             uint32_t *out_len);

/**
 * @brief   从线缆字节流解码出 frame（自动去 HEAD/TAIL/转义/校验 CRC16）
 *
 * @param[in]  buf      输入字节流
 * @param      buf_len  字节数（≥ PROTO_FRAME_MIN）
 * @param[out] frame    解码结果
 *
 * @retval PROTO_ERR_OK
 * @retval PROTO_ERR_BAD_HEAD / BAD_TAIL / BAD_CRC / LEN_MISMATCH / BAD_ESCAPE
 * @retval PROTO_ERR_PAYLOAD_TOO_LONG / TOO_SHORT / BUF_TOO_SMALL
 */
proto_err_t proto_frame_unpack(const uint8_t *buf, uint32_t buf_len,
                               proto_frame_t *frame);

/**
 * @brief   轻量校验（不填 frame，只检查 HEAD/TAIL/TYPE/LEN 合法性）
 * @note    完整 CRC 校验交给 proto_frame_unpack，本函数适合"先看头再说"的场景
 */
proto_err_t proto_frame_validate(const uint8_t *buf, uint32_t buf_len);

/* ========== 便利宏 ========== */

/** 判断某 TYPE 是否已知 */
#define PROTO_TYPE_IS_VALID(t) \
    ((t) == PROTO_TYPE_QUERY  || (t) == PROTO_TYPE_CONTROL || \
     (t) == PROTO_TYPE_ASK    || (t) == PROTO_TYPE_REPLY   || \
     (t) == PROTO_TYPE_ALARM  || (t) == PROTO_TYPE_SCAN)

/** 判断某地址 4 位值是否合法 */
#define PROTO_ADDR_IS_VALID(a)  ((a) <= PROTO_ADDR_NIBBLE_MASK)

/** 按已知 TYPE 校验 MSG_NUM 是否在该 TYPE 的子表内（不做强类型检查，仅范围） */
#define PROTO_MSG_NUM_IN_RANGE(m)  ((m) <= 0x0Fu)

/* ========== 接收侧状态机（Bit 流 → 帧对象）========== */

/**
 * @brief   接收状态
 */
typedef enum {
    PROTO_RX_IDLE,          /**< 空闲，等待数据 */
    PROTO_RX_SYNCING,       /**< 搜索帧头 0xFF */
    PROTO_RX_RECEIVING,     /**< 接收帧体（直到 TAIL） */
    PROTO_RX_DONE,          /**< 接收完成，帧可取 */
    PROTO_RX_ERROR,         /**< 接收错误（CRC 失败等） */
} proto_rx_state_t;

/**
 * @brief   初始化接收状态机
 * @param   local_addr  本机地址（用于早期地址过滤，0xF 表示接收所有）
 * @retval  PROTO_ERR_OK
 */
proto_err_t proto_rx_init(uint8_t local_addr);

/**
 * @brief   喂入 bit 流（自动转字节 + 帧同步 + 解析）
 * @param   bit_buf     bit 流缓冲（LSB first，即 buf[0] 的 bit0 是第一个 bit）
 * @param   bit_count   bit 数（必须是 8 的倍数）
 * @retval  PROTO_ERR_OK / PROTO_ERR_BAD_CRC / PROTO_ERR_* （解析错误）
 * @note    内部状态机：
 *          - IDLE → 搜索 HEAD 0xFF → RECEIVING
 *          - RECEIVING → 累积字节直到 TAIL 0x00 → 调用 proto_frame_unpack()
 *          - 成功 → DONE，失败 → ERROR
 */
proto_err_t proto_rx_feed_bits(const uint8_t *bit_buf, uint16_t bit_count);

/**
 * @brief   获取接收状态
 */
proto_rx_state_t proto_rx_get_state(void);

/**
 * @brief   取出解析好的帧（仅 PROTO_RX_DONE 状态有效）
 * @param[out] out  输出帧对象
 * @retval  PROTO_ERR_OK / PROTO_ERR_BUF_TOO_SMALL / PROTO_ERR_* （状态错误）
 * @note    调用后状态机自动回到 IDLE
 */
proto_err_t proto_rx_take_frame(proto_frame_t *out);

/**
 * @brief   获取最后一次接收错误码（仅 PROTO_RX_ERROR 状态有效）
 */
proto_err_t proto_rx_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_PROTO_H */
