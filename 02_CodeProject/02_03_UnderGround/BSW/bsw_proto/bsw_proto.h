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
 * ==================== 帧类型快速参考 ====================
 *
 *   TYPE     | 枚举名           | 用途           | 子类型分类
 *   ---------|------------------|----------------|-----------------------------------------------
 *   0x0      | PROTO_TYPE_QUERY   | 查询帧（请求上报）| 4 种（VERSION、TEMP_PRESS、BATTERY、FAULT）
 *   0x1      | PROTO_TYPE_CONTROL | 控制帧（设置参数）| 5 种（SET_SLEEP_DUR、SLEEP、WAKE、SET_TEMP_THRESH、SET_PRESS_THRESH）
 *   0x2      | PROTO_TYPE_ASK     | 询问/应答帧（握手）| 询问：HEARTBEAT(0x0) / 应答：ALIVE(0x8)、ACK(0x9)、NACK(0xA)、RETRY_REQ(0xB)、FREQ_PAIR(0xC)
 *   0x4      | PROTO_TYPE_ALARM   | 警报帧（告警）    | 5 种（LOW_BATT、TEMP_OVER、PRESS_OVER、SENSOR_FAULT、LINK_FAIL）
 *   其他     | PROTO_TYPE_RESERVED | 保留              | -
 *
 *   每种 TYPE 下独立计数 MSG_NUM（4 位，0~15）。
 *   特殊设计：PROTO_TYPE_ASK 使用 MSG_NUM 高位区分询问(0x0-0x7)/应答(0x8-0xF)。
 *
 * ==================== 典型通信流程 ====================
 *
 *   场景 1：查询温压（井上 → 井下）
 *      1. 井上 → 井下: QUERY_TEMP_PRESS（查询温压）
 *      2. 井下 → 井上: REPLY_ACK（确认收到）
 *      3. 井下 → 井上: QUERY_TEMP_PRESS + 数据（回复温压数据）
 *
 *   场景 2：握手（井上 → 井下）
 *      1. 井上 → 井下: ASK_HEARTBEAT（心跳查询：你还活着吗？）
 *      2. 井下 → 井上: ASK_ALIVE（心跳应答：我还活着）
 *      3. 井上 → 井下: QUERY_xxx（开始查询）
 *
 *   场景 3：警报（井下 → 井上）
 *      1. 井下 → 井上: ALARM_TEMP_OVER（温度超限警报）
 *      2. 井上 → 井下: REPLY_ACK（收到警报）
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

/* ============================================================================
 * 帧边界常量
 * ============================================================================ */

#define PROTO_HEAD              (0xFFu)   /* 帧起始固定字节 */
#define PROTO_TAIL              (0x00u)   /* 帧结束固定字节 */
#define PROTO_ESC               (0xFEu)   /* 转义引导字节 */

/* ============================================================================
 * 帧长度限制
 * ============================================================================ */

#define PROTO_PAYLOAD_MAX       (10u)     /* 单帧 PAYLOAD 最大字节数（原始，未转义）*/
#define PROTO_ADDR_NIBBLE_MASK  (0x0Fu)   /* 4 位地址掩码 */
#define PROTO_SEQ_MAX           (255u)    /* 序列号最大值 */

/* ============================================================================
 * 协议字段大小常量
 * ============================================================================ */

#define PROTO_CRC16_SIZE        (2u)      /* CRC16 字段字节数 */
#define PROTO_HEADER_SIZE       (5u)      /* HEAD + ADDR + TYPE_INFO + SEQ + LEN */
#define PROTO_FIXED_SIZE        (PROTO_HEADER_SIZE + PROTO_CRC16_SIZE + 1u)  /* +1 为 TAIL */
#define BITS_PER_BYTE           (8u)      /* 每字节位数 */

/* ============================================================================
 * 协议字段偏移量（用于早期过滤）
 * ============================================================================ */

#define PROTO_OFFSET_ADDR       (2u)      /* rx_len == 2 时完整接收到 ADDR 字段（索引 1）*/
#define PROTO_OFFSET_TYPE_INFO  (3u)      /* rx_len == 3 时完整接收到 TYPE_INFO 字段（索引 2）*/
#define PROTO_OFFSET_LEN        (5u)      /* rx_len == 5 时完整接收到 LEN 字段（索引 4）*/

/* ============================================================================
 * 整帧字节数限制
 * ============================================================================ */

/**
 * @brief 整帧线缆字节上限（最坏情况：PAYLOAD 和 CRC16 每个字节都被转义成 2 字节）
 * @details 计算：
 *          HEAD(1) + ADDR(1) + TYPE_INFO(1) + SEQ(1) + LEN(1)
 *        + PAYLOAD 转义后(2 * PROTO_PAYLOAD_MAX = 64)
 *        + CRC16 转义后(4)
 *        + TAIL(1)
 *        = 74，按 8 对齐到 80
 */
#define PROTO_FRAME_MAX         (80u)

/**
 * @brief 整帧最小字节数
 * @details HEAD + ADDR + TYPE_INFO + SEQ + LEN=0 + CRC16 + TAIL = 8 字节
 */
#define PROTO_FRAME_MIN         (8u)

/* ============================================================================
 * TYPE - 帧类型（4 位大类，共 5 种已定义）
 * ============================================================================ */

typedef enum
{
    PROTO_TYPE_QUERY      = 0x0,    /* 查询帧：请求对方上报数据 */
    PROTO_TYPE_CONTROL    = 0x1,    /* 控制帧：设置参数/状态 */
    PROTO_TYPE_ASK        = 0x2,    /* 询问/应答帧：握手协议（MSG_NUM 0x0-0x7 为询问，0x8-0xF 为应答）*/
    PROTO_TYPE_ALARM      = 0x4,    /* 警报帧：主动告警 */
    PROTO_TYPE_RESERVED   = 0xF,    /* 保留 */
} proto_type_t;

/* ============================================================================
 * MSG_NUM - 子类型（每种 TYPE 独立计数，4 位）
 * ============================================================================ */

/**
 * @brief QUERY - 查询帧子类型（§6.1）
 * @details 请求对方上报数据，无载荷
 */
typedef enum
{
    PROTO_QUERY_TEMP_PRESS = 0x0,   /* 触发对方上报温压 */
    PROTO_QUERY_BATTERY    = 0x1,   /* 触发对方上报电池电压/电量 */
    PROTO_QUERY_FAULT      = 0x2,   /* 触发对方上报传感器故障 */
    PROTO_QUERY_VERSION    = 0x3,   /* 触发对方上报软硬件版本号（日期格式）*/
} proto_query_msg_t;

/**
 * @brief CONTROL - 控制帧子类型（§6.2）
 * @details 设置参数/状态
 */
typedef enum
{
    PROTO_CONTROL_SET_SLEEP_DUR    = 0x0,   /* 载荷 [sleep_sec(2)] 设置下次休眠时长 */
    PROTO_CONTROL_SLEEP            = 0x1,   /* (空) 立刻进入休眠 */
    PROTO_CONTROL_WAKE             = 0x2,   /* (空) 强制唤醒（中继/井上） */
    PROTO_CONTROL_SET_TEMP_THRESH  = 0x3,   /* 载荷 [temp_low(2)][temp_high(2)] 0.01°C */
    PROTO_CONTROL_SET_PRESS_THRESH = 0x4,   /* 载荷 [press_low(2)][press_high(2)] 0.01MPa */
} proto_control_msg_t;

/**
 * @brief ASK - 询问/应答帧子类型（§6.3 + §6.4 合并）
 * @details 握手协议，MSG_NUM 使用高位区分询问/应答：
 *          - 0x0-0x7：询问（Ask）
 *          - 0x8-0xF：应答（Reply）
 * 
 * @note 版本号通过 PROTO_TYPE_QUERY + PROTO_QUERY_VERSION 查询，
 *       此处仅保留握手相关的应答：心跳应答、ACK、NACK、重传请求、频点确认
 */
typedef enum
{
    /* 询问子类型 (0x0 - 0x7) */
    PROTO_ASK_HEARTBEAT = 0x0,      /* (空) 心跳查询：你还活着吗？ */
    
    /* 应答子类型 (0x8 - 0xF) */
    PROTO_ASK_ALIVE        = 0x8,   /* (空) 心跳应答：我还活着 */
    PROTO_ASK_ACK          = 0x9,   /* (空) 应答：业务帧接收成功 */
    PROTO_ASK_NACK         = 0xA,   /* 载荷 [err_code(1)] 应答：接收失败，详见 proto_nack_code_t */
    PROTO_ASK_FREQ_PAIR    = 0xB,   /* 载荷 [f0_hz(2)][f1_hz(2)] 应答：频点确认回复（扫频后建链） */
} proto_ask_msg_t;

/**
 * @brief ALARM - 警报帧子类型（§6.5）
 * @details 主动告警
 */
typedef enum
{
    PROTO_ALARM_LOW_BATT    = 0x0,  /* 载荷 [batt_pct(1)] 电量低 */
    PROTO_ALARM_TEMP_OVER   = 0x1,  /* 载荷 [current_temp(2)][limit_type(1)] 温度超限 */
    PROTO_ALARM_PRESS_OVER  = 0x2,  /* 载荷 [current_press(2)][limit_type(1)] 压力超限 */
    PROTO_ALARM_SENSOR_FAULT = 0x3, /* 载荷 [fault_code(1)][fault_data(2)] 传感器故障 */
    PROTO_ALARM_LINK_FAIL   = 0x4,  /* 载荷 [retry_count(1)] 链路失败 */
} proto_alarm_msg_t;


/* ============================================================================
 * 错误码 - API 返回值
 * ============================================================================ */

typedef enum
{
    PROTO_ERR_OK                = 0x00,     /* 成功 */
    PROTO_ERR_BAD_HEAD          = 0x01,     /* HEAD ≠ 0xFF */
    PROTO_ERR_BAD_TAIL          = 0x02,     /* TAIL ≠ 0x00 或未找到 */
    PROTO_ERR_BAD_CRC           = 0x03,     /* CRC16 校验失败 */
    PROTO_ERR_LEN_MISMATCH      = 0x04,     /* LEN 与解出的 PAYLOAD 字节数不匹配 */
    PROTO_ERR_BAD_ESCAPE        = 0x05,     /* 转义序列非法（ESC 后续字节不是 01/02/03）*/
    PROTO_ERR_UNKNOWN_TYPE      = 0x06,     /* TYPE 字段未知 */
    PROTO_ERR_UNKNOWN_MSG       = 0x07,     /* MSG_NUM 字段未知（TYPE 已知）*/
    PROTO_ERR_ADDR_OUT_OF_RANGE = 0x08,     /* DST/SRC 字段 > 0x0F */
    PROTO_ERR_PAYLOAD_TOO_LONG  = 0x09,     /* LEN 超过 PROTO_PAYLOAD_MAX 或解出的字节数超限 */
    PROTO_ERR_BUF_TOO_SMALL     = 0x0A,     /* 缓冲过小或入参 NULL */
    PROTO_ERR_TOO_SHORT         = 0x0B,     /* 整帧字节数 < PROTO_FRAME_MIN */
} proto_err_t;

/* ============================================================================
 * NACK 错误码（协议 §6.4 列出，独立于 API 错误码）
 * ============================================================================ */

typedef enum
{
    PROTO_NACK_CRC_FAIL         = 0x01,     /* CRC 校验失败 */
    PROTO_NACK_LEN_BAD          = 0x02,     /* 长度异常 */
    PROTO_NACK_TYPE_UNKNOWN     = 0x03,     /* TYPE/MSG_NUM 未知 */
    PROTO_NACK_ADDR_MISMATCH    = 0x04,     /* 目标地址不匹配 */
    PROTO_NACK_SEQ_DUP          = 0x05,     /* SEQ 序号重复/过期 */
    PROTO_NACK_SENSOR_FAIL      = 0x06,     /* 传感器故障（无有效数据）*/
} proto_nack_code_t;

/* ============================================================================
 * ALARM 传感器故障码（协议 §6.5）
 * ============================================================================ */

typedef enum
{
    PROTO_FAULT_PT100_OPEN        = 0x01,    /* PT100 断路 */
    PROTO_FAULT_PT100_SHORT       = 0x02,    /* PT100 短路 */
    PROTO_FAULT_PRESS_NO_RESPOND  = 0x03,    /* 压力传感器无响应 */
    PROTO_FAULT_ADC_BAD           = 0x04,    /* ADC 异常 */
    PROTO_FAULT_RTC_BAD           = 0x05,    /* RTC 异常 */
} proto_alarm_fault_t;

/**
 * @brief ALARM 超限类型
 */
typedef enum
{
    PROTO_LIMIT_BELOW_LOW  = 0x01,          /* 低于下限 */
    PROTO_LIMIT_ABOVE_HIGH = 0x02,          /* 高于上限 */
} proto_alarm_limit_t;

/* ============================================================================
 * 载荷结构定义 - 版本号
 * ============================================================================ */

/**
 * @brief   版本号载荷（用于 PROTO_ASK_VERSION）
 * 
 * @details
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
typedef struct
{
    uint32_t sw_version_date;  /* 软件版本号（日期格式 YYYYMMDD）*/
    uint32_t hw_version_date;  /* 硬件版本号（日期格式 YYYYMMDD）*/
} proto_version_payload_t;

/** 版本号载荷字节数 */
#define PROTO_VERSION_PAYLOAD_SIZE  (8u)

/* ============================================================================
 * 载荷结构定义 - 温压数据
 * ============================================================================ */

/**
 * @brief   温压数据载荷（用于 QUERY 温压查询的回复）
 * 
 * @details
 * 载荷格式（共 6 字节）：
 *   [temperature_centi_c(2)] + [pressure_pa(4)]
 * 
 * 数据单位：
 *   - temperature_centi_c: int16_t，单位 0.01°C（例如 2550 表示 25.50°C）
 *   - pressure_pa:         uint32_t，单位 Pa（例如 101325 表示 101.325 kPa）
 * 
 * @note    小端传输（低字节在前）
 * 
 * @example
 *   温度：25.50°C  → temperature_centi_c = 2550 → [0xF6, 0x09]
 *   气压：101325Pa → pressure_pa = 101325 → [0xCD, 0x8B, 0x01, 0x00]
 */
typedef struct
{
    int16_t  temperature_centi_c;  /* 温度，单位 0.01°C，有符号（支持负温）*/
    uint32_t pressure_pa;          /* 气压，单位 Pa，无符号 */
} proto_temp_press_payload_t;

/** 温压载荷字节数 */
#define PROTO_TEMP_PRESS_PAYLOAD_SIZE  (6u)

/* ============================================================================
 * 帧对象结构定义
 * ============================================================================ */

/**
 * @brief 地址对（解码/编码前的人类可读形式）
 */
typedef struct
{
    uint8_t dst;       /* 目的地址 0..15（见 bsw_node_id.h） */
    uint8_t src;       /* 源地址   0..15 */
} proto_addr_t;

/**
 * @brief 类型/子类型对
 */
typedef struct
{
    proto_type_t type;     /* 大类（QUERY、CONTROL、ASK、REPLY、ALARM、SCAN） */
    uint8_t      msg_num;  /* 子类型 0..15，对应 proto_*_msg_t 枚举 */
} proto_type_info_t;

/**
 * @brief 完整帧对象（解码后）
 */
typedef struct
{
    proto_addr_t      addr;                       /* 地址对（DST、SRC） */
    proto_type_info_t type_info;                  /* 类型对（TYPE、MSG_NUM） */
    uint8_t           seq;                        /* 序列号 0..255 */
    uint8_t           payload_len;                /* 载荷长度 0..PROTO_PAYLOAD_MAX */
    uint8_t           payload[PROTO_PAYLOAD_MAX]; /* 载荷数据 */
} proto_frame_t;

/* ============================================================================
 * 地址/类型编解码函数
 * ============================================================================ */

/**
 * @brief   把 DST/SRC 4 位对打包到 1 字节
 * @param   a  地址对指针
 * @return  打包后的字节 (dst << 4) | (src & 0x0F)
 */
uint8_t proto_addr_pack(const proto_addr_t *a);

/**
 * @brief   从 1 字节 ADDR 解码出 DST/SRC
 * @param   packed  打包的 ADDR 字节
 * @param   a       输出地址对指针
 */
void proto_addr_unpack(uint8_t packed, proto_addr_t *a);

/**
 * @brief   把 TYPE/MSG_NUM 4 位对打包到 1 字节
 * @param   t  类型对指针
 * @return  打包后的字节 ((uint8_t)type << 4) | (msg_num & 0x0F)
 */
uint8_t proto_type_info_pack(const proto_type_info_t *t);

/**
 * @brief   从 1 字节 TYPE_INFO 解码出 TYPE 和 MSG_NUM
 * @param   packed  打包的 TYPE_INFO 字节
 * @param   t       输出类型对指针
 */
void proto_type_info_unpack(uint8_t packed, proto_type_info_t *t);

/* ============================================================================
 * CRC-16/CCITT-FALSE 校验
 * ============================================================================ */

/**
 * @brief   计算 CRC-16/CCITT-FALSE
 * @param   data  输入数据
 * @param   len   字节数
 * @return  16 位校验值（按位计算版本，0 ROM 开销）
 * 
 * @note    嵌入式友好：~8× CPU 换 0 ROM（井下节点 Flash 紧张）。
 *          若实测瓶颈可换查表（512 B ROM 表）。
 */
uint16_t proto_crc_calc(const uint8_t *data, uint32_t len);

/* ============================================================================
 * 帧打包/解包/校验函数
 * ============================================================================ */

/**
 * @brief   把 frame 编码为带 HEAD/TAIL/转义/CRC16 的线缆字节流
 *
 * @param[out] buf        输出缓冲（≥ PROTO_FRAME_MAX）
 * @param      buf_size   缓冲容量
 * @param[in]  frame      待编码帧（payload_len ≤ PROTO_PAYLOAD_MAX）
 * @param[out] out_len    实际写入字节数
 *
 * @retval PROTO_ERR_OK                     成功
 * @retval PROTO_ERR_BUF_TOO_SMALL          缓冲过小
 * @retval PROTO_ERR_PAYLOAD_TOO_LONG       载荷超长
 * @retval PROTO_ERR_ADDR_OUT_OF_RANGE      地址越界
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
 * @retval PROTO_ERR_OK                     成功
 * @retval PROTO_ERR_BAD_HEAD               HEAD ≠ 0xFF
 * @retval PROTO_ERR_BAD_TAIL               TAIL ≠ 0x00 或未找到
 * @retval PROTO_ERR_BAD_CRC                CRC16 校验失败
 * @retval PROTO_ERR_LEN_MISMATCH           LEN 与解出的 PAYLOAD 字节数不匹配
 * @retval PROTO_ERR_BAD_ESCAPE             转义序列非法
 * @retval PROTO_ERR_PAYLOAD_TOO_LONG       载荷超长
 * @retval PROTO_ERR_TOO_SHORT              整帧字节数 < PROTO_FRAME_MIN
 * @retval PROTO_ERR_BUF_TOO_SMALL          缓冲过小或入参 NULL
 */
proto_err_t proto_frame_unpack(const uint8_t *buf, uint32_t buf_len,
                               proto_frame_t *frame);

/**
 * @brief   轻量校验（不填 frame，只检查 HEAD/TAIL/TYPE/LEN 合法性）
 * @param   buf      输入字节流
 * @param   buf_len  字节数
 * @return  proto_err_t 校验结果
 * 
 * @note    完整 CRC 校验交给 proto_frame_unpack，本函数适合"先看头再说"的场景
 */
proto_err_t proto_frame_validate(const uint8_t *buf, uint32_t buf_len);

/* ============================================================================
 * 便利宏
 * ============================================================================ */

/** 判断某 TYPE 是否已知 */
#define PROTO_TYPE_IS_VALID(t) \
    ((t) == PROTO_TYPE_QUERY  || (t) == PROTO_TYPE_CONTROL || \
     (t) == PROTO_TYPE_ASK    || (t) == PROTO_TYPE_ALARM)

/** 判断某地址 4 位值是否合法 */
#define PROTO_ADDR_IS_VALID(a)  ((a) <= PROTO_ADDR_NIBBLE_MASK)

/** 按已知 TYPE 校验 MSG_NUM 是否在该 TYPE 的子表内（不做强类型检查，仅范围） */
#define PROTO_MSG_NUM_IN_RANGE(m)  ((m) <= 0x0Fu)

/* ============================================================================
 * 接收侧状态机（Bit 流 → 帧对象）
 * ============================================================================ */

/**
 * @brief   接收状态
 */
typedef enum
{
    PROTO_RX_IDLE,          /* 空闲，等待数据 */
    PROTO_RX_SYNCING,       /* 搜索帧头 0xFF */
    PROTO_RX_RECEIVING,     /* 接收帧体（直到 TAIL） */
    PROTO_RX_DONE,          /* 接收完成，帧可取 */
    PROTO_RX_ERROR,         /* 接收错误（CRC 失败等） */
} proto_rx_state_t;

/**
 * @brief   初始化接收状态机
 * @param   local_addr  本机地址（用于早期地址过滤）
 * @return  PROTO_ERR_OK
 * 
 * @note    接收条件：
 *            1. 目的地址 == local_addr（单播）
 *            2. 目的地址 == 0xF（广播）且 上级或下级地址未知
 *          通过 bsw_node_id_set_upstream/downstream() 设置邻居地址后，
 *          当上下级地址都已知时，将拒绝接收广播帧，避免干扰其他链路。
 */
proto_err_t proto_rx_init(uint8_t local_addr);

/**
 * @brief   喂入 bit 流（自动转字节 + 帧同步 + 解析）
 * @param   bit_buf     bit 流缓冲（LSB first，即 buf[0] 的 bit0 是第一个 bit）
 * @param   bit_count   bit 数（必须是 8 的倍数）
 * @return  proto_err_t 解析结果
 * 
 * @note    内部状态机：
 *          - IDLE → 搜索 HEAD 0xFF → RECEIVING
 *          - RECEIVING → 累积字节直到 TAIL 0x00 → 调用 proto_frame_unpack()
 *          - 成功 → DONE，失败 → ERROR
 */
proto_err_t proto_rx_feed_bits(const uint8_t *bit_buf, uint16_t bit_count);

/**
 * @brief   获取接收状态
 * @return  proto_rx_state_t 当前状态
 */
proto_rx_state_t proto_rx_get_state(void);

/**
 * @brief   取出解析好的帧（仅 PROTO_RX_DONE 状态有效）
 * @param[out] out  输出帧对象
 * @return  proto_err_t 取出结果
 * 
 * @note    调用后状态机自动回到 IDLE
 */
proto_err_t proto_rx_take_frame(proto_frame_t *out);

/**
 * @brief   获取最后一次接收错误码（仅 PROTO_RX_ERROR 状态有效）
 * @return  proto_err_t 错误码
 */
proto_err_t proto_rx_get_last_error(void);

#ifdef __cplusplus
}
#endif

#endif /* BSW_PROTO_H */
