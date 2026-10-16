/**
 * @file    app_frame_handler.c
 * @brief   帧业务处理层 - 实现
 *
 * 处理流程（单一流程）：
 *   接收帧 → 判断类型 → 执行业务逻辑 → 组装回复帧 → 调制发送 → 返回结果
 *
 * 架构设计：
 *   - 顶层分发：使用 switch 按帧类型（4 种稳定类型）分发
 *   - 子类型处理：使用函数表模式，便于扩展和单元测试
 *   - 每个子类型处理函数独立，职责清晰
 *
 * 当前实现状态：
 *   - _handle_query_frame()    QUERY 类型查询处理（函数表，4 项）
 *     - PROTO_QUERY_VERSION    版本号查询（✅ 已完成）
 *     - PROTO_QUERY_TEMP_PRESS 温压查询（✅ 已完成）
 *     - PROTO_QUERY_BATTERY    电池查询（⏳ 待实现）
 *     - PROTO_QUERY_FAULT      故障查询（⏳ 待实现）
 *
 *   - _handle_control_frame()  CONTROL 类型控制帧（函数表，5 项）
 *     - PROTO_CONTROL_SET_SLEEP_DUR    设置休眠时长（⏳ 待实现）
 *     - PROTO_CONTROL_SLEEP            立刻休眠（⏳ 待实现）
 *     - PROTO_CONTROL_WAKE             强制唤醒（⏳ 待实现）
 *     - PROTO_CONTROL_SET_TEMP_THRESH  设置温度阈值（⏳ 待实现）
 *     - PROTO_CONTROL_SET_PRESS_THRESH 设置气压阈值（⏳ 待实现）
 *
 *   - _handle_ask_frame()      ASK 类型询问/应答帧（函数表，6 项）
 *     - PROTO_ASK_HEARTBEAT    心跳查询（⏳ 待实现）
 *     - PROTO_ASK_ALIVE        心跳应答（⏳ 待实现）
 *     - PROTO_ASK_ACK          确认应答（⏳ 待实现）
 *     - PROTO_ASK_NACK         否定应答（⏳ 待实现）
 *     - PROTO_ASK_RETRY_REQ    重传请求（⏳ 待实现）
 *     - PROTO_ASK_FREQ_PAIR    频点确认（⏳ 待实现）
 *
 *   - _handle_alarm_frame()    ALARM 类型警报帧（函数表，5 项）
 *     - PROTO_ALARM_LOW_BATT    电池低电告警（⏳ 待实现）
 *     - PROTO_ALARM_TEMP_OVER   温度超限告警（⏳ 待实现）
 *     - PROTO_ALARM_PRESS_OVER  气压超限告警（⏳ 待实现）
 *     - PROTO_ALARM_SENSOR_FAULT 传感器故障告警（⏳ 待实现）
 *     - PROTO_ALARM_LINK_FAIL   链路失败告警（⏳ 待实现）
 *
 * @dependency  app_frame_handler.h
 *              bsw_proto.h
 *              bsw_version.h
 *              bsw_node_id.h
 *              app_sensor.h
 *              bsw_bfsk_mod.h
 */

#include "app_frame_handler.h"

#include <string.h>
#include "bsw_proto.h"
#include "bsw_version.h"
#include "bsw_node_id.h"
#include "app_sensor.h"
#include "app_msg_send.h"

/* ========== 配置宏定义 ========== */

/** 温压数据有效性检查 - 至少一个传感器有效 */
#define APP_SENSOR_MIN_VALID_COUNT          (1U)

/** 单次调度最多组装的回复帧数（ACK + 数据帧） */
#define APP_FRAME_HANDLER_MAX_REPLY_FRAMES  (2U)

/* ========== 内部类型定义 ========== */

/**
 * @brief QUERY 子类型处理函数原型
 * @param frame 接收到的查询帧
 * @param p_reply_buf 回复帧缓冲区（可能需要1-2个帧）
 * @param p_reply_count 输出：实际需要回复的帧数量
 * @return 处理结果
 */
typedef frame_handler_ret_t (*query_handler_t)(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/**
 * @brief QUERY 处理表项
 */
typedef struct
{
    uint8_t msg_num;                /**< 子类型编号（proto_query_msg_t）*/
    query_handler_t handler;        /**< 处理函数 */
    const char *p_name;             /**< 名称（调试用）*/
} QueryHandlerEntry_t;

/**
 * @brief ASK 子类型处理函数原型
 * @param frame 接收到的 ASK 帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：实际需要回复的帧数量
 * @return 处理结果
 */
typedef frame_handler_ret_t (*ask_handler_t)(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count);

/**
 * @brief ASK 处理表项
 */
typedef struct
{
    uint8_t msg_num;                /**< 子类型编号（proto_ask_msg_t）*/
    ask_handler_t handler;          /**< 处理函数 */
    const char *p_name;             /**< 名称（调试用）*/
} AskHandlerEntry_t;

/**
 * @brief CONTROL 子类型处理函数原型
 * @param frame 接收到的控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：实际需要回复的帧数量
 * @return 处理结果
 */
typedef frame_handler_ret_t (*control_handler_t)(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);

/**
 * @brief CONTROL 处理表项
 */
typedef struct
{
    uint8_t msg_num;                /**< 子类型编号（proto_control_msg_t）*/
    control_handler_t handler;      /**< 处理函数 */
    const char *p_name;             /**< 名称（调试用）*/
} ControlHandlerEntry_t;

/**
 * @brief ALARM 子类型处理函数原型
 * @param frame 接收到的警报帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：实际需要回复的帧数量
 * @return 处理结果
 */
typedef frame_handler_ret_t (*alarm_handler_t)(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/**
 * @brief ALARM 处理表项
 */
typedef struct
{
    uint8_t msg_num;                /**< 子类型编号（proto_alarm_msg_t）*/
    alarm_handler_t handler;        /**< 处理函数 */
    const char *p_name;             /**< 名称（调试用）*/
} AlarmHandlerEntry_t;

/* ========== 内部函数声明 ========== */

static frame_handler_ret_t _handle_query_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);
static frame_handler_ret_t _handle_control_frame(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_ask_frame(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count);
static frame_handler_ret_t _handle_alarm_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/* QUERY 子类型处理 */
static frame_handler_ret_t _handle_query_version(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_query_temp_press(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count);
static frame_handler_ret_t _handle_query_battery(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_query_fault(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count);

/* ASK 子类型处理 */
static frame_handler_ret_t _handle_ask_heartbeat(const proto_frame_t *frame,
                                                  proto_frame_t *p_reply_buf,
                                                  uint8_t *p_reply_count);
static frame_handler_ret_t _handle_ask_alive(const proto_frame_t *frame,
                                              proto_frame_t *p_reply_buf,
                                              uint8_t *p_reply_count);
static frame_handler_ret_t _handle_ask_ack(const proto_frame_t *frame,
                                            proto_frame_t *p_reply_buf,
                                            uint8_t *p_reply_count);
static frame_handler_ret_t _handle_ask_nack(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count);
static frame_handler_ret_t _handle_ask_freq_pair(const proto_frame_t *frame,
                                                  proto_frame_t *p_reply_buf,
                                                  uint8_t *p_reply_count);

/* CONTROL 子类型处理 */
static frame_handler_ret_t _handle_control_set_sleep_dur(const proto_frame_t *frame,
                                                          proto_frame_t *p_reply_buf,
                                                          uint8_t *p_reply_count);
static frame_handler_ret_t _handle_control_sleep(const proto_frame_t *frame,
                                                  proto_frame_t *p_reply_buf,
                                                  uint8_t *p_reply_count);
static frame_handler_ret_t _handle_control_wake(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count);
static frame_handler_ret_t _handle_control_set_temp_thresh(const proto_frame_t *frame,
                                                            proto_frame_t *p_reply_buf,
                                                            uint8_t *p_reply_count);
static frame_handler_ret_t _handle_control_set_press_thresh(const proto_frame_t *frame,
                                                             proto_frame_t *p_reply_buf,
                                                             uint8_t *p_reply_count);

/* ALARM 子类型处理 */
static frame_handler_ret_t _handle_alarm_low_batt(const proto_frame_t *frame,
                                                   proto_frame_t *p_reply_buf,
                                                   uint8_t *p_reply_count);
static frame_handler_ret_t _handle_alarm_temp_over(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count);
static frame_handler_ret_t _handle_alarm_press_over(const proto_frame_t *frame,
                                                     proto_frame_t *p_reply_buf,
                                                     uint8_t *p_reply_count);
static frame_handler_ret_t _handle_alarm_sensor_fault(const proto_frame_t *frame,
                                                       proto_frame_t *p_reply_buf,
                                                       uint8_t *p_reply_count);
static frame_handler_ret_t _handle_alarm_link_fail(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count);

/* 通用辅助函数 */
static bool _is_ack_frame(const proto_frame_t *p_frame);
static frame_handler_ret_t _send_frames(const proto_frame_t *p_frames, uint8_t count);
static void _make_ack_frame(const proto_frame_t *req_frame, proto_frame_t *p_ack_frame);
static void _make_nack_frame(const proto_frame_t *req_frame,
                             proto_nack_code_t err_code,
                             proto_frame_t *p_nack_frame);

/* ========== QUERY 处理函数表 ========== */

/**
 * @brief QUERY 子类型处理表
 * @note 新增 QUERY 子类型时，在此表添加一项即可，无需修改 _handle_query_frame()
 */
static const QueryHandlerEntry_t s_query_handler_table[] =
{
    /* msg_num,                handler,                        name */
    { PROTO_QUERY_VERSION,     _handle_query_version,          "VERSION" },
    { PROTO_QUERY_TEMP_PRESS,  _handle_query_temp_press,       "TEMP_PRESS" },
    { PROTO_QUERY_BATTERY,     _handle_query_battery,          "BATTERY" },
    { PROTO_QUERY_FAULT,       _handle_query_fault,            "FAULT" },
};

#define QUERY_HANDLER_TABLE_SIZE  (sizeof(s_query_handler_table) / sizeof(s_query_handler_table[0]))

/* ========== ASK 处理函数表 ========== */

/**
 * @brief ASK 子类型处理表
 * @note 新增 ASK 子类型时，在此表添加一项即可，无需修改 _handle_ask_frame()
 */
static const AskHandlerEntry_t s_ask_handler_table[] =
{
    /* msg_num,              handler,                name */
    { PROTO_ASK_HEARTBEAT,    _handle_ask_heartbeat, "HEARTBEAT" },
    { PROTO_ASK_ALIVE,        _handle_ask_alive,     "ALIVE" },
    { PROTO_ASK_ACK,          _handle_ask_ack,       "ACK" },
    { PROTO_ASK_NACK,         _handle_ask_nack,      "NACK" },
    { PROTO_ASK_FREQ_PAIR,    _handle_ask_freq_pair, "FREQ_PAIR" },
};

#define ASK_HANDLER_TABLE_SIZE  (sizeof(s_ask_handler_table) / sizeof(s_ask_handler_table[0]))

/* ========== CONTROL 处理函数表 ========== */

/**
 * @brief CONTROL 子类型处理表
 * @note 新增 CONTROL 子类型时，在此表添加一项即可，无需修改 _handle_control_frame()
 */
static const ControlHandlerEntry_t s_control_handler_table[] =
{
    /* msg_num,                          handler,                            name */
    { PROTO_CONTROL_SET_SLEEP_DUR,      _handle_control_set_sleep_dur,      "SET_SLEEP_DUR" },
    { PROTO_CONTROL_SLEEP,              _handle_control_sleep,              "SLEEP" },
    { PROTO_CONTROL_WAKE,               _handle_control_wake,               "WAKE" },
    { PROTO_CONTROL_SET_TEMP_THRESH,    _handle_control_set_temp_thresh,    "SET_TEMP_THRESH" },
    { PROTO_CONTROL_SET_PRESS_THRESH,   _handle_control_set_press_thresh,   "SET_PRESS_THRESH" },
};

#define CONTROL_HANDLER_TABLE_SIZE  (sizeof(s_control_handler_table) / sizeof(s_control_handler_table[0]))

/* ========== ALARM 处理函数表 ========== */

/**
 * @brief ALARM 子类型处理表
 * @note 新增 ALARM 子类型时，在此表添加一项即可，无需修改 _handle_alarm_frame()
 */
static const AlarmHandlerEntry_t s_alarm_handler_table[] =
{
    /* msg_num,                  handler,                        name */
    { PROTO_ALARM_LOW_BATT,      _handle_alarm_low_batt,         "LOW_BATT" },
    { PROTO_ALARM_TEMP_OVER,     _handle_alarm_temp_over,        "TEMP_OVER" },
    { PROTO_ALARM_PRESS_OVER,    _handle_alarm_press_over,       "PRESS_OVER" },
    { PROTO_ALARM_SENSOR_FAULT,  _handle_alarm_sensor_fault,     "SENSOR_FAULT" },
    { PROTO_ALARM_LINK_FAIL,     _handle_alarm_link_fail,        "LINK_FAIL" },
};

#define ALARM_HANDLER_TABLE_SIZE  (sizeof(s_alarm_handler_table) / sizeof(s_alarm_handler_table[0]))

/* ========== 公共 API 实现 ========== */

void app_frame_handler_init(void)
{
    /* 当前无需要初始化的内容（各 BSW 模块自己负责 init）*/
}

frame_handler_ret_t app_frame_handler_dispatch(const proto_frame_t *frame)
{
    if (frame == NULL)
    {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    /* 地址过滤：
     * 1. 目的地址 == 本机地址（单播）
     * 2. 目的地址 == 广播地址（0xF）且（上级或下级地址未知）
     * 
     * 为什么：握手阶段需要接收广播帧，链路建立后自动拒绝广播
     */
    uint8_t local_addr = bsw_node_id_get();
    bool accept = (frame->addr.dst == local_addr);
    
    if (!accept && (frame->addr.dst == BSW_NODE_ADDR_BROADCAST))
    {
        accept = bsw_node_id_can_accept_broadcast();
    }
    
    if (!accept)
    {
        return FRAME_HANDLER_ERR_ADDR_MISMATCH;
    }
    
    /* 准备回复帧缓冲区 */
    proto_frame_t reply_buf[APP_FRAME_HANDLER_MAX_REPLY_FRAMES];
    uint8_t reply_count = 0;
    frame_handler_ret_t ret;
    
    /* 根据帧类型分发 */
    switch (frame->type_info.type)
    {
        case PROTO_TYPE_QUERY:
            ret = _handle_query_frame(frame, reply_buf, &reply_count);
            break;
            
        case PROTO_TYPE_CONTROL:
            ret = _handle_control_frame(frame, reply_buf, &reply_count);
            break;
            
        case PROTO_TYPE_ASK:
            ret = _handle_ask_frame(frame, reply_buf, &reply_count);
            break;
            
        case PROTO_TYPE_ALARM:
            ret = _handle_alarm_frame(frame, reply_buf, &reply_count);
            break;
            
        default:
            return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
    }
    
    /* 如果业务处理成功且有需要发送的回复帧，则发送 */
    if (ret == FRAME_HANDLER_OK && reply_count > 0)
    {
        ret = _send_frames(reply_buf, reply_count);
    }
    
    return ret;
}

/* ========== QUERY 类型帧处理 ========== */

/**
 * @brief 处理 QUERY 类型帧 - 使用函数表模式
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 为什么用函数表：
 * 1. 添加新 QUERY 子类型时只需在表中添加，无需修改此函数
 * 2. 每个子类型处理逻辑独立，便于单元测试
 * 3. 表项集中管理，便于查看所有支持的查询类型
 */
static frame_handler_ret_t _handle_query_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count)
{
    /* 在处理表中查找对应的处理函数 */
    for (uint16_t i = 0; i < QUERY_HANDLER_TABLE_SIZE; i++)
    {
        if (s_query_handler_table[i].msg_num == frame->type_info.msg_num)
        {
            return s_query_handler_table[i].handler(frame, p_reply_buf, p_reply_count);
        }
    }
    
    /* 未找到对应的处理函数 */
    return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
}

/**
 * @brief 处理版本号查询
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 读取本机版本信息
 * 2. 构造 QUERY 类型回复帧（携带版本数据）
 * 3. 设置输出参数
 * 4. 返回（由上层统一发送）
 */
static frame_handler_ret_t _handle_query_version(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    const bsw_version_info_t *p_ver = bsw_version_get_info();
    
    /* 准备回复帧：QUERY 类型，携带版本载荷 */
    proto_frame_t *p_reply = &p_reply_buf[0];
    p_reply->addr.dst = frame->addr.src;  /* 回复给查询方 */
    p_reply->addr.src = bsw_node_id_get();
    p_reply->type_info.type = PROTO_TYPE_QUERY;
    p_reply->type_info.msg_num = PROTO_QUERY_VERSION;
    p_reply->seq = frame->seq;
    p_reply->payload_len = PROTO_VERSION_PAYLOAD_SIZE;
    
    /* 打包版本号（小端格式）*/
    p_reply->payload[0] = (uint8_t)((p_ver->sw_version_date >>  0) & 0xFFU);
    p_reply->payload[1] = (uint8_t)((p_ver->sw_version_date >>  8) & 0xFFU);
    p_reply->payload[2] = (uint8_t)((p_ver->sw_version_date >> 16) & 0xFFU);
    p_reply->payload[3] = (uint8_t)((p_ver->sw_version_date >> 24) & 0xFFU);
    p_reply->payload[4] = (uint8_t)((p_ver->hw_version_date >>  0) & 0xFFU);
    p_reply->payload[5] = (uint8_t)((p_ver->hw_version_date >>  8) & 0xFFU);
    p_reply->payload[6] = (uint8_t)((p_ver->hw_version_date >> 16) & 0xFFU);
    p_reply->payload[7] = (uint8_t)((p_ver->hw_version_date >> 24) & 0xFFU);
    
    *p_reply_count = 1;  /* 只需回复1个帧 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理温压查询
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 读取传感器数据
 * 2. 检查有效性（至少一个传感器有效）
 * 3. 先组装 ACK 确认帧
 * 4. 再组装温压数据帧
 * 5. 返回2个帧（由上层统一发送）
 */
static frame_handler_ret_t _handle_query_temp_press(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count)
{
    /* 读取传感器数据 */
    app_sensor_data_t sensor_data;
    if (app_sensor_get_data(&sensor_data) != 0)
    {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    /* 检查数据有效性：至少温度或气压之一有效才回复 */
    if (!sensor_data.temp_valid && !sensor_data.press_valid)
    {
        /* 两个传感器都失败，组装 NACK */
        _make_nack_frame(frame, PROTO_NACK_SENSOR_FAIL, &p_reply_buf[0]);
        *p_reply_count = 1;
        return FRAME_HANDLER_OK;
    }
    
    /* 第1个帧：ACK 确认收到 */
    _make_ack_frame(frame, &p_reply_buf[0]);
    
    /* 第2个帧：温压数据帧（QUERY 类型，带数据载荷）*/
    proto_frame_t *p_data_frame = &p_reply_buf[1];
    p_data_frame->addr.dst = frame->addr.src;  /* 回复给查询方 */
    p_data_frame->addr.src = bsw_node_id_get();
    p_data_frame->type_info.type = PROTO_TYPE_QUERY;
    p_data_frame->type_info.msg_num = PROTO_QUERY_TEMP_PRESS;
    p_data_frame->seq = (frame->seq + 1) & 0xFFU;  /* SEQ + 1 */
    p_data_frame->payload_len = PROTO_TEMP_PRESS_PAYLOAD_SIZE;
    
    /* 打包温压数据（小端格式）*/
    int16_t temp = sensor_data.temp_valid ? sensor_data.temperature_centi_c : 0;
    uint32_t press = sensor_data.press_valid ? sensor_data.pressure_pa : 0;
    
    p_data_frame->payload[0] = (uint8_t)((temp >>  0) & 0xFFU);
    p_data_frame->payload[1] = (uint8_t)((temp >>  8) & 0xFFU);
    p_data_frame->payload[2] = (uint8_t)((press >>  0) & 0xFFU);
    p_data_frame->payload[3] = (uint8_t)((press >>  8) & 0xFFU);
    p_data_frame->payload[4] = (uint8_t)((press >> 16) & 0xFFU);
    p_data_frame->payload[5] = (uint8_t)((press >> 24) & 0xFFU);
    
    *p_reply_count = 2;  /* 回复 ACK + 数据帧 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理电池查询（TODO）
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 */
static frame_handler_ret_t _handle_query_battery(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 读取电池电压并组装回复 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理故障查询（TODO）
 * @param frame 查询帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 */
static frame_handler_ret_t _handle_query_fault(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 读取故障状态并组装回复 */
    return FRAME_HANDLER_OK;
}

/* ========== CONTROL 类型帧处理 ========== */

/**
 * @brief 处理 CONTROL 类型帧 - 使用函数表模式
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 为什么用函数表：
 * 1. 添加新 CONTROL 子类型时只需在表中添加，无需修改此函数
 * 2. 每个子类型处理逻辑独立，便于单元测试
 * 3. 表项集中管理，便于查看所有支持的控制命令
 */
static frame_handler_ret_t _handle_control_frame(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    /* 在处理表中查找对应的处理函数 */
    for (size_t i = 0; i < CONTROL_HANDLER_TABLE_SIZE; i++)
    {
        if (s_control_handler_table[i].msg_num == frame->type_info.msg_num)
        {
            return s_control_handler_table[i].handler(frame, p_reply_buf, p_reply_count);
        }
    }
    
    /* 未找到对应的处理函数 */
    return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
}

/**
 * @brief 处理设置休眠时长控制命令
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：根据载荷设置下次休眠时长
 */
static frame_handler_ret_t _handle_control_set_sleep_dur(const proto_frame_t *frame,
                                                          proto_frame_t *p_reply_buf,
                                                          uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 SET_SLEEP_DUR 处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理立刻进入休眠控制命令
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：触发系统进入休眠状态
 */
static frame_handler_ret_t _handle_control_sleep(const proto_frame_t *frame,
                                                  proto_frame_t *p_reply_buf,
                                                  uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 SLEEP 处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理强制唤醒控制命令
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：唤醒系统（中继/井上使用）
 */
static frame_handler_ret_t _handle_control_wake(const proto_frame_t *frame,
                                                 proto_frame_t *p_reply_buf,
                                                 uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 WAKE 处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理设置温度阈值控制命令
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：设置温度告警上下限
 */
static frame_handler_ret_t _handle_control_set_temp_thresh(const proto_frame_t *frame,
                                                            proto_frame_t *p_reply_buf,
                                                            uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 SET_TEMP_THRESH 处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理设置气压阈值控制命令
 * @param frame 控制帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：设置气压告警上下限
 */
static frame_handler_ret_t _handle_control_set_press_thresh(const proto_frame_t *frame,
                                                             proto_frame_t *p_reply_buf,
                                                             uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 SET_PRESS_THRESH 处理 */
    return FRAME_HANDLER_OK;
}

/* ========== ASK 类型帧处理 ========== */

/**
 * @brief 处理 ASK 类型帧 - 使用函数表模式
 * @param frame 询问/应答帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 为什么用函数表：
 * 1. ASK 有 6 个子类型（询问 1 个 + 应答 5 个），用函数表便于管理
 * 2. 添加新 ASK 子类型时只需在表中添加，无需修改此函数
 * 3. 每个子类型处理逻辑独立，便于单元测试
 * 
 * ASK 帧分类：
 * - MSG_NUM < 0x8：询问帧（例如 IDLE_CONFIRM）
 * - MSG_NUM >= 0x8：应答帧（例如 ACK、NACK、READY）
 */
static frame_handler_ret_t _handle_ask_frame(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count)
{
    /* 在处理表中查找对应的处理函数 */
    for (size_t i = 0; i < ASK_HANDLER_TABLE_SIZE; i++)
    {
        if (s_ask_handler_table[i].msg_num == frame->type_info.msg_num)
        {
            return s_ask_handler_table[i].handler(frame, p_reply_buf, p_reply_count);
        }
    }
    
    /* 未找到对应的处理函数 */
    return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
}

/**
 * @brief 处理心跳查询（HEARTBEAT）
 * @param frame 询问帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 收到心跳查询，说明对方询问"你还活着吗？"
 * 2. 如果能正确解析并执行到这里，说明本机活着且能接收数据
 * 3. 回复 ALIVE 应答
 * 
 * 实现：构造 ALIVE 应答帧（空载荷）
 */
static frame_handler_ret_t _handle_ask_heartbeat(const proto_frame_t *frame,
                                                  proto_frame_t *p_reply_buf,
                                                  uint8_t *p_reply_count)
{
    /* 组装 ALIVE 应答帧 */
    proto_frame_t *p_reply = &p_reply_buf[0];
    p_reply->addr.dst = frame->addr.src;  /* 回复给查询方 */
    p_reply->addr.src = bsw_node_id_get();
    p_reply->type_info.type = PROTO_TYPE_ASK;
    p_reply->type_info.msg_num = PROTO_ASK_ALIVE;
    p_reply->seq = frame->seq;
    p_reply->payload_len = 0;  /* 空载荷，收到即说明活着 */
    
    *p_reply_count = 1;  /* 回复 1 个 ALIVE 帧 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理心跳应答（ALIVE）
 * @param frame 应答帧
 * @param p_reply_buf 回复帧缓冲区（本函数不需要回复）
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 收到 ALIVE 应答，说明对方还活着
 * 2. 更新心跳时间戳，标记链路存活
 * 3. 不需要回复
 * 
 * 待实现：通知状态机"链路心跳正常"
 */
static frame_handler_ret_t _handle_ask_alive(const proto_frame_t *frame,
                                              proto_frame_t *p_reply_buf,
                                              uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    
    /* 无需回复 */
    *p_reply_count = 0;
    
    /* TODO: 更新心跳时间戳，标记链路存活 */
    /* 示例：g_last_heartbeat_time = get_current_time(); */
    /* 示例：g_link_alive = true; */
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理 ACK 应答
 * @param frame 应答帧
 * @param p_reply_buf 回复帧缓冲区（本函数不需要回复）
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 收到 ACK，说明对方成功接收了我们发送的帧
 * 2. 通知 app_msg_send 模块"ACK 已收到"
 * 3. app_msg_send 会停止重传该帧
 * 4. 不需要再回复
 */
static frame_handler_ret_t _handle_ask_ack(const proto_frame_t *frame,
                                            proto_frame_t *p_reply_buf,
                                            uint8_t *p_reply_count)
{
    (void)p_reply_buf;
    
    /* 无需回复 */
    *p_reply_count = 0;
    
    /* 通知发送模块"ACK 收到，发送成功" */
    app_msg_send_notify_ack(frame->seq);
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理 NACK 应答
 * @param frame 应答帧
 * @param p_reply_buf 回复帧缓冲区（本函数不需要回复）
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 流程：
 * 1. 收到 NACK，说明对方接收失败（CRC错误、地址不匹配、传感器故障等）
 * 2. 通知 app_msg_send 模块"收到 NACK"
 * 3. app_msg_send 会根据错误码决定是否重传
 * 4. 载荷格式：[err_code(1)]
 * 5. 不需要回复
 */
static frame_handler_ret_t _handle_ask_nack(const proto_frame_t *frame,
                                             proto_frame_t *p_reply_buf,
                                             uint8_t *p_reply_count)
{
    (void)p_reply_buf;
    
    /* 无需回复 */
    *p_reply_count = 0;
    
    /* 解析错误码 */
    if (frame->payload_len >= 1)
    {
        proto_nack_code_t err_code = (proto_nack_code_t)frame->payload[0];
        
        /* 通知发送模块"NACK 收到，可能需要重传" */
        app_msg_send_notify_nack(frame->seq, err_code);
    }
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理重传请求（RETRY_REQ）
 * @param frame 应答帧
 * @param p_reply_buf 回复帧缓冲区（本函数不需要回复）
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到 RETRY_REQ，说明对方没收到我们的帧，请求重传
 * 不需要回复，只需要触发重传逻辑
 */
/**
 * @brief 处理频点确认（FREQ_PAIR）
 * @param frame 应答帧
 * @param p_reply_buf 回复帧缓冲区（本函数不需要回复）
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到 FREQ_PAIR，说明对方确认了扫频后的频点
 * 载荷格式：[f0_hz(2)][f1_hz(2)]
 * 不需要回复，只需要保存频点信息
 */
static frame_handler_ret_t _handle_ask_freq_pair(const proto_frame_t *frame,
                                                  proto_frame_t *p_reply_buf,
                                                  uint8_t *p_reply_count)
{
    (void)p_reply_buf;
    
    /* 无需回复 */
    *p_reply_count = 0;
    
    /* TODO: 解析频点信息（小端）并保存 */
    if (frame->payload_len >= 4)
    {
        uint16_t f0_hz = (uint16_t)((frame->payload[1] << 8) | frame->payload[0]);
        uint16_t f1_hz = (uint16_t)((frame->payload[3] << 8) | frame->payload[2]);
        (void)f0_hz;  /* 避免未使用警告 */
        (void)f1_hz;  /* 避免未使用警告 */
    }
    
    return FRAME_HANDLER_OK;
}

/* ========== ALARM 类型帧处理 ========== */

/**
 * @brief 处理 ALARM 类型帧 - 使用函数表模式
 * @param frame 警报帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 为什么用函数表：
 * 1. 添加新 ALARM 子类型时只需在表中添加，无需修改此函数
 * 2. 每个子类型处理逻辑独立，便于单元测试
 * 3. 表项集中管理，便于查看所有支持的告警类型
 */
static frame_handler_ret_t _handle_alarm_frame(const proto_frame_t *frame,
                                               proto_frame_t *p_reply_buf,
                                               uint8_t *p_reply_count)
{
    /* 在处理表中查找对应的处理函数 */
    for (size_t i = 0; i < ALARM_HANDLER_TABLE_SIZE; i++)
    {
        if (s_alarm_handler_table[i].msg_num == frame->type_info.msg_num)
        {
            return s_alarm_handler_table[i].handler(frame, p_reply_buf, p_reply_count);
        }
    }
    
    /* 未找到对应的处理函数 */
    return FRAME_HANDLER_ERR_UNKNOWN_TYPE;
}

/**
 * @brief 处理电池低电告警
 * @param frame 告警帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到电池低电告警，响应处理
 * 载荷格式：[batt_pct(1)]
 */
static frame_handler_ret_t _handle_alarm_low_batt(const proto_frame_t *frame,
                                                   proto_frame_t *p_reply_buf,
                                                   uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 LOW_BATT 告警处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理温度超限告警
 * @param frame 告警帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到温度超限告警，响应处理
 * 载荷格式：[current_temp(2)][limit_type(1)]
 */
static frame_handler_ret_t _handle_alarm_temp_over(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 TEMP_OVER 告警处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理气压超限告警
 * @param frame 告警帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到气压超限告警，响应处理
 * 载荷格式：[current_press(2)][limit_type(1)]
 */
static frame_handler_ret_t _handle_alarm_press_over(const proto_frame_t *frame,
                                                     proto_frame_t *p_reply_buf,
                                                     uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 PRESS_OVER 告警处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理传感器故障告警
 * @param frame 告警帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到传感器故障告警，响应处理
 * 载荷格式：[fault_code(1)][fault_data(2)]
 */
static frame_handler_ret_t _handle_alarm_sensor_fault(const proto_frame_t *frame,
                                                       proto_frame_t *p_reply_buf,
                                                       uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 SENSOR_FAULT 告警处理 */
    return FRAME_HANDLER_OK;
}

/**
 * @brief 处理链路失败告警
 * @param frame 告警帧
 * @param p_reply_buf 回复帧缓冲区
 * @param p_reply_count 输出：回复帧数量
 * @return 处理结果
 * 
 * @details
 * 待实现：收到链路失败告警，响应处理
 * 载荷格式：[retry_count(1)]
 */
static frame_handler_ret_t _handle_alarm_link_fail(const proto_frame_t *frame,
                                                    proto_frame_t *p_reply_buf,
                                                    uint8_t *p_reply_count)
{
    (void)frame;
    (void)p_reply_buf;
    (void)p_reply_count;
    /* TODO: 实现 LINK_FAIL 告警处理 */
    return FRAME_HANDLER_OK;
}

/* ========== 通用辅助函数 ========== */

/**
 * @brief 判断帧是否是 ACK/NACK 确认帧
 * @param p_frame 待判断的帧
 * @return true 表示是 ACK/NACK，false 表示是普通数据帧
 * 
 * @details
 * ACK/NACK 帧特征：
 * - 类型为 PROTO_TYPE_ASK
 * - msg_num 为 PROTO_ASK_ACK 或 PROTO_ASK_NACK
 * 
 * 发送策略的区别：
 * - ACK/NACK：直接发送，不需要等待和重传（确认帧本身不需要确认）
 * - 其他帧：需要等待对方的 ACK 和自动重传保障
 */
static bool _is_ack_frame(const proto_frame_t *p_frame)
{
    if (p_frame == NULL)
    {
        return false;
    }
    
    if (p_frame->type_info.type == PROTO_TYPE_ASK)
    {
        return (p_frame->type_info.msg_num == PROTO_ASK_ACK ||
                p_frame->type_info.msg_num == PROTO_ASK_NACK);
    }
    
    return false;
}

/**
 * @brief 发送多个回复帧（智能选择发送方式）
 * @param p_frames 帧数组
 * @param count 帧数量
 * @return FRAME_HANDLER_OK 表示全部发送成功
 * 
 * @details
 * 智能发送策略：
 * - ACK/NACK 帧：调用 app_msg_send_ack()（直接发送，无需等待）
 * - 其他帧（查询应答、控制应答等）：调用 app_msg_send_frame()（等待 ACK，自动重传）
 * 
 * 为什么分类？
 * 1. ACK/NACK 只是确认收到，不需要二次确认（避免无限往复）
 * 2. 数据帧需要对方的 ACK 确认，才能确保可靠传输（重传保障）
 * 
 * @note 改进：相比旧设计，新增了 _is_ack_frame() 来识别确认帧，
 *       根据帧类型自动选择不同的发送策略，统一在此函数处理。
 */
static frame_handler_ret_t _send_frames(const proto_frame_t *p_frames, uint8_t count)
{
    if (p_frames == NULL || count == 0 || count > APP_FRAME_HANDLER_MAX_REPLY_FRAMES)
    {
        return FRAME_HANDLER_ERR_PARAM;
    }
    
    for (uint8_t i = 0; i < count; i++)
    {
        msg_send_ret_t send_ret;
        
        if (_is_ack_frame(&p_frames[i]))
        {
            /* ACK/NACK 帧：直接发送，不等待对方确认 */
            send_ret = app_msg_send_ack(&p_frames[i]);
        }
        else
        {
            /* 数据帧：需要等待 ACK 和重传保障（1000ms 超时） */
            send_ret = app_msg_send_frame(&p_frames[i], 1000);
        }
        
        if (send_ret != MSG_SEND_OK)
        {
            return FRAME_HANDLER_ERR_SEND_FAIL;
        }
    }
    
    return FRAME_HANDLER_OK;
}

/**
 * @brief 组装 ACK 回复帧（通用）
 * @param req_frame 原始请求帧
 * @param p_ack_frame 输出：ACK 帧
 * 
 * @details
 * ACK 帧格式：
 * - DST = 接收帧的 SRC（回复给发送方）
 * - SRC = 本机地址
 * - TYPE = PROTO_TYPE_ASK
 * - MSG_NUM = PROTO_ASK_ACK
 * - SEQ = 接收帧的 SEQ
 * - PAYLOAD = 无
 * 
 * @note 为什么目的地址是 req_frame->addr.src：
 *       ACK 回复必须发给原始发送方，无论是上级、下级还是广播帧
 */
static void _make_ack_frame(const proto_frame_t *req_frame, proto_frame_t *p_ack_frame)
{
    p_ack_frame->addr.dst = req_frame->addr.src;  /* 回复给原始发送方 */
    p_ack_frame->addr.src = bsw_node_id_get();
    p_ack_frame->type_info.type = PROTO_TYPE_ASK;
    p_ack_frame->type_info.msg_num = PROTO_ASK_ACK;
    p_ack_frame->seq = req_frame->seq;
    p_ack_frame->payload_len = 0;  /* ACK 无载荷 */
}

/**
 * @brief 组装 NACK 回复帧
 * @param req_frame 原始请求帧
 * @param err_code 错误码（见 proto_nack_code_t）
 * @param p_nack_frame 输出：NACK 帧
 * 
 * @details
 * NACK 帧格式：
 * - TYPE = PROTO_TYPE_ASK
 * - MSG_NUM = PROTO_ASK_NACK
 * - PAYLOAD[0] = err_code
 */
static void _make_nack_frame(const proto_frame_t *req_frame,
                             proto_nack_code_t err_code,
                             proto_frame_t *p_nack_frame)
{
    p_nack_frame->addr.dst = req_frame->addr.src;  /* 回复给发送方 */
    p_nack_frame->addr.src = bsw_node_id_get();
    p_nack_frame->type_info.type = PROTO_TYPE_ASK;
    p_nack_frame->type_info.msg_num = PROTO_ASK_NACK;
    p_nack_frame->seq = req_frame->seq;
    p_nack_frame->payload_len = 1;
    p_nack_frame->payload[0] = (uint8_t)err_code;
}
