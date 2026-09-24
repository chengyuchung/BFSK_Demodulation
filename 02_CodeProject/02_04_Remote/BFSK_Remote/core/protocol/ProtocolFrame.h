/**
 * @file    ProtocolFrame.h
 * @brief   帧协议 - 与 STM32 端 bsw_frame 完全对应
 * @note    Qt 侧负责组帧（发送）/ 解帧（接收）
 *          帧格式：FLAG + dst + src + type + ctrl + seq + payload + crc16 + FLAG
 */

#ifndef PROTOCOL_FRAME_H
#define PROTOCOL_FRAME_H

#include <QObject>
#include <QByteArray>
#include <QList>

/* ========== 宏定义（与 STM32 端一致） ========== */
#define FRAME_FLAG       0x7E
#define FRAME_ESC       0x7D
#define FRAME_STUFF_XOR 0x20

/* ========== 消息类型 ========== */
typedef enum {
    MSG_TYPE_DATA        = 0x01,  /* 传感器数据 */
    MSG_TYPE_STATUS      = 0x02,  /* 节点状态 */
    MSG_TYPE_CMD         = 0x03,  /* 控制命令 */
    MSG_TYPE_ACK         = 0x04,  /* 应答 */
    MSG_TYPE_SCAN_START  = 0x05,  /* 扫频开始 */
    MSG_TYPE_SCAN_RESULT = 0x06,  /* 扫频结果 */
    MSG_TYPE_HEARTBEAT   = 0x07   /* 心跳 */
} ProtocolMsgType;

/* ========== 帧结构 ========== */
struct ProtocolFrame {
    quint8  dst;           /* 目标地址 */
    quint8  src;           /* 源地址 */
    quint8  msgType;       /* 消息类型 */
    quint8  control;       /* 控制字段 */
    quint8  seq;           /* 序号 */
    QByteArray payload;    /* 载荷 */
};

/* ========== 协议解析状态 ========== */
enum class ParseState {
    Idle,
    Data,
    Escape,
    Complete
};

/* ========== 类 ========== */
class ProtocolFrameParser : public QObject
{
    Q_OBJECT

public:
    explicit ProtocolFrameParser(QObject *parent = nullptr);

    /* 逐字节输入，返回完整帧列表 */
    QList<ProtocolFrame> input(const QByteArray &data);

    /* 组帧：将一帧打包为字节流 */
    QByteArray pack(const ProtocolFrame &frame);

signals:
    void frameReceived(const ProtocolFrame &frame);

private:
    ParseState  m_state;
    QByteArray  m_buf;
    quint8      m_crcHigh;
    bool        m_inEscape;
};

#endif /* PROTOCOL_FRAME_H */
