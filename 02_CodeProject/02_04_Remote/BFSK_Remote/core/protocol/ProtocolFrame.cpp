/* ProtocolFrame.cpp */
#include "ProtocolFrame.h"
#include "ProtocolCRC.h"

ProtocolFrameParser::ProtocolFrameParser(QObject *parent)
    : QObject(parent)
    , m_state(ParseState::Idle)
    , m_inEscape(false)
{
}

QByteArray ProtocolFrameParser::pack(const ProtocolFrame &frame)
{
    /* 组装原始数据 */
    QByteArray raw;
    raw.append(frame.dst);
    raw.append(frame.src);
    raw.append(frame.msgType);
    raw.append(frame.control);
    raw.append(frame.seq);
    raw.append(frame.payload);

    /* 计算 CRC */
    quint16 crc = ProtocolCRC::calc16((const quint8 *)raw.constData(), raw.size());

    /* 字节填充 + FLAG */
    QByteArray stuffed;
    stuffed.append(FRAME_FLAG);
    for (int i = 0; i < raw.size(); i++) {
        quint8 b = (quint8)raw[i];
        if (b == FRAME_FLAG) {
            stuffed.append(FRAME_ESC);
            stuffed.append(b ^ FRAME_STUFF_XOR);
        } else if (b == FRAME_ESC) {
            stuffed.append(FRAME_ESC);
            stuffed.append(b ^ FRAME_STUFF_XOR);
        } else {
            stuffed.append(b);
        }
    }
    /* CRC 两字节 */
    quint8 crc_hi = (crc >> 8) & 0xFF;
    quint8 crc_lo = crc & 0xFF;
    for (int i = 0; i < 2; i++) {
        quint8 b = (i == 0) ? crc_hi : crc_lo;
        if (b == FRAME_FLAG) {
            stuffed.append(FRAME_ESC);
            stuffed.append(b ^ FRAME_STUFF_XOR);
        } else if (b == FRAME_ESC) {
            stuffed.append(FRAME_ESC);
            stuffed.append(b ^ FRAME_STUFF_XOR);
        } else {
            stuffed.append(b);
        }
    }
    stuffed.append(FRAME_FLAG);
    return stuffed;
}

QList<ProtocolFrame> ProtocolFrameParser::input(const QByteArray &data)
{
    QList<ProtocolFrame> result;
    for (int i = 0; i < data.size(); i++) {
        quint8 byte = (quint8)data[i];

        if (m_state == ParseState::Idle) {
            if (byte == FRAME_FLAG) {
                m_buf.clear();
                m_inEscape = false;
                m_state = ParseState::Data;
            }
        } else if (m_state == ParseState::Data) {
            if (byte == FRAME_FLAG) {
                /* 帧结束，解析 */
                if (m_buf.size() >= 5) {
                    ProtocolFrame f;
                    f.dst      = (quint8)m_buf[0];
                    f.src      = (quint8)m_buf[1];
                    f.msgType  = (quint8)m_buf[2];
                    f.control  = (quint8)m_buf[3];
                    f.seq      = (quint8)m_buf[4];
                    f.payload  = m_buf.mid(5);
                    result.append(f);
                    emit frameReceived(f);
                }
                m_state = ParseState::Idle;
            } else if (byte == FRAME_ESC) {
                m_inEscape = true;
            } else {
                quint8 b = m_inEscape ? (byte ^ FRAME_STUFF_XOR) : byte;
                m_buf.append(b);
                m_inEscape = false;
            }
        }
    }
    return result;
}
