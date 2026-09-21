/* SensorDisplay.cpp */
#include "SensorDisplay.h"

SensorDisplay::SensorDisplay(QObject *parent)
    : QObject(parent)
{
}

void SensorDisplay::onFrameReceived(const ProtocolFrame &frame)
{
    switch (frame.msgType) {
        case MSG_TYPE_DATA:    parseSensorPayload(frame.payload);  break;
        case MSG_TYPE_STATUS:  parseStatusPayload(frame.payload); break;
        case MSG_TYPE_HEARTBEAT: /* 心跳，忽略 */                 break;
        default:                                                  break;
    }
}

void SensorDisplay::parseSensorPayload(const QByteArray &payload)
{
    /* 载荷格式示例（由 STM32 侧 app_sensor_task 填充）：
     * [0-1]   int16_t  temperature × 100
     * [2-5]   int32_t  pressure Pa
     * [6]     quint8   rssi
     * [7]     quint8   node_addr
     */
    if (payload.size() < 8) return;
    const quint8 *d = (const quint8 *)payload.constData();

    int16_t temp = d[0] | (d[1] << 8);
    int32_t press = d[2] | (d[3] << 8) | (d[4] << 16) | (d[5] << 24);
    int rssi = d[6];

    emit temperatureUpdated(temp);
    emit pressureUpdated(press);
    emit signalStrengthUpdated(rssi);
}

void SensorDisplay::parseStatusPayload(const QByteArray &payload)
{
    if (payload.isEmpty()) return;
    QString status = QString::fromUtf8(payload);
    emit statusUpdated(status);
}
