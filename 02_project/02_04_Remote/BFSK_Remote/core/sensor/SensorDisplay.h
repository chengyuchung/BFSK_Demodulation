/**
 * @file    SensorDisplay.h
 * @brief   传感器数据显示模块
 */

#ifndef SENSOR_DISPLAY_H
#define SENSOR_DISPLAY_H

#include <QObject>
#include "ProtocolFrame.h"

class SensorDisplay : public QObject
{
    Q_OBJECT

public:
    explicit SensorDisplay(QObject *parent = nullptr);

    /* 处理一帧数据 */
    void onFrameReceived(const ProtocolFrame &frame);

signals:
    void temperatureUpdated(int16_t value);      /* 温度（×0.01 °C） */
    void pressureUpdated(int32_t value);          /* 气压（Pa） */
    void signalStrengthUpdated(int rssi);          /* 信号强度 */
    void statusUpdated(const QString &status);    /* 节点状态字符串 */

private:
    /* 解析载荷数据 */
    void parseSensorPayload(const QByteArray &payload);
    void parseStatusPayload(const QByteArray &payload);
};

#endif /* SENSOR_DISPLAY_H */
