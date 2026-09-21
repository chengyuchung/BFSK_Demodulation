/**
 * @file    CommManager.h
 * @brief   通信管理器 - 统一管理串口/网络连接
 */

#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include <QObject>
#include <QByteArray>
#include "UartThread.h"
#include "ProtocolFrame.h"

class CommManager : public QObject
{
    Q_OBJECT

public:
    static CommManager &instance(void);

    /* 连接 / 断开 */
    bool connectUart(const QString &port, qint32 baud = 115200);
    void disconnect(void);

    /* 发送帧 */
    void sendFrame(const ProtocolFrame &frame);

    /* 发送原始数据 */
    void sendRaw(const QByteArray &data);

signals:
    /* 帧接收 */
    void frameReceived(const ProtocolFrame &frame);

    /* 连接状态 */
    void connectionStatusChanged(bool connected, const QString &info);

    /* 错误 */
    void errorOccurred(const QString &msg);

    /* 日志 */
    void logReceived(const QString &msg);

private:
    explicit CommManager(QObject *parent = nullptr);
    ~CommManager() = default;
    CommManager(const CommManager &) = delete;
    CommManager &operator=(const CommManager &) = delete;

    UartThread          *m_uart;
    ProtocolFrameParser *m_parser;
};

#endif /* COMM_MANAGER_H */
