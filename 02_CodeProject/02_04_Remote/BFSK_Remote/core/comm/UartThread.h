/**
 * @file    UartThread.h
 * @brief   串口通信线程
 * @note    在独立线程中运行 QSerialPort，避免阻塞 UI
 */

#ifndef UART_THREAD_H
#define UART_THREAD_H

#include <QThread>
#include <QByteArray>
#include <QSerialPort>
#include <QMutex>

class UartThread : public QThread
{
    Q_OBJECT

public:
    explicit UartThread(QObject *parent = nullptr);
    ~UartThread();

    /* 打开串口 */
    bool open(const QString &portName, qint32 baudRate);

    /* 关闭串口 */
    void close(void);

    /* 发送数据 */
    void send(const QByteArray &data);

    /* 检查是否已打开 */
    bool isOpen(void) const;

signals:
    /* 数据接收信号 */
    void dataReceived(const QByteArray &data);

    /* 连接状态变化 */
    void connectionChanged(bool connected);

    /* 错误信息 */
    void errorOccurred(const QString &err);

protected:
    void run(void) override;

private:
    QSerialPort *m_port;
    QByteArray   m_sendBuf;
    QMutex       m_mutex;
    bool         m_running;
};

#endif /* UART_THREAD_H */
