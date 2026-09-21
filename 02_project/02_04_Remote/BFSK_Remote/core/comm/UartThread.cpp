/* UartThread.cpp */
#include "UartThread.h"
#include <QSerialPortInfo>
#include <QDebug>

UartThread::UartThread(QObject *parent)
    : QThread(parent)
    , m_port(new QSerialPort(this))
    , m_running(false)
{
}

UartThread::~UartThread()
{
    close();
    wait();
}

bool UartThread::open(const QString &portName, qint32 baudRate)
{
    m_port->setPortName(portName);
    m_port->setBaudRate(baudRate);
    m_port->setDataBits(QSerialPort::Data8);
    m_port->setParity(QSerialPort::NoParity);
    m_port->setStopBits(QSerialPort::OneStop);
    m_port->setFlowControl(QSerialPort::NoFlowControl);

    if (!m_port->open(QIODevice::ReadWrite)) {
        emit errorOccurred("无法打开串口: " + portName);
        return false;
    }

    m_running = true;
    start();
    emit connectionChanged(true);
    return true;
}

void UartThread::close(void)
{
    m_running = false;
    if (m_port->isOpen()) {
        m_port->close();
        emit connectionChanged(false);
    }
}

bool UartThread::isOpen(void) const
{
    return m_port->isOpen();
}

void UartThread::send(const QByteArray &data)
{
    if (!m_port->isOpen()) return;
    m_port->write(data);
    m_port->flush();
}

void UartThread::run(void)
{
    QByteArray readBuf;
    while (m_running) {
        if (m_port->waitForReadyRead(10)) {
            readBuf += m_port->readAll();
            if (!readBuf.isEmpty()) {
                emit dataReceived(readBuf);
                readBuf.clear();
            }
        }
        /* 处理发送缓冲 */
        m_mutex.lock();
        if (!m_sendBuf.isEmpty()) {
            m_port->write(m_sendBuf);
            m_port->flush();
            m_sendBuf.clear();
        }
        m_mutex.unlock();
    }
}
