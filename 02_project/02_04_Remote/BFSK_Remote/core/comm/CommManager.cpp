/* CommManager.cpp */
#include "CommManager.h"
#include <QDebug>

CommManager::CommManager(QObject *parent)
    : QObject(parent)
    , m_uart(new UartThread(this))
    , m_parser(new ProtocolFrameParser(this))
{
    connect(m_uart, &UartThread::dataReceived,
            this, [this](const QByteArray &data) {
                auto frames = m_parser->input(data);
                for (const auto &f : frames) {
                    emit frameReceived(f);
                }
            });

    connect(m_uart, &UartThread::connectionChanged,
            this, [this](bool connected) {
                emit connectionStatusChanged(connected, connected ? "已连接" : "已断开");
            });

    connect(m_uart, &UartThread::errorOccurred,
            this, [this](const QString &err) {
                emit errorOccurred(err);
            });
}

CommManager &CommManager::instance(void)
{
    static CommManager inst;
    return inst;
}

bool CommManager::connectUart(const QString &port, qint32 baud)
{
    return m_uart->open(port, baud);
}

void CommManager::disconnect(void)
{
    m_uart->close();
}

void CommManager::sendFrame(const ProtocolFrame &frame)
{
    QByteArray data = m_parser->pack(frame);
    sendRaw(data);
}

void CommManager::sendRaw(const QByteArray &data)
{
    m_uart->send(data);
}
