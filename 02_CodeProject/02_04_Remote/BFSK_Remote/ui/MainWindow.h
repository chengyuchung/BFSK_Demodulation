/**
 * @file    MainWindow.h
 * @brief   主窗口 - 远程终端 UI
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPortInfo>
#include "SensorDisplay.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class CommManager;
class SensorDisplay;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    /* 串口连接/断开 */
    void onBtnConnectClicked(void);
    void onBtnRefreshClicked(void);

    /* 发送命令 */
    void onBtnSendClicked(void);

    /* 数据更新 */
    void onTemperatureUpdated(int16_t value);
    void onPressureUpdated(int32_t value);
    void onSignalUpdated(int rssi);
    void onStatusUpdated(const QString &status);
    void onFrameReceived(const ProtocolFrame &frame);

    /* 连接状态 */
    void onConnectionChanged(bool connected, const QString &info);
    void onLogReceived(const QString &msg);

private:
    Ui::MainWindow  *ui;
    CommManager     *m_comm;
    SensorDisplay   *m_sensor;
    quint8           m_seqCounter;
};

#endif /* MAINWINDOW_H */
