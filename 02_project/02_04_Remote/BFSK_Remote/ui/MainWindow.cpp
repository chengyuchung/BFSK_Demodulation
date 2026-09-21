/* MainWindow.cpp */
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include "CommManager.h"
#include "SensorDisplay.h"
#include <QMessageBox>
#include <QDateTime>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , m_comm(&CommManager::instance())
    , m_sensor(new SensorDisplay(this))
    , m_seqCounter(0)
{
    ui->setupUi(this);

    setWindowTitle("BFSK 远程终端 v1.0");

    /* 刷新串口列表 */
    onBtnRefreshClicked();

    /* 连接信号槽 */
    connect(m_comm, &CommManager::connectionStatusChanged,
            this, &MainWindow::onConnectionChanged);
    connect(m_comm, &CommManager::frameReceived,
            this, &MainWindow::onFrameReceived);
    connect(m_comm, &CommManager::errorOccurred,
            this, [this](const QString &msg) {
                ui->textLog->append("[错误] " + msg);
            });

    connect(m_sensor, &SensorDisplay::temperatureUpdated,
            this, &MainWindow::onTemperatureUpdated);
    connect(m_sensor, &SensorDisplay::pressureUpdated,
            this, &MainWindow::onPressureUpdated);
    connect(m_sensor, &SensorDisplay::signalStrengthUpdated,
            this, &MainWindow::onSignalUpdated);
    connect(m_sensor, &SensorDisplay::statusUpdated,
            this, &MainWindow::onStatusUpdated);

    connect(ui->btnConnect, &QPushButton::clicked,
            this, &MainWindow::onBtnConnectClicked);
    connect(ui->btnRefresh, &QPushButton::clicked,
            this, &MainWindow::onBtnRefreshClicked);
    connect(ui->btnSend, &QPushButton::clicked,
            this, &MainWindow::onBtnSendClicked);

    ui->textLog->setReadOnly(true);
}

MainWindow::~MainWindow()
{
    m_comm->disconnect();
    delete ui;
}

void MainWindow::onBtnRefreshClicked(void)
{
    ui->comboPort->clear();
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        QString desc = info.description();
        if (desc.isEmpty()) desc = "(无描述)";
        ui->comboPort->addItem(info.portName() + " - " + desc, info.portName());
    }
}

void MainWindow::onBtnConnectClicked(void)
{
    if (m_comm->isConnected()) {
        m_comm->disconnect();
        return;
    }

    int idx = ui->comboPort->currentIndex();
    if (idx < 0) {
        QMessageBox::warning(this, "提示", "请选择串口");
        return;
    }

    QString portName = ui->comboPort->itemData(idx).toString();
    if (!m_comm->connectUart(portName, ui->spinBaud->value())) {
        QMessageBox::critical(this, "错误", "无法打开串口 " + portName);
    }
}

void MainWindow::onConnectionChanged(bool connected, const QString &info)
{
    ui->btnConnect->setText(connected ? "断开" : "连接");
    ui->comboPort->setEnabled(!connected);
    ui->spinBaud->setEnabled(!connected);
    ui->labelStatus->setText(info);
    onLogReceived("[" + QTime::currentTime().toString("hh:mm:ss") + "] " + info);
}

void MainWindow::onBtnSendClicked(void)
{
    QString cmd = ui->editCmd->text().trimmed();
    if (cmd.isEmpty()) return;

    ProtocolFrame frame;
    frame.dst     = ui->spinDst->value();
    frame.src     = 0x00;  /* 本机地址 */
    frame.msgType = MSG_TYPE_CMD;
    frame.control = 0;
    frame.seq     = m_seqCounter++;
    frame.payload = cmd.toUtf8();

    m_comm->sendFrame(frame);
    ui->editCmd->clear();
    onLogReceived("[发送] " + cmd);
}

void MainWindow::onFrameReceived(const ProtocolFrame &frame)
{
    m_sensor->onFrameReceived(frame);

    QString timeStr = QTime::currentTime().toString("hh:mm:ss");
    QString s = QString("[%1] 收到帧 src=0x%2 type=0x%3 seq=%4 len=%5")
                    .arg(timeStr)
                    .arg(frame.src, 2, 16, QChar('0'))
                    .arg(frame.msgType, 2, 16, QChar('0'))
                    .arg(frame.seq)
                    .arg(frame.payload.size());
    ui->textLog->append(s);
}

void MainWindow::onTemperatureUpdated(int16_t value)
{
    double temp = value / 100.0;
    ui->labelTemp->setText(QString::number(temp, 'f', 2) + " °C");
}

void MainWindow::onPressureUpdated(int32_t value)
{
    double kpa = value / 1000.0;
    ui->labelPressure->setText(QString::number(kpa, 'f', 2) + " kPa");
}

void MainWindow::onSignalUpdated(int rssi)
{
    ui->labelRssi->setText(QString::number(rssi) + " dBm");
}

void MainWindow::onStatusUpdated(const QString &status)
{
    ui->labelNodeStatus->setText(status);
}

void MainWindow::onLogReceived(const QString &msg)
{
    ui->textLog->append(msg);
}
