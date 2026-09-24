QT       += core gui serialport network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

SOURCES += \
    main.cpp \
    ui/MainWindow.cpp \
    core/protocol/ProtocolFrame.cpp \
    core/protocol/ProtocolCRC.cpp \
    core/comm/CommManager.cpp \
    core/comm/UartThread.cpp \
    core/sensor/SensorDisplay.cpp

HEADERS += \
    ui/MainWindow.h \
    core/protocol/ProtocolFrame.h \
    core/protocol/ProtocolCRC.h \
    core/comm/CommManager.h \
    core/comm/UartThread.h \
    core/sensor/SensorDisplay.h

FORMS += \
    ui/MainWindow.ui

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
