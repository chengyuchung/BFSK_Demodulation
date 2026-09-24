/**
 * @file    main.cpp
 * @brief   远程终端主程序入口
 */

#include "MainWindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    /* 设置应用信息 */
    QApplication::setApplicationName("BFSK_Remote");
    QApplication::setApplicationVersion("1.0.0");
    QApplication::setOrganizationName("BFSK_Demodulation");

    MainWindow w;
    w.show();

    return a.exec();
}
