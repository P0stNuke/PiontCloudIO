#include "mainwindow.h"

#include <QApplication>
//创建菜单栏，工具栏，状态栏应当包含的头文件
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QStatusBar>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    w.setWindowTitle("PCL software");
    return a.exec();
}
