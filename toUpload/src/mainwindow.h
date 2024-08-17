#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "pcl_fuction.h"    // PCL相关函数
#include "colorselection.h" // 选色器类

//-----------------------------vtk---------------------------------
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderWindow.h>
#include <vtkAutoInit.h>
#include <vtkGenericOpenGLRenderWindow.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType)
//-----------------------------vtk---------------------------------
//-----------------------------Qt---------------------------------
#include <QDebug>
#include <QColorDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QTime>
#include <QDir>
#include <QFile>
#include <QtMath>
#include <QDirIterator>
#include <QVector>
#include <QMap>
#include <QStandardItem>
#include <QIcon>
//-----------------------------Qt---------------------------------

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    // 定义图标容器
    QMap<QString, QIcon> m_publicIconMap;

private:
    Ui::MainWindow *ui;
    // 定义点云对象
    // QVector<pcl::shared_ptr<PointCloudXYZ>> cloud_vec;
    // QVector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec;
    // QVector<int> cloud_index;
    std::vector<int> cloud_index;
    PointCloudXYZ cloud;
    std::vector<std::string> cloud_name{"0", "1", "2"};//点云名称
    int point_size = 1;//点云大小
    // 定义点云对象指针
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr;
    // 定义显示器
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // 定义选色器对象指针
    colorSelection *colorPicker;
    // 定义树形控件相关对象
    QStandardItem *itemFolder;
    QStandardItemModel *model;
//--------------------------------------private funaction-----------------------------------------
    void initMainWidgetViewer();
    void initDBtree();
    void initIconMap();
    void viewer_update(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vector_cloud, std::vector<int> index);

//--------------------------------------slot funaction-----------------------------------------
private slots:
/*********************************************基本操作***********************************************/
    void on_openAction_triggered(); // 读取点云
    void on_saveAction_triggered(); // 保存点云
    void on_quitAction_triggered(); // 退出应用
    void on_backguandColorAction_triggered();   // 设置viewer背景颜色
    void on_DBtreeView_clicked(const QModelIndex &index);
};
#endif // MAINWINDOW_H
