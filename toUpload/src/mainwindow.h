#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "pcl_fuction.h"    // PCL相关函数
#include "colorselection.h" // 选色器类
#include "view_rendering.h" // 点云渲染窗口类
#include "view_pointsizesetting.h"  // 单点尺寸窗口类
#include "filter_voxel.h"   // 体素滤波窗口类
#include "dialog_icp.h"     // ICP_SVD配准窗口类
#include "dialog_icptv.h"   // ICP_SVD配准窗口类（使用TreeView）

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
#define DEFAUT_PT_SIZE 1
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
    // 图标容器
    QMap<QString, QIcon> m_publicIconMap;

private:
    Ui::MainWindow *ui;
    // 点云对象
    PointCloudXYZ cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_vec; // 点云指针容器
    std::vector<int> cloud_index;   // 点云显示标识符容器
    struct rgbColor // 点云颜色
    {
        unsigned int r;
        unsigned int g;
        unsigned int b;
    };
    std::vector<rgbColor*> cloud_color; // 点云颜色容器
    std::vector<int> cloud_pointSize;   // 点云尺寸容器
    int point_size = 1;//点云大小
    // 点云对象指针
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr;
    // 显示器对象指针
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // 选色器对象指针
    colorSelection *colorPicker;
    // 树形控件相关对象
    QStandardItem *itemFolder;
    QStandardItemModel *model;
    int pickedIndex;    // 树形控件中选择的项目索引
    // 点云渲染窗口对象指针
    View_rendering *dialog_render;
    // 单点尺寸窗口对象指针
    View_pointSizeSetting *dialog_pointSizeSetting;
    // 体素滤波窗口对象指针
    Filter_Voxel *dialog_voxel;
    PointCloudXYZ::Ptr filter_cloud_out;
    // ICP_SVD配准窗口对象指针
    Dialog_icp *dialog_icp;
    Dialog_icpTV *dialog_icpTV;
    PointCloudXYZ src_cloud;
    PointCloudXYZ tgt_cloud;
    PointCloudXYZ::Ptr rst_cloud;
//--------------------------------------private funaction-----------------------------------------
    void initMainWidgetViewer();
    void initDBtree();
    void initIconMap();
    void viewer_update(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vector_cloud, std::vector<int> index);
    void initCloudColor();

//--------------------------------------slot funaction-----------------------------------------
private slots:
/*********************************************基本操作***********************************************/
    void on_openAction_triggered(); // 读取点云
    void on_saveAction_triggered(); // 保存点云
    void on_quitAction_triggered(); // 退出应用
    void on_backguandColorAction_triggered();   // 设置viewer背景颜色
    void on_DBtreeView_clicked(const QModelIndex &index);   // 控制树形控件
    void on_pointCloudColorAction_triggered();  // 设置点云颜色
    void on_pointCloudRenderAction_triggered(); // 启动点云渲染设置窗口
    void rendering_Setting(QString axis);   // 设置点云渲染字段（按照深度渲染）
    void on_pointSizeAction_triggered();    // 启动单点尺寸设置窗口
    void pointSize_Setting(int ptSize);     // 设置单点尺寸
    void on_filterVoxelAction_triggered();  // 启动体素滤波窗口
    void voxel_Filter(QString voxelSize);   // 进行体素滤波操作
    void on_ICP_SVDAction_triggered();      // 启动ICP_SVD配准窗口

    /** @brief 进行ICP_SVD配准操作.
          * @param[in] maxDistance 点对间最大配对距离
          * @param[in] transEpsilon 两次变换矩阵之间的差值
          * @param[in] fitnessEpsilon 前后两侧均方误差大小，当误差值小于这个值停止迭代
          * @param[in] maxIterations 最大迭代次数
          * @param[in] srcFilePath 源点云文件路径
          * @param[in] tgtFilePath 目标点云文件路径
          * @return void
          */
    void icp_svd_Registration(QString maxDistance,
                              QString transEpsilon,
                              QString fitnessEpsilon,
                              QString maxIterations,
                              QString srcFilePath,
                              QString tgtFilePath);

    /**
     * @brief on_ICP_SVD_TreeViewAction_triggered 启动ICP_SVD(TreeView)配准窗口
     */
    void on_ICP_SVD_TreeViewAction_triggered();

    /** @brief 进行ICP_SVD配准操作.
          * @param[in] maxDistance 点对间最大配对距离
          * @param[in] transEpsilon 两次变换矩阵之间的差值
          * @param[in] fitnessEpsilon 前后两侧均方误差大小，当误差值小于这个值停止迭代
          * @param[in] maxIterations 最大迭代次数
          * @param[in] srcIndex 源点云索引
          * @param[in] tgtIndex 目标点云索引
          * @return void
          */
    void icp_svd_treeView_Registration(QString maxDistance,
                                       QString transEpsilon,
                                       QString fitnessEpsilon,
                                       QString maxIterations,
                                       int srcIndex,
                                       int tgtIndex);

};
#endif // MAINWINDOW_H
