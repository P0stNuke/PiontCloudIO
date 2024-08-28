#include "mainwindow.h"
#include "qobjectdefs.h"
#include "ui_mainwindow.h"
#include <QStandardItemModel>
#include <filesystem>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    qDebug() << "currentPath:" <<QDir::currentPath();
    /*************************************初始化*************************************/
    // 初始化PCL显示控件
    initMainWidgetViewer();
    // 初始化树形控件
    initDBtree();
    // 初始化图标容器
    initIconMap();
    initCloudColor();
    pickedIndex = -1;
    ui->saveAction->setEnabled(false);
    //***********************************connect设置*******************************************//
    //------------------------------------------------------------------
}

MainWindow::~MainWindow()
{
    delete ui;
}

QDebug operator<<(QDebug dbg, const Eigen::Matrix4f &mat)
{
    dbg.nospace() << "Matrix4f:\n";
    for (int i = 0; i < mat.rows(); ++i) {
        dbg.nospace() << "[ ";
        for (int j = 0; j < mat.cols(); ++j) {
            dbg.nospace() << mat(i, j) << " ";
        }
        dbg.nospace() << "]\n";
    }
    return dbg;
}
//*************************************initialization*************************************//
void MainWindow::initMainWidgetViewer()
{
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer,renderWindow,"viewer",false));
    ui->mainWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->mainWidget->interactor(), ui->mainWidget->renderWindow());
    viewer->setBackgroundColor(25.0/255, 25.0/255 , 25.0/255);
    ui->mainWidget->update();
}

void MainWindow::initDBtree()
{
    model = new QStandardItemModel(this);
    //ui->DBtreeView->setHeaderHidden(true);  //设置表头隐藏
    model->setHorizontalHeaderLabels(QStringList() << "--Cloud--DB--Tree--");  // 设置水平表头标题
    ui->DBtreeView->setModel(model);    //设置model
    ui->DBtreeView->expandAll();    //设置展开
    // 设置允许多选
    ui->DBtreeView->setSelectionMode(QAbstractItemView::ExtendedSelection); //支持shift, ctrl, 鼠标框选等方式
}

void MainWindow::initIconMap()
{
    QIcon treeItem_cloud("../../resource/icons/cloud.png");
    m_publicIconMap.insert(QStringLiteral("treeItem_cloud"), treeItem_cloud);
}

void MainWindow::initCloudColor()
{
    rgbColor *color1Red = new rgbColor{255, 0, 0};
    rgbColor *color2Green = new rgbColor{0, 255, 0};
    rgbColor *color3Blue = new rgbColor{0, 0, 255};
    cloud_color.push_back(color1Red);
    cloud_color.push_back(color2Green);
    cloud_color.push_back(color3Blue);
}
//*************************************initialization*************************************//

//***********************************fileManu*******************************************//
//点云读取txt文件
void CreateCloudFromTxt(const std::string& file_path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::ifstream file(file_path.c_str());
    std::string line;
    pcl::PointXYZ point;
    //float nx, ny, nz;
    while (getline(file, line)) {
        std::stringstream ss(line);
        ss >> point.x;
        ss >> point.y;
        ss >> point.z;
        //ss >> nx;
        //ss >> ny;
        //ss >> nz;
        cloud->push_back(point);
    }
    file.close();
}

void MainWindow::on_openAction_triggered()
{
    QString filePath = QFileDialog::getOpenFileName(this, tr("open  file"),
                                                    "E://Qt_project//00pointCloud",
                                                    tr("pcb files(*.pcd *.ply *.txt) ;;All files (*.*)"));
    std::string filePathStdStr = filePath.toStdString();
    // 获取文件名
    std::filesystem::path temp_Path(filePathStdStr);
    QString fileName = QString::fromStdString(temp_Path.filename().string());

    if(filePath.isEmpty())
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if(filePath.endsWith("ply"))
    {
        qDebug()<<filePath;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(filePathStdStr, *new_cloud) == -1) //* load the file
        {
            qDebug()<<"Couldn't read file  \n";
            return ;
        }
    }
    else if (filePath.endsWith("pcd"))
    {
        qDebug()<<filePath;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(filePathStdStr, *new_cloud) == -1) //* load the file
        {
            qDebug()<<"Couldn't read pcd file  \n";
            return ;
        }
    }
    else if (filePath.endsWith("txt"))
    {
        qDebug()<<filePath;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
        CreateCloudFromTxt(filePathStdStr,new_cloud);
        // cloud=*cloudTemp;
    }
    else {
        QMessageBox::warning(this, "Warning","点云读取格式错误！");
    }


    cloud_vec.push_back(new_cloud);
    cloud_index.push_back(1);
    cloud_pointSize.push_back(DEFAUT_PT_SIZE);

    // itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],QStringLiteral("cloud%1").arg(cloud_vec.size()-1));
    itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],
                                   fileName);
    itemFolder->setCheckable(true);
    itemFolder->setCheckState(Qt::Checked);//获取选中状态
    model->appendRow(itemFolder);


    // 移除窗口点云
    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    std::string cloud_id = std::to_string(cloud_vec.size() - 1);
    int temp_id = cloud_vec.size() - 1;
    if(temp_id < 3)
    {
        // 如果点云数量小于3则使用预设rgb颜色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_vec[temp_id],
                                                                                 cloud_color[temp_id]->r,
                                                                                 cloud_color[temp_id]->g,
                                                                                 cloud_color[temp_id]->b);
        // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud_vec[temp_id]);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_vec[temp_id], colorHandler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);
    }
    else
    {
        // 如果点云数量大于3使用白色
        rgbColor *colorex = new rgbColor{255, 255, 255};
        cloud_color.push_back(colorex);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_vec[temp_id],
                                                                                     cloud_color[temp_id]->r,
                                                                                     cloud_color[temp_id]->g,
                                                                                     cloud_color[temp_id]->b);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_vec[temp_id], colorHandler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);
    }
    viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();    // 强制立即刷新viewer
    ui->mainWidget->update();               // 确保 Qt 窗口组件也同步刷新

    ui->saveAction->setEnabled(true);
}


void MainWindow::on_saveAction_triggered()
{
    if(pickedIndex == -1)
    {
        QMessageBox::information(this, "无选中的点云", "请先在树状图中选中一个点云");
        return;
    }
    if(cloud_vec[pickedIndex]->empty())
    {
        QMessageBox::information(this, "注意", "点云指针错误");
        return;
    }
    else
    {
        QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "", tr ("*.pcd;;*.ply"));

        if (filename.isEmpty ())
            return;
        int return_status;
        if (filename.endsWith (".pcd", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_vec[pickedIndex]);
        else if (filename.endsWith (".ply", Qt::CaseInsensitive))
            return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud_vec[pickedIndex]);
        else
        {
            filename.append(".pcd");
            return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud_vec[pickedIndex]);
        }
        if (return_status != 0)
        {
            PCL_ERROR("Error writing point cloud %s\n", filename.toStdString ().c_str ());
            return;
        }
    }
}


void MainWindow::on_quitAction_triggered()
{
    this->close();
}
//***********************************fileManu*******************************************//


//***********************************viewerManu*******************************************//
// 背景颜色
void MainWindow::on_backguandColorAction_triggered()
{
    colorPicker = new colorSelection();
    QColor color = colorPicker->getColor();
    viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
    return;
}
// 点云颜色
void MainWindow::on_pointCloudColorAction_triggered()
{
    if(cloud_vec.size() == 0)
    {
        QMessageBox::information(this, "无点云输入","请先读取点云文件");
        return;
    }
    else if(pickedIndex == -1)
    {
        QMessageBox::information(this, "无选中的点云", "请先在树状图中选中一个点云");
        return;
    }
    colorPicker =new colorSelection();
    QColor color=colorPicker->getColor();
    QColor temp;
    std::string cloud_id = std::to_string(pickedIndex);
    int idx = pickedIndex;
    temp.setRgb(25,25,25,255);//防止与背景串色
    if(!cloud_vec[idx]->empty() && (color!=temp) )
    {
        cloud_color[idx]->r = color.redF()*255;
        cloud_color[idx]->g = color.greenF()*255;
        cloud_color[pickedIndex]->b = color.blueF()*255;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ >slected_color(cloud_vec[idx],
                                                                                      cloud_color[idx]->r,
                                                                                      cloud_color[idx]->g,
                                                                                      cloud_color[idx]->b);
        viewer->updatePointCloud(cloud_vec[idx], slected_color, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                 cloud_pointSize[pickedIndex],
                                                 cloud_id);
    }
    // else
    // {
    //     cloud_color[idx]->r = 255;
    //     cloud_color[idx]->g = 255;
    //     cloud_color[idx]->b = 255;

    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ >slected_color(cloud_vec[idx],255,255,255);
    //     viewer->updatePointCloud(cloud_vec[idx],slected_color, cloud_id);
    // }
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
    return ;
}
// 点云渲染
void MainWindow::on_pointCloudRenderAction_triggered()
{
    if(cloud_vec.size() == 0)
    {
        QMessageBox::information(this, "无点云输入","请先读取点云文件");
        return;
    }
    else if(pickedIndex == -1)
    {
        QMessageBox::information(this, "无选中的点云", "请先在树状图中选中一个点云");
        return;
    }
    dialog_render = new View_rendering();
    connect(dialog_render, &View_rendering::sendData,
            this, &MainWindow::rendering_Setting);
    if(dialog_render->exec() == QDialog::Accepted){}
    delete dialog_render;
}
void MainWindow::rendering_Setting(QString axis)
{
    std::string cloud_id = std::to_string(pickedIndex);
    int idx = pickedIndex;
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ > render (cloud_vec[idx],
                                                                                  axis.toStdString());
    viewer->updatePointCloud(cloud_vec[idx], render, cloud_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             cloud_pointSize[idx],
                                             cloud_id);
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
}
// 单点尺寸
void MainWindow::on_pointSizeAction_triggered()
{
    if(cloud_vec.size() == 0)
    {
        QMessageBox::information(this, "无点云输入","请先读取点云文件");
        return;
    }
    else if(pickedIndex == -1)
    {
        QMessageBox::information(this, "无选中的点云", "请先在树状图中选中一个点云");
        return;
    }
    dialog_pointSizeSetting = new View_pointSizeSetting();
    connect(dialog_pointSizeSetting,SIGNAL(sendData(int)),
            this,SLOT(pointSize_Setting(int)));

    if(dialog_pointSizeSetting->exec()==QDialog::Accepted){}

    delete dialog_pointSizeSetting;
}
void MainWindow::pointSize_Setting(int ptSize)
{
    std::string cloud_id = std::to_string(pickedIndex);
    int idx = pickedIndex;

    cloud_pointSize[idx] = ptSize;
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             ptSize,
                                             cloud_id);
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
}
//***********************************viewerManu*******************************************//


//***********************************DBTreaView*******************************************//
void MainWindow::viewer_update(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vector_cloud,std::vector<int> index)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    for(int i=0;i<vector_cloud.size();i++)
    {
        if(index[i]==1)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(vector_cloud[i],
                                                                                         cloud_color[i]->r,
                                                                                         cloud_color[i]->g,
                                                                                         cloud_color[i]->b);

            viewer->addPointCloud<pcl::PointXYZ>(vector_cloud[i], colorHandler, std::to_string(i));
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                     cloud_pointSize[i],
                                                     std::to_string(i));
        }
    }
    //viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();    // 强制立即刷新viewer
    ui->mainWidget->update();               // 确保 Qt 窗口组件也同步刷新
}



void MainWindow::on_DBtreeView_clicked(const QModelIndex &index)
{
    QStandardItem* item = model->itemFromIndex(index);

    //---------------点数数量更改------------------

    // QStandardItemModel* model = static_cast<QStandardItemModel*>(ui->DBtreeView->model());
    QModelIndex index_temp = ui->DBtreeView->currentIndex();

    pickedIndex = index_temp.row(); // 赋值给私有成员方便其他方法调用

    int size = static_cast<int>(cloud_vec[index_temp.row()]->size());
    QString PointSize = QString("%1").arg(size);

    ui->textBrowser_2->clear();
    ui->textBrowser_2->insertPlainText("点云数量: "+PointSize);
    ui->textBrowser_2->setFont(QFont( "Arial" , 9 ,QFont::Normal ));


    //--------------可视化更改---------------------

    if(item == nullptr)
        return;
    if(item->isCheckable())
    {

        //判断状态
        Qt::CheckState state = item->checkState();//获取当前的选择状态

        if(Qt::Checked==state)
        {
            cloud_index[index.row()]=1;

        }

        if(Qt::Unchecked==state)
        {
            cloud_index[index.row()]=0;
        }

        viewer_update(cloud_vec,cloud_index);

    }
}
//***********************************DBTreaView*******************************************//

//***********************************FilterManu*******************************************//
// 体素采样 对同一点云多次应用体素滤波时程序会崩溃，暂未找到解决办法
void MainWindow::on_filterVoxelAction_triggered()
{
    if(cloud_vec.size() == 0)
    {
        QMessageBox::information(this, "无点云输入","请先读取点云文件");
        return;
    }
    else if(pickedIndex == -1)
    {
        QMessageBox::information(this, "无选中的点云", "请先在树状图中选中一个点云");
        return;
    }
    dialog_voxel = new Filter_Voxel();
    connect(dialog_voxel, SIGNAL(sendSizeofVoxel(QString)),
            this, SLOT(voxel_Filter(QString)));
    if(dialog_voxel->exec() == QDialog::Accepted){}
    delete dialog_voxel;
}
void MainWindow::voxel_Filter(QString voxelSize)
{
    if(voxelSize.isEmpty())
    {
        QMessageBox::warning(this, "Warning","参数格式输入错误");
        return;
    }
    // 检查指针是否有效
    if (!cloud_vec[pickedIndex] || cloud_vec[pickedIndex]->empty())
    {
        QMessageBox::warning(this, "Warning", "点云数据无效或为空");
        return;
    }

    std::string cloud_id = std::to_string(pickedIndex);
    int idx = pickedIndex;

    float size=voxelSize.toFloat();
    // 创建新的点云指针以存储滤波后的点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_filter_voxel(cloud_vec[idx], cloud_out, size);
    qDebug() << "Applying voxel filter with leaf size:" << size;
    qDebug() << "Before filtering, point cloud size:" << cloud_vec[idx]->size();
    // 替换掉当前索引处的点云对象
    // cloud_vec[idx].reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_vec[idx] = cloud_out;

    ui->textBrowser_2->clear();
    int size1 = static_cast<int>(cloud_vec[idx]->size());
    QString PointSize = QString("%1").arg(size1);
    ui->textBrowser_2->insertPlainText("点云数量: "+PointSize);
    ui->textBrowser_2->setFont(QFont( "Arial" , 9 ,QFont::Normal ));

    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    viewer->removePointCloud(cloud_id);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_vec[idx],
                                                                                 cloud_color[idx]->r,
                                                                                 cloud_color[idx]->g,
                                                                                 cloud_color[idx]->b);
    viewer->addPointCloud(cloud_vec[idx], colorHandler, cloud_id);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             cloud_pointSize[idx],
                                             cloud_id);
    // viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
}
//***********************************FilterManu*******************************************//

//***********************************RegistrationManu*******************************************//
void MainWindow::on_ICP_SVDAction_triggered()
{
    dialog_icp=new Dialog_icp();

    connect(dialog_icp,SIGNAL(sendData(QString,QString,QString,QString,QString,QString)),
            this,SLOT(icp_svd_Registration(QString,QString,QString,QString,QString,QString)));

    if(dialog_icp->exec()==QDialog::Accepted){}

    delete dialog_icp;
}
void MainWindow::icp_svd_Registration(QString maxDistance,QString transEpsilon,
                                      QString fitnessEpsilon,QString maxIterations,
                                      QString srcFilePath,QString tgtFilePath)
{
    // 确认路径非空
    if(srcFilePath.isEmpty() || tgtFilePath.isEmpty())
    {
        return;
    }

    // 读取源点云文件
    if(srcFilePath.endsWith("ply"))
    {
        qDebug()<<srcFilePath;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(srcFilePath.toStdString(), src_cloud) == -1 ) //* load the file
        {
            qDebug()<<"Couldn't read file  \n";
            return ;
        }
    }
    else if (srcFilePath.endsWith("pcd"))
    {
        qDebug()<<srcFilePath;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(srcFilePath.toStdString(), src_cloud) == -1) //* load the file
        {
            qDebug()<<"Couldn't read pcd file  \n";
            return ;
        }
    }
    else {
        QMessageBox::warning(this, "Warning","点云读取格式错误！");
    }

    // 读取目标点云文件
    if(tgtFilePath.endsWith("ply"))
    {
        qDebug()<<tgtFilePath;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(tgtFilePath.toStdString(), tgt_cloud) == -1 ) //* load the file
        {
            qDebug()<<"Couldn't read file  \n";
            return ;
        }
    }
    else if (tgtFilePath.endsWith("pcd"))
    {
        qDebug()<<tgtFilePath;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(tgtFilePath.toStdString(), tgt_cloud) == -1) //* load the file
        {
            qDebug()<<"Couldn't read pcd file  \n";
            return ;
        }
    }
    else {
        QMessageBox::warning(this, "Warning","点云读取格式错误！");
    }

    // 参数类型转换
    double dstThershold = maxDistance.toDouble();
    double tEpsilon = transEpsilon.toDouble();
    double fEpsilon = fitnessEpsilon.toDouble();
    int maxIter = maxIterations.toInt();
    rst_cloud = src_cloud.makeShared();
    // 执行ICP配准
    pcl_registration_icp(src_cloud.makeShared(), tgt_cloud.makeShared(), rst_cloud,
                        dstThershold, tEpsilon, fEpsilon, maxIter);

    // 点云显示
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> srcHandler(src_cloud.makeShared(), 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgtHandler(src_cloud.makeShared(), 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rstHandler(src_cloud.makeShared(), 0, 0, 255);

    viewer->addPointCloud<pcl::PointXYZ>(src_cloud.makeShared(), srcHandler, "0");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, "0");
    viewer->addPointCloud<pcl::PointXYZ>(tgt_cloud.makeShared(), tgtHandler, "1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, "1");
    viewer->addPointCloud<pcl::PointXYZ>(rst_cloud, rstHandler, "2");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, "2");

    viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();

}


void MainWindow::on_ICP_SVD_TreeViewAction_triggered()
{
    // 获取所有选中的索引
    QModelIndexList selectedIndexes = ui->DBtreeView->selectionModel()->selectedIndexes();
    if(2 != selectedIndexes.size())
    {
        QMessageBox::warning(this, "Warning", "请选择两个点云");
        return;
    }

    int idx1 = selectedIndexes[0].row();
    int idx2 = selectedIndexes[1].row();

    qDebug() << "selectedIndex 1 :" << idx1;
    qDebug() << "selectedIndex 2 :" << idx2;
    //当前位置包含-1值返回
    if( -1 == idx1 || -1 == idx2)
    {
        return;
    }

    QString cloudName1 = selectedIndexes[0].data().toString();
    QString cloudName2 = selectedIndexes[1].data().toString();
    qDebug() << "selectedCloudName 1 :" << cloudName1;
    qDebug() << "selectedCloudName 2 :" << cloudName2;


    dialog_icpTV = new Dialog_icpTV(idx1, idx2, cloudName1, cloudName2);

    connect(dialog_icpTV, SIGNAL(sendPara(QString,QString,QString,QString,int,int)),
            this, SLOT(icp_svd_treeView_Registration(QString,QString,QString,QString,int,int)));

    if(dialog_icpTV->exec()==QDialog::Accepted){}

    delete dialog_icpTV;

}
void MainWindow::icp_svd_treeView_Registration(QString maxDistance,
                                               QString transEpsilon,
                                               QString fitnessEpsilon,
                                               QString maxIterations,
                                               int srcIndex,
                                               int tgtIndex)
{
    // 参数类型转换&定义局部变量
    double dstThershold = maxDistance.toDouble();
    double tEpsilon = transEpsilon.toDouble();
    double fEpsilon = fitnessEpsilon.toDouble();
    int maxIter = maxIterations.toInt();
    PointCloudXYZ::Ptr cloud_src = cloud_vec[srcIndex];
    PointCloudXYZ::Ptr cloud_tgt = cloud_vec[tgtIndex];
    PointCloudXYZ::Ptr cloud_rst(new PointCloudXYZ);

    // 执行ICP配准
    pcl_registration_icp(cloud_src, cloud_tgt, cloud_rst,
                         dstThershold, tEpsilon, fEpsilon, maxIter);

    // 将结果点云入列
    cloud_vec.push_back(cloud_rst);
    cloud_index.push_back(1);
    cloud_pointSize.push_back(cloud_pointSize[srcIndex]);   // 继承源点云pointSize

    // 获取模型对象
    // QAbstractItemModel *model = ui->DBtreeView->model();
    // 使用行索引创建QModelIndex对象
    QModelIndex index = model->index(srcIndex, 0); // 0表示第一列
    // 获取显示的文本内容
    QString srcName = model->data(index, Qt::DisplayRole).toString();
    // 新建树形控件项目
    itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],
                                   "aligned " + srcName);
    itemFolder->setCheckable(true);
    itemFolder->setCheckState(Qt::Checked);//获取选中状态
    model->appendRow(itemFolder);

    // 点云显示
    std::string cloud_id = std::to_string(cloud_vec.size() - 1);
    int newidx = cloud_vec.size() - 1;
    if(newidx >= 3)
    {
        // 如果点云数量小于3则使用预设rgb颜色
        // 如果点云数量大于3使用白色
        rgbColor *colorex = new rgbColor{255, 255, 255};
        cloud_color.push_back(colorex);
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_rst,
                                                                                 cloud_color[newidx]->r,
                                                                                 cloud_color[newidx]->g,
                                                                                 cloud_color[newidx]->b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_rst, colorHandler, cloud_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);

    viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
}

// 手动配准
void MainWindow::on_alignPPPAction_triggered()
{
    QModelIndexList selectedIndexes = ui->DBtreeView->selectionModel()->selectedIndexes();
    if(2 != selectedIndexes.size())
    {
        QMessageBox::warning(this, "Warning", "请选择两个点云");
        return;
    }

    int idx1 = selectedIndexes[0].row();
    int idx2 = selectedIndexes[1].row();
        //当前位置包含-1值返回
    if( -1 == idx1 || -1 == idx2)
    {
        return;
    }

    QString cloudName1 = selectedIndexes[0].data().toString();
    QString cloudName2 = selectedIndexes[1].data().toString();
    qDebug() << "selectedCloudName 1 :" << cloudName1;
    qDebug() << "selectedCloudName 2 :" << cloudName2;

    dialog_alignppp = new Dialog_AlignPPP(idx1, idx2, cloudName1, cloudName2, this);
    dialog_alignppp->setAttribute(Qt::WA_DeleteOnClose);

    connect(dialog_alignppp, SIGNAL(sendIdx(int,int)),
            this, SLOT(pointPairsPicking_Align(int,int)));

    if(dialog_alignppp->exec()==QDialog::Accepted){}
}



void MainWindow::pointPairsPicking_Align(int srcIdx, int tgtIdx)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->createViewPort (0.0, 0.5, 0.5, 1.0, srcVp);
    viewer->createViewPort (0.5, 0.5, 1.0, 1.0, tgtVp);
    viewer->createViewPort (0.0, 0.0, 1.0, 0.5, rstVp);
    viewer->setBackgroundColor(25.0/255, 25.0/255 , 25.0/255);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (cloud_vec[srcIdx],
                                                                          cloud_color[srcIdx]->r,
                                                                          cloud_color[srcIdx]->g,
                                                                          cloud_color[srcIdx]->b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_vec[srcIdx], src_h, "srcCloud", srcVp);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             cloud_pointSize[srcIdx],
                                             "srcCloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (cloud_vec[tgtIdx],
                                                                          cloud_color[tgtIdx]->r,
                                                                          cloud_color[tgtIdx]->g,
                                                                          cloud_color[tgtIdx]->b);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_vec[tgtIdx], tgt_h, "tgtCloud", tgtVp);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             cloud_pointSize[tgtIdx],
                                             "tgtCloud");

    srcPP.clicked_points_3d = PointCloudXYZ::Ptr(new PointCloudXYZ);
    tgtPP.clicked_points_3d = PointCloudXYZ::Ptr(new PointCloudXYZ);
    srcPP.picked_idx.clear();
    tgtPP.picked_idx.clear();
    viewer->registerPointPickingCallback(point_callback, this);

    viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();

    // 禁用菜单栏和树形控件
    ui->menubar->setDisabled(true);
    ui->DBtreeView->setDisabled(true);
    // 启动选点窗口
    form_ppp = new Form_PPP(srcIdx, tgtIdx, this);
    form_ppp->setAttribute(Qt::WA_DeleteOnClose);
    // form_ppp->setWindowFlags(Qt::WindowStaysOnTopHint);
    // form_ppp->setWindowFlags(Qt::Widget);
    form_ppp->show();
    ui->tabWidget->addTab(form_ppp, "Align(Point Pairs Picking)");
    ui->tabWidget->setCurrentWidget(form_ppp);
    // ui->verticalLayout_2->addWidget(form_ppp);
    //--------------------------connection--------------------------
    connect(form_ppp, SIGNAL(sendIdx(int, int)),
            this, SLOT(pointPairsPicking_Preview(int, int)));

    // connect(this, SIGNAL(sendPoint(pcl::PointXYZ, int)),
    //         form_ppp, SLOT(addItemtoTableview(pcl::PointXYZ, int)));

    connect(this, SIGNAL(updateTableview(pcl::PointCloud<pcl::PointXYZ>::Ptr, int)),
            form_ppp, SLOT(updateItemtoTableview(pcl::PointCloud<pcl::PointXYZ>::Ptr, int)));

    connect(this, SIGNAL(sendPPPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)),
            form_ppp, SLOT(receiveResultCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

    connect(form_ppp, SIGNAL(confirm(pcl::PointCloud<pcl::PointXYZ>::Ptr)),
            this, SLOT(pointPairsPicking_Confirm(pcl::PointCloud<pcl::PointXYZ>::Ptr)));

    // 连接源点云点删除信号和删除函数
    connect(form_ppp, SIGNAL(deleteSrcPoint()),
            this, SLOT(pointPairsPicking_deleteSrcPoint()));

    // 连接目标点云点删除信号和删除函数
    connect(form_ppp, SIGNAL(deleteTgtPoint()),
            this, SLOT(pointPairsPicking_deleteTgtPoint()));

    // 连接form_ppp窗口取消按钮信号与关闭函数
    connect(form_ppp, SIGNAL(rejected()),
            this, SLOT(pointPairsPicking_rejected()));

}

void MainWindow::point_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
    MainWindow *p = (MainWindow *)args;
    if (event.getPointIndex() == -1)
        return;

    pcl::PointXYZ current_point;
    std::string srcPoints = "clicked_points_src";
    std::string tgtPoints = "clicked_points_tgt";
    event.getPoint(current_point.x, current_point.y, current_point.z);
    if("srcCloud" == event.getCloudName())
    {
        // 保存选点和点的索引
        // 此处保存的点的索引是相对于完整的源点云而言的，如srcPP.picked_idx相对于srcCloud，
        // 而不是srcPP.clicked_points_3d。
        // srcPP.clicked_points_3d中保存的点的索引从0开始计数。
        p->srcPP.clicked_points_3d->points.push_back(current_point);
        p->srcPP.picked_idx.push_back(event.getPointIndex());

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(p->srcPP.clicked_points_3d, 0, 0, 255);
        p->viewer->removePointCloud(srcPoints);
        p->viewer->addPointCloud(p->srcPP.clicked_points_3d, blue, srcPoints, p->srcVp);
        p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, srcPoints);

        // emit p->sendPoint(current_point, 0);
        emit p->updateTableview(p->srcPP.clicked_points_3d,
                                0);
    }
    else if("tgtCloud" == event.getCloudName())
    {
        p->tgtPP.clicked_points_3d->points.push_back(current_point);
        p->tgtPP.picked_idx.push_back(event.getPointIndex());

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(p->tgtPP.clicked_points_3d,0, 0, 255);
        p->viewer->removePointCloud(tgtPoints);
        p->viewer->addPointCloud(p->tgtPP.clicked_points_3d, blue, tgtPoints, p->tgtVp);
        p->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, tgtPoints);

        emit p->updateTableview(p->tgtPP.clicked_points_3d,
                                1);
    }
    else
    {
        QMessageBox::warning(p, "Warning!", "点云选择错误！");
    }
}

void MainWindow::pointPairsPicking_Preview(int srcIdx, int tgtIdx)
{
    if (srcPP.picked_idx.size() != tgtPP.picked_idx.size()) {
        QMessageBox::warning(this, "Waring!", "点的数目不一致");
        return;
    }

    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    bool isAdjustScaleTrue =form_ppp->is_adjustScale_checked();
    // 进行手动配准
    manualAlign(cloud_vec[srcIdx],
                cloud_vec[tgtIdx],
                srcPP.picked_idx,
                tgtPP.picked_idx,
                transformation_matrix,
                isAdjustScaleTrue);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_vec[srcIdx], *transformed_cloud, transformation_matrix);

    // 发送结果点云指针
    emit sendPPPCloud(transformed_cloud);

    qDebug() << transformation_matrix;
    std::string rstCloud = "rstCloud";
    std::string tgtCloud = "tgtCloud_rst";
    viewer->removePointCloud(rstCloud);
    viewer->removePointCloud(tgtCloud);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(transformed_cloud,255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_vec[tgtIdx],0, 255, 0);

    viewer->addPointCloud(transformed_cloud, red, rstCloud, rstVp);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             DEFAUT_PT_SIZE,
                                             rstCloud);
    viewer->addPointCloud(cloud_vec[tgtIdx], green, tgtCloud, rstVp);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                             cloud_pointSize[tgtIdx],
                                             tgtCloud);

    if(form_ppp->is_cameraAutoReset_checked())
    {
        viewer->resetCamera();
    }

    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
}

void MainWindow::pointPairsPicking_Confirm(pcl::PointCloud<pcl::PointXYZ>::Ptr rstCloud)
{
    cloud_vec.push_back(rstCloud);
    cloud_index.push_back(1);
    cloud_pointSize.push_back(DEFAUT_PT_SIZE);

    // 重置结果点云指针
    // rst_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 重置viewer
    initMainWidgetViewer();

    // itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],QStringLiteral("cloud%1").arg(cloud_vec.size()-1));
    itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],
                                   "Aligned(PPP) Cloud");
    itemFolder->setCheckable(true);
    itemFolder->setCheckState(Qt::Checked);//获取选中状态
    model->appendRow(itemFolder);

    std::string cloud_id = std::to_string(cloud_vec.size() - 1);
    int temp_id = cloud_vec.size() - 1;
    if(temp_id < 3)
    {
        // 如果点云数量小于3则使用预设rgb颜色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_vec[temp_id],
                                                                                     cloud_color[temp_id]->r,
                                                                                     cloud_color[temp_id]->g,
                                                                                     cloud_color[temp_id]->b);
        // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud_vec[temp_id]);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_vec[temp_id], colorHandler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);
    }
    else
    {
        // 如果点云数量大于3使用白色
        rgbColor *colorex = new rgbColor{255, 255, 255};
        cloud_color.push_back(colorex);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud_vec[temp_id],
                                                                                     cloud_color[temp_id]->r,
                                                                                     cloud_color[temp_id]->g,
                                                                                     cloud_color[temp_id]->b);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_vec[temp_id], colorHandler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);
    }
    viewer->resetCamera();
    // 刷新窗口
    viewer_update(cloud_vec, cloud_index);
    viewer->getRenderWindow()->Render();    // 强制立即刷新viewer
    ui->mainWidget->update();               // 确保 Qt 窗口组件也同步刷新

    // 启用菜单栏和树形控件
    ui->menubar->setDisabled(false);
    ui->DBtreeView->setDisabled(false);
}

void MainWindow::pointPairsPicking_deleteSrcPoint()
{
    // 检验剩余元素数量是否不小于一
    if(srcPP.picked_idx.size() < 1 ||
        srcPP.clicked_points_3d->size() < 1)
    {
        QMessageBox::warning(this, "Warning!", "没有选中点可删除");
        return;
    }
    // 移除最后元素
    srcPP.picked_idx.pop_back();
    srcPP.clicked_points_3d->points.pop_back();
    // 更新viewer
    std::string srcPoints = "clicked_points_src";
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(srcPP.clicked_points_3d, 0, 0, 255);
    viewer->removePointCloud(srcPoints);
    viewer->addPointCloud(srcPP.clicked_points_3d, blue, srcPoints, srcVp);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, srcPoints);
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
    // 更新tableview
    emit updateTableview(srcPP.clicked_points_3d,
                         0);
}

void MainWindow::pointPairsPicking_deleteTgtPoint()
{
    // 检验剩余元素数量是否不小于一
    if(tgtPP.picked_idx.size() < 1 ||
        tgtPP.clicked_points_3d->size() < 1)
    {
        QMessageBox::warning(this, "Warning!", "没有选中点可删除");
        return;
    }
    // 移除最后元素
    tgtPP.picked_idx.pop_back();
    tgtPP.clicked_points_3d->points.pop_back();
    // 更新viewer
    std::string tgtPoints = "clicked_points_tgt";
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue(tgtPP.clicked_points_3d, 0, 0, 255);
    viewer->removePointCloud(tgtPoints);
    viewer->addPointCloud(tgtPP.clicked_points_3d, blue, tgtPoints, tgtVp);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, tgtPoints);
    // 刷新窗口
    viewer->getRenderWindow()->Render();
    ui->mainWidget->update();
    // 更新tableview
    emit updateTableview(tgtPP.clicked_points_3d,
                         1);
}

void MainWindow::pointPairsPicking_rejected()
{
    // 重置viewer
    initMainWidgetViewer();

    viewer->resetCamera();
    // 刷新窗口
    viewer_update(cloud_vec, cloud_index);
    viewer->getRenderWindow()->Render();    // 强制立即刷新viewer
    ui->mainWidget->update();               // 确保 Qt 窗口组件也同步刷新

    // 启用菜单栏和树形控件
    ui->menubar->setDisabled(false);
    ui->DBtreeView->setDisabled(false);

}

//***********************************RegistrationManu*******************************************//
