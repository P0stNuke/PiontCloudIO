#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStandardItemModel>

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


//*************************************initialization*************************************//
void MainWindow::initMainWidgetViewer()
{
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer,renderWindow,"viewer",false));
    ui->mainWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->mainWidget->interactor(), ui->mainWidget->renderWindow());
    viewer->setBackgroundColor(0.1, 0.1, 0.1);
    ui->mainWidget->update();
}

void MainWindow::initDBtree()
{
    model = new QStandardItemModel(this);
    //ui->DBtreeView->setHeaderHidden(true);  //设置表头隐藏
    model->setHorizontalHeaderLabels(QStringList() << "--Cloud--DB--Tree--");  // 设置水平表头标题
    ui->DBtreeView->setModel(model);    //设置model
    ui->DBtreeView->expandAll();    //设置展开
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
    QString fileName = QFileDialog::getOpenFileName(this, tr("open  file"),
                                                    "E://Qt_project//00pointCloud",
                                                    tr("pcb files(*.pcd *.ply *.txt) ;;All files (*.*)"));

    if(fileName.isEmpty())
    {
        return;
    }

    if(fileName.endsWith("ply"))
    {
        qDebug()<<fileName;
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toStdString(), cloud) == -1) //* load the file
        {
            qDebug()<<"Couldn't read file  \n";
            return ;
        }
    }
    else if (fileName.endsWith("pcd"))
    {
        qDebug()<<fileName;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), cloud) == -1) //* load the file
        {
            qDebug()<<"Couldn't read pcd file  \n";
            return ;
        }
    }
    else if (fileName.endsWith("txt"))
    {
        qDebug()<<fileName;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>);
        CreateCloudFromTxt(fileName.toStdString(),cloudTemp);
        cloud=*cloudTemp;
    }
    else {
        QMessageBox::warning(this, "Warning","点云读取格式错误！");
    }


    cloud_vec.push_back(cloud.makeShared());
    cloud_index.push_back(1);
    cloud_pointSize.push_back(DEFAUT_PT_SIZE);

    itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],QStringLiteral("cloud%1").arg(cloud_vec.size()-1));
    itemFolder->setCheckable(true);
    itemFolder->setCheckState(Qt::Checked);//获取选中状态
    model->appendRow(itemFolder);


    // 移除窗口点云
    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    // viewer->addPointCloud(cloud.makeShared() ,cloud_name[0]);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_name[0]);
    std::string cloud_id = std::to_string(cloud_vec.size() - 1);
    int temp_id = cloud_vec.size() - 1;
    if(temp_id <= 2)
    {
        // 如果点云数量小于3则使用预设rgb颜色
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud.makeShared(),
                                                                                 cloud_color[temp_id]->r,
                                                                                 cloud_color[temp_id]->g,
                                                                                 cloud_color[temp_id]->b);
        // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> RandomColor(cloud.makeShared());
        viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), colorHandler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);
    }
    else
    {
        // 如果点云数量大于3使用白色
        rgbColor *colorex = new rgbColor{255, 255, 255};
        cloud_color.push_back(colorex);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> colorHandler(cloud.makeShared(),
                                                                                     cloud_color[temp_id]->r,
                                                                                     cloud_color[temp_id]->g,
                                                                                     cloud_color[temp_id]->b);
        viewer->addPointCloud<pcl::PointXYZ>(cloud.makeShared(), colorHandler, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, DEFAUT_PT_SIZE, cloud_id);
    }
    viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();    // 强制立即刷新viewer
    ui->mainWidget->update();               // 确保 Qt 窗口组件也同步刷新
}


void MainWindow::on_saveAction_triggered()
{
    if(cloud.empty())
    {
        QMessageBox::information(this, "无可供保存的点云", "请先读取一个点云文件");
        return;
    }
    else
    {
        QString filename = QFileDialog::getSaveFileName(this, tr ("Open point cloud"), "", tr ("*.pcd;;*.ply"));

        if (filename.isEmpty ())
            return;
        int return_status;
        if (filename.endsWith (".pcd", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud.makeShared());
        else if (filename.endsWith (".ply", Qt::CaseInsensitive))
            return_status = pcl::io::savePLYFileBinary (filename.toStdString (), *cloud.makeShared());
        else
        {
            filename.append(".pcd");
            return_status = pcl::io::savePCDFileBinary (filename.toStdString (), *cloud.makeShared());
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
    temp.setRgb(143,153,159,255);//防止与背景串色
    if(!cloud.empty() && (color!=temp) )
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
    else
    {
        cloud_color[idx]->r = 255;
        cloud_color[idx]->g = 255;
        cloud_color[idx]->b = 255;

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ >slected_color(cloud_vec[idx],255,255,255);
        viewer->updatePointCloud(cloud_vec[idx],slected_color, cloud_id);
    }
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

    QStandardItemModel* model = static_cast<QStandardItemModel*>(ui->DBtreeView->model());
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

    std::string cloud_id = std::to_string(pickedIndex);
    int idx = pickedIndex;

    float size=voxelSize.toFloat();
    auto cloud_out = pcl_filter_voxel(cloud_vec[idx],size);
    cloud_vec[idx] = cloud_out;
    // cloud_vec[idx] = pcl_filter_voxel(cloud_vec[idx],size);


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
//***********************************RegistrationManu*******************************************//
