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

    itemFolder = new QStandardItem(m_publicIconMap[QStringLiteral("treeItem_cloud")],QStringLiteral("cloud%1").arg(cloud_vec.size()-1));
    itemFolder->setCheckable(true);
    itemFolder->setCheckState(Qt::Checked);//获取选中状态
    model->appendRow(itemFolder);


    // 移除窗口点云
    // viewer->removeAllPointClouds();
    // viewer->removeAllShapes();
    // viewer->addPointCloud(cloud.makeShared() ,cloud_name[0]);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_name[0]);
    std::string cloud_id = std::to_string(cloud_vec.size() - 1);
    viewer->addPointCloud(cloud.makeShared(), cloud_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_id);
    viewer->resetCamera();
    // 刷新窗口
    viewer->getRenderWindow()->Render();    // 强制立即刷新viewer
    ui->mainWidget->update();               // 确保 Qt 窗口组件也同步刷新
}


void MainWindow::on_saveAction_triggered()
{
    if(cloud.empty())
    {
        QMessageBox::information(this, "消息", "无可供保存的点云");
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
void MainWindow::on_backguandColorAction_triggered()
{
    colorPicker = new colorSelection();
    QColor color = colorPicker->getColor();
    viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    return;
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
            // pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>render(vector_cloud[i], "intensity");
            // viewer->addPointCloud<pcl::PointXYZ>(vector_cloud[i],render,std::to_string(i));
            viewer->addPointCloud<pcl::PointXYZ>(vector_cloud[i], std::to_string(i));
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, std::to_string(i));

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
//***********************************viewerManu*******************************************//
