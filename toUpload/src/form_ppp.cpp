#include "form_ppp.h"
#include "ui_form_ppp.h"
#include <QMessageBox>

Form_PPP::Form_PPP(int srcIndex, int tgtIndex, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Form_PPP)
{
    ui->setupUi(this);
    // 接收参数
    srcIdx = srcIndex;
    tgtIdx = tgtIndex;
    rstCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ui->srcTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    ui->tgtTableView->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // 创建一个 QStandardItemModel 并设置列数为 4（序号, x, y, z）
    srcModel = new QStandardItemModel(0, 4);
    srcModel->setHorizontalHeaderLabels({"Index", "X", "Y", "Z"});
    ui->srcTableView->setModel(srcModel);
    ui->srcTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    tgtModel = new QStandardItemModel(0, 4);
    tgtModel->setHorizontalHeaderLabels({"Index", "X", "Y", "Z"});
    ui->tgtTableView->setModel(tgtModel);
    ui->tgtTableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
}

Form_PPP::~Form_PPP()
{
    delete ui;
}

void Form_PPP::on_previewPushButton_clicked()
{
    emit sendIdx(srcIdx, tgtIdx);
    qDebug() << "preview";
}

void Form_PPP::updateItemtoTableview(pcl::PointCloud<pcl::PointXYZ>::Ptr picked_points,
                                     int flag)
{
    if(flag == 0)
    {
        delete srcModel;
        srcModel = new QStandardItemModel(picked_points->size(), 4);
        srcModel->setHorizontalHeaderLabels({"Index", "X", "Y", "Z"});
        for (int row = 0; row < picked_points->size(); ++row )
        {
            const pcl::PointXYZ& point = picked_points->points[row];

            // 设置序号
            QString pointLabel = QString("A%1").arg(row);
            QStandardItem *labelItem = new QStandardItem(pointLabel);
            srcModel->setItem(row, 0, labelItem);

            // 设置 X 坐标
            QStandardItem *xItem = new QStandardItem(QString::number(point.x));
            srcModel->setItem(row, 1, xItem);

            // 设置 Y 坐标
            QStandardItem *yItem = new QStandardItem(QString::number(point.y));
            srcModel->setItem(row, 2, yItem);

            // 设置 Z 坐标
            QStandardItem *zItem = new QStandardItem(QString::number(point.z));
            srcModel->setItem(row, 3, zItem);
        }
        ui->srcTableView->setModel(srcModel);
        // ui->srcTableView->resizeColumnsToContents();
        ui->srcTableView->resizeRowsToContents();
    }
    else if(flag == 1)
    {
        delete tgtModel;
        tgtModel = new QStandardItemModel(picked_points->size(), 4);
        tgtModel->setHorizontalHeaderLabels({"Index", "X", "Y", "Z"});
        for (int row = 0; row < picked_points->size(); ++row)
        {
                const pcl::PointXYZ point = picked_points->points[row];

                // 设置序号
                QString pointLabel = QString("B%1").arg(row);
                QStandardItem *labelItem = new QStandardItem(pointLabel);
                tgtModel->setItem(row, 0, labelItem);

                // 设置 X 坐标
                QStandardItem *xItem = new QStandardItem(QString::number(point.x));
                tgtModel->setItem(row, 1, xItem);

                // 设置 Y 坐标
                QStandardItem *yItem = new QStandardItem(QString::number(point.y));
                tgtModel->setItem(row, 2, yItem);

                // 设置 Z 坐标
                QStandardItem *zItem = new QStandardItem(QString::number(point.z));
                tgtModel->setItem(row, 3, zItem);
        }
        ui->tgtTableView->setModel(tgtModel);
        // ui->srcTableView->resizeColumnsToContents();
        ui->tgtTableView->resizeRowsToContents();
    }
}


void Form_PPP::receiveResultCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr rst_cloud)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr rst (new pcl::PointCloud<pcl::PointXYZ>);
    // rst = rst_cloud;
    rstCloud = rst_cloud;
}

void Form_PPP::on_buttonBox_accepted()
{
    if(rstCloud->empty())
    {
        QMessageBox::warning(this, "Warning!", "请先进行预览");
        return;
    }
    emit confirm(rstCloud);

    this->close();
}

void Form_PPP::on_buttonBox_rejected()
{
    emit rejected();

    this->close();
}

void Form_PPP::on_srcDeleteButton_clicked()
{
    emit deleteSrcPoint();
}


void Form_PPP::on_tgtDeleteButton_clicked()
{
    emit deleteTgtPoint();
}

bool Form_PPP::is_adjustScale_checked()
{
    if(ui->adjustScaleCheckBox->isChecked())
    {
        return true;
    }
    return false;
}

bool Form_PPP::is_cameraAutoReset_checked()
{
    if(ui->cameraAutoResetCheckBox->isChecked())
    {
        return true;
    }
    return false;
}

