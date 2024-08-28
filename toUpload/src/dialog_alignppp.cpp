#include "dialog_alignppp.h"
#include "ui_dialog_alignppp.h"

Dialog_AlignPPP::Dialog_AlignPPP(int index1,
                                 int index2,
                                 QString cloudName1,
                                 QString cloudName2,
                                 QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog_AlignPPP)
{
    ui->setupUi(this);

    // 接收参数
    idx1 = index1;
    idx2 = index2;
    cloud1 = cloudName1;
    cloud2 = cloudName2;

    // 禁用QListView编辑
    ui->listView->setEditTriggers(QAbstractItemView::NoEditTriggers);
    // 创建点云数据显示至清单视图
    QStringList list;
    list.append(cloud1);
    list.append(cloud2);
    // 用数据列表创建数据显示模型进行实现
    QStringListModel *listmode = new QStringListModel(list);
    ui->listView->setModel(listmode);
}

Dialog_AlignPPP::~Dialog_AlignPPP()
{
    delete ui;
}

void Dialog_AlignPPP::on_buttonBox_accepted()
{
    QModelIndex curIndex = ui->listView->currentIndex();
    if( -1 == curIndex.row())
    {
        QMessageBox::warning(this, "Warning", "请选择一组点云作为目标点云");
        return;
    }
    if(cloud1 == curIndex.data().toString())
    {
        int srcidx = idx2;
        int tgtidx = idx1;
        emit sendIdx(srcidx, tgtidx);
    }
    else if(cloud2 == curIndex.data().toString())
    {
        int srcidx = idx1;
        int tgtidx = idx2;
        emit sendIdx(srcidx, tgtidx);

    }
    else
    {
        QMessageBox::warning(this, "Warning", "点云选择错误");
    }

    this->close();

}


void Dialog_AlignPPP::on_buttonBox_rejected()
{
    this->close();
}

