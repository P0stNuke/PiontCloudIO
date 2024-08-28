#include "dialog_icp.h"
#include "ui_dialog_icp.h"
#include "qfiledialog.h"


Dialog_icp::Dialog_icp(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Dialog_icp)
{
    ui->setupUi(this);

}

Dialog_icp::~Dialog_icp()
{
    delete ui;
}

void Dialog_icp::on_buttonBox_accepted()
{
    emit sendData(ui->lineEdit->text(),ui->lineEdit_2->text(),ui->lineEdit_3->text(),
                  ui->lineEdit_4->text(),ui->textBrowser->toPlainText(),ui->textBrowser_2->toPlainText());

    this->close();
}

void Dialog_icp::on_buttonBox_rejected()
{
    this->close();
}

void Dialog_icp::on_model_pathPushButton_clicked()
{
    QString PathName_1 = QFileDialog::getOpenFileName(
        this, tr("open  file"),
        "E://Qt_project//00pointCloud",
        tr("pcb files(*.pcd *.ply) ;;All files (*.*)"));
    ui->textBrowser->setText(PathName_1);
}


void Dialog_icp::on_scene_pathPushButton_clicked()
{
    QString PathName_2 = QFileDialog::getOpenFileName(
        this, tr("open  file"),
        "E://Qt_project//00pointCloud",
        tr("pcb files(*.pcd *.ply) ;;All files (*.*)"));
    ui->textBrowser_2->setText(PathName_2);
}



