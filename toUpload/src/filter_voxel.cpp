#include "filter_voxel.h"
#include "ui_filter_voxel.h"

Filter_Voxel::Filter_Voxel(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::Filter_Voxel)
{
    ui->setupUi(this);
    ui->lineEdit->setValidator( new QDoubleValidator(0, 100, 5) );
}

Filter_Voxel::~Filter_Voxel()
{
    delete ui;
}

void Filter_Voxel::on_buttonBox_accepted()
{
    emit sendSizeofVoxel(ui->lineEdit->text());

    this->close();
}


void Filter_Voxel::on_buttonBox_rejected()
{
    this->close();
}

