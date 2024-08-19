#include "view_rendering.h"
#include "ui_view_rendering.h"

View_rendering::View_rendering(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::View_rendering)
{
    ui->setupUi(this);
}

View_rendering::~View_rendering()
{
    delete ui;
}

void View_rendering::on_buttonBox_accepted()
{
    emit sendData(axis);
    this->close();
}

void View_rendering::on_buttonBox_rejected()
{
    this->close();
}

void View_rendering::on_XradioButton_clicked()
{
    if(ui->XradioButton->isChecked())
    {
        ui->YradioButton->setChecked(false);
        ui->ZradioButton->setChecked(false);
        axis = "x";
    }
}


void View_rendering::on_YradioButton_clicked()
{
    if(ui->YradioButton->isChecked())
    {
        ui->XradioButton->setChecked(false);
        ui->ZradioButton->setChecked(false);
        axis = "y";
    }
}

void View_rendering::on_ZradioButton_clicked()
{
    if(ui->ZradioButton->isChecked())
    {
        ui->XradioButton->setChecked(false);
        ui->YradioButton->setChecked(false);
        axis = "z";
    }
}
