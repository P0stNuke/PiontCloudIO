#include "view_pointsizesetting.h"
#include "ui_view_pointsizesetting.h"

View_pointSizeSetting::View_pointSizeSetting(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::View_pointSizeSetting)
{
    ui->setupUi(this);

    ui->spinBox->setMinimum(1);
    ui->spinBox->setMaximum(10);
    ui->spinBox->setSingleStep(1);
    ui->spinBox->setValue(1);

    ui->horizontalSlider->setMinimum(1);
    ui->horizontalSlider->setMaximum(10);
    ui->horizontalSlider->setSingleStep(1);
    ui->horizontalSlider->setValue(1);

    connect(ui->spinBox, SIGNAL(valueChanged(int)),
            ui->horizontalSlider, SLOT(setValue(int)));
    connect(ui->horizontalSlider, SIGNAL(valueChanged(int)),
            ui->spinBox, SLOT(setValue(int)));

}

View_pointSizeSetting::~View_pointSizeSetting()
{
    delete ui;
}

void View_pointSizeSetting::on_buttonBox_accepted()
{
    int value = ui->spinBox->value();
    emit sendData(value);
    this->close();
}


void View_pointSizeSetting::on_buttonBox_rejected()
{
    this->close();
}

