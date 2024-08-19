#ifndef FILTER_VOXEL_H
#define FILTER_VOXEL_H

#include <QDialog>
#include <QValidator>

namespace Ui {
class Filter_Voxel;
}

class Filter_Voxel : public QDialog
{
    Q_OBJECT

public:
    explicit Filter_Voxel(QWidget *parent = nullptr);
    ~Filter_Voxel();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

signals:
    void sendSizeofVoxel(QString voxelSize);

private:
    Ui::Filter_Voxel *ui;
};

#endif // FILTER_VOXEL_H
