#ifndef FORM_PPP_H
#define FORM_PPP_H

#include <QWidget>
#include <QStandardItemModel>
#include "pcl_fuction.h"

namespace Ui {
class Form_PPP;
}

class Form_PPP : public QWidget
{
    Q_OBJECT

signals:
    void sendIdx(int src, int tgt);

    void confirm(pcl::PointCloud<pcl::PointXYZ>::Ptr rst_cloud);

    void rejected();

    void deleteSrcPoint();

    void deleteTgtPoint();

public:
    explicit Form_PPP(int srcIndex,
                      int tgtIndex,
                      QWidget *parent = nullptr);
    ~Form_PPP();
    bool is_adjustScale_checked();
    bool is_cameraAutoReset_checked();

private slots:

    void on_previewPushButton_clicked();

    void updateItemtoTableview(pcl::PointCloud<pcl::PointXYZ>::Ptr picked_points,
                               int flag);

    void on_buttonBox_accepted();

    void receiveResultCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr rst_cloud);

    void on_srcDeleteButton_clicked();

    void on_tgtDeleteButton_clicked();

    void on_buttonBox_rejected();

private:
    Ui::Form_PPP *ui;
    QStandardItemModel *srcModel;
    QStandardItemModel *tgtModel;
    int srcIdx, tgtIdx;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rstCloud;
};

#endif // FORM_PPP_H
