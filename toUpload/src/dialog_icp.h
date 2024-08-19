#ifndef DIALOG_ICP_H
#define DIALOG_ICP_H

#include <QDialog>

namespace Ui {
class Dialog_icp;
}

class Dialog_icp : public QDialog
{
    Q_OBJECT

signals:
    void sendData(QString data1,QString data2,QString data3,
                  QString data4,QString data5,QString data6);

public:
    explicit Dialog_icp(QWidget *parent = nullptr);
    ~Dialog_icp();

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_model_pathPushButton_clicked();
    void on_scene_pathPushButton_clicked();

private:
    Ui::Dialog_icp *ui;
};

#endif // DIALOG_ICP_H
