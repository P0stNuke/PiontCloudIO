#ifndef DIALOG_ICPTV_H
#define DIALOG_ICPTV_H

#include <QDialog>
#include <QStringListModel> // 字符串列表模型
#include <QMessageBox>


namespace Ui {
class Dialog_icpTV;
}

class Dialog_icpTV : public QDialog
{
    Q_OBJECT
signals:
    void sendPara(QString maxDistance,
                  QString transEpsilon,
                  QString fitnessEpsilon,
                  QString maxIterations,
                  int src_idx,
                  int tgt_idx);

public:
    explicit Dialog_icpTV(int index1,
                          int index2,
                          QString cloudName1,
                          QString cloudName2,
                          QWidget *parent = nullptr);
    ~Dialog_icpTV();

    int idx1;
    int idx2;
    QString cloud1;
    QString cloud2;

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::Dialog_icpTV *ui;
};

#endif // DIALOG_ICPTV_H
