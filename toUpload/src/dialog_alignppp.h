#ifndef DIALOG_ALIGNPPP_H
#define DIALOG_ALIGNPPP_H

#include <QDialog>
#include <QStringListModel> // 字符串列表模型
#include <QMessageBox>


namespace Ui {
class Dialog_AlignPPP;
}

class Dialog_AlignPPP : public QDialog
{
    Q_OBJECT

signals:
    void sendIdx(int srcIdx, int tgtIdx);

public:
    explicit Dialog_AlignPPP(int index1,
                             int index2,
                             QString cloudName1,
                             QString cloudName2,
                             QWidget *parent = nullptr);
    ~Dialog_AlignPPP();

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::Dialog_AlignPPP *ui;
    int idx1;
    int idx2;
    QString cloud1;
    QString cloud2;
};

#endif // DIALOG_ALIGNPPP_H
