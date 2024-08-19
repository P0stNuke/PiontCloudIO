#ifndef VIEW_POINTSIZESETTING_H
#define VIEW_POINTSIZESETTING_H

#include <QDialog>

namespace Ui {
class View_pointSizeSetting;
}

class View_pointSizeSetting : public QDialog
{
    Q_OBJECT

public:
    explicit View_pointSizeSetting(QWidget *parent = nullptr);
    ~View_pointSizeSetting();

signals:
    void sendData(int ptSize);

private slots:
    void on_buttonBox_accepted();

    void on_buttonBox_rejected();

private:
    Ui::View_pointSizeSetting *ui;
};

#endif // VIEW_POINTSIZESETTING_H
