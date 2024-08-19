#ifndef VIEW_RENDERING_H
#define VIEW_RENDERING_H

#include <QDialog>

namespace Ui {
class View_rendering;
}

class View_rendering : public QDialog
{
    Q_OBJECT

public:
    explicit View_rendering(QWidget *parent = nullptr);
    ~View_rendering();

signals:
    void sendData(QString data);

private slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_XradioButton_clicked();
    void on_YradioButton_clicked();
    void on_ZradioButton_clicked();

private:
    Ui::View_rendering *ui;
    QString axis;
};

#endif // VIEW_RENDERING_H
