#ifndef COLORSELECTION_H
#define COLORSELECTION_H

#include <QWidget>

class colorSelection : public QWidget
{
    Q_OBJECT
public:
    explicit colorSelection(QWidget *parent = nullptr);
    ~colorSelection();
    void    setColor(const QColor &c);
    QColor  getColor();

private:
    QColor color;
};

#endif // COLORSELECTION_H
