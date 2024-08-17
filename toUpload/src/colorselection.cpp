#include "colorselection.h"
#include <QHBoxLayout>
#include <qcolordialog.h>

colorSelection::colorSelection(QWidget *parent)
    : QWidget{parent}
{
    QColor c=QColorDialog::getColor(Qt::white, this);

    if(c.isValid())
    {
        color=c;
    }
    else
    {
        c.setRgb(143,153,159,255);
        color=c;
    }

}

colorSelection::~colorSelection()
{}


// 设置颜色
void colorSelection::setColor(const QColor &c)
{
    if(c.isValid())
    {
        color = c;
    }
}

// 获取颜色
QColor colorSelection::getColor()
{
    return color;
}
