#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include "tab7camviewer.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWidget;
}
QT_END_NAMESPACE

class MainWidget : public QWidget
{
    Q_OBJECT

public:
    MainWidget(QWidget *parent = nullptr);
    ~MainWidget();

private:
    Ui::MainWidget *ui;
    Tab7CamViewer *pTab7CamViewer;
};
#endif // MAINWIDGET_H
