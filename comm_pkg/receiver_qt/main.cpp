#include "mainwidget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWidget w;
    w.setFixedSize(800, 480);


    //w.showFullScreen();
    w.move(0, 0);


    w.show();
    return a.exec();
}
