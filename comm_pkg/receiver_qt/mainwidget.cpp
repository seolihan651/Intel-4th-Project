#include "mainwidget.h"
#include "ui_mainwidget.h"

#include <QVBoxLayout>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);


    pTab7CamViewer = new Tab7CamViewer(this);

    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0,0,0,0);
    lay->setSpacing(0);
    lay->addWidget(pTab7CamViewer);

    setLayout(lay);

}

MainWidget::~MainWidget()
{
    delete ui;
}
