#include "mainwidget.h"
#include "ui_mainwidget.h"

#include <QVBoxLayout>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainWidget)
{
    ui->setupUi(this);

    // Tab7CamViewer를 메인 위젯에 바로 붙이기
    pTab7CamViewer = new Tab7CamViewer(this);

    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0,0,0,0);
    lay->setSpacing(0);
    lay->addWidget(pTab7CamViewer);

    setLayout(lay);

    // ✅ Tab2SocketClient가 없으니 connect 제거
    // connect(pTab2SocketClient, SIGNAL(tab7RecvDataSig(QString)), pTab7CamViewer, SLOT(tab7RecvDataSlot(QString)));
}

MainWidget::~MainWidget()
{
    delete ui;
}
