#include "tab7camviewer.h"
#include "ui_tab7camviewer.h"

Tab7CamViewer::Tab7CamViewer(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab7CamViewer)
{
    ui->setupUi(this);
    setWindowTitle("CamViewer");

    ui->pPBsnapShot->setEnabled(false);
    ui->pCBrgb->setEnabled(false);

    pMeshRxThread = new MeshRxThread(this);
    connect(pMeshRxThread, &MeshRxThread::frameReady, this,
            [this](const QImage &img){
                if (img.isNull()) return;
                ui->plabelCamView->setPixmap(
                    QPixmap::fromImage(img).scaled(
                        ui->plabelCamView->size(),
                        Qt::KeepAspectRatio,
                        Qt::FastTransformation
                        )
                    );
            },
            Qt::QueuedConnection);

    connect(pMeshRxThread, &MeshRxThread::resetToDefault, this,
            [this](){
                ui->plabelCamView->setPixmap(QPixmap(":/Images/Images/display.jpg"));
            },
            Qt::QueuedConnection);


    //pMeshRxThread->pCamView = ui->plabelCamView;   // ✅ QLabel 그대로 사용
}

Tab7CamViewer::~Tab7CamViewer()
{
    // 안전 종료
    pMeshRxThread->camViewFlag = false;
    if (pMeshRxThread->isRunning()) {
        pMeshRxThread->wait();
    }
    delete ui;
}

void Tab7CamViewer::on_pPBcamStart_clicked(bool checked)
{
    if (checked) {
        pMeshRxThread->camViewFlag = true;

        // 🔧 여기 MAC은 네 환경에 맞게!
        pMeshRxThread->phyIface = "wlan0";
        pMeshRxThread->originatorMac = "2c:cf:67:8c:2a:7b"; // 송신 Pi wlan0 MAC
        pMeshRxThread->port = 9999;

        if (!pMeshRxThread->isRunning()) {
            pMeshRxThread->start();
            ui->pPBcamStart->setText("CamStop");
            ui->pPBsnapShot->setEnabled(true);
            ui->pCBrgb->setEnabled(true);
        }
    } else {
        pMeshRxThread->camViewFlag = false;
        ui->pPBcamStart->setText("CamStart");
        ui->pPBsnapShot->setEnabled(false);
        ui->pCBrgb->setChecked(false);
        ui->pCBrgb->setEnabled(false);
    }
}

void Tab7CamViewer::on_pPBsnapShot_clicked()
{
    pMeshRxThread->snapShot();
}

void Tab7CamViewer::tab7RecvDataSlot(QString recvData)
{
    QStringList strList = recvData.split("@"); // @ID@SNAPSHOT
    if (strList.size() >= 3 && strList[2] == "SNAPSHOT") {
        if (!ui->pPBcamStart->isChecked()) {
            on_pPBcamStart_clicked(true);
            ui->pPBcamStart->setChecked(true);
            QThread::msleep(500);
        }
        pMeshRxThread->snapShot();
    }
}

void Tab7CamViewer::on_pCBrgb_clicked(bool checked)
{
    // ✅ 체크박스를 RSSI/TQ 갱신 ON/OFF로 사용
    pMeshRxThread->setLinkPollEnabled(checked);
}
