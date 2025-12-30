#include "tab7camviewer.h"
#include "ui_tab7camviewer.h"

#include <QPixmap>
#include <QImage>
#include <QThread>
#include <QMessageBox>
#include <QDebug>

Tab7CamViewer::Tab7CamViewer(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab7CamViewer)
{
    ui->setupUi(this);
    setWindowTitle("CamViewer");

    // 초기 UI
    //ui->pPBsnapShot->setEnabled(false);
    ui->pCBrgb->setEnabled(false);

    // RX Thread
    pMeshRxThread = new MeshRxThread(this);

    // 워커 → UI (QueuedConnection 강제)
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

    // mesh on/off runner
    meshProc = new QProcess(this);
    connect(meshProc, &QProcess::finished, this, &Tab7CamViewer::onMeshProcFinished);
    connect(meshProc, &QProcess::errorOccurred, this, &Tab7CamViewer::onMeshProcError);
}

Tab7CamViewer::~Tab7CamViewer()
{
    // 안전 종료
    stopReceiver();

    // meshProc는 parent(this)라 자동 해제
    delete ui;
}

void Tab7CamViewer::setUiBusy(const QString &msg)
{
    ui->pPBcamStart->setEnabled(false);
    ui->pPBcamStart->setText(msg);

    //ui->pPBsnapShot->setEnabled(false);
    ui->pCBrgb->setEnabled(false);
}

void Tab7CamViewer::setUiRunning(bool running)
{
    ui->pPBcamStart->setEnabled(true);
    ui->pPBcamStart->setChecked(running);
    ui->pPBcamStart->setText(running ? "CamStop" : "CamStart");

    //ui->pPBsnapShot->setEnabled(running);
    ui->pCBrgb->setEnabled(running);
}

void Tab7CamViewer::on_pPBcamStart_clicked(bool checked)
{
    // 이미 mesh script가 도는 중이면 무시
    if (meshProc && meshProc->state() != QProcess::NotRunning) {
        // 버튼 상태 되돌리기
        ui->pPBcamStart->setChecked(!checked);
        return;
    }

    if (checked) {
        // 1) mesh_on 실행 → 2) 성공 시 receiver 시작
        setUiBusy("MeshOn...");
        runMeshOn();
    } else {
        // 1) receiver 정지 → 2) mesh_off 실행 → 3) UI 원복
        setUiBusy("Stopping...");
        stopReceiver();

        setUiBusy("MeshOff...");
        runMeshOff();
    }
}

void Tab7CamViewer::runMeshOn()
{
    meshAction = MeshAction::On;

    // sudo -n : 비밀번호 입력 불가(=NOPASSWD 설정 필수). 안되면 즉시 실패.
    QStringList args;
    args << "-n" << meshOnScript;

    meshProc->start("sudo", args);
}

void Tab7CamViewer::runMeshOff()
{
    meshAction = MeshAction::Off;

    QStringList args;
    args << "-n" << meshOffScript;

    meshProc->start("sudo", args);
}

void Tab7CamViewer::onMeshProcFinished(int exitCode, QProcess::ExitStatus status)
{
    const QString out = QString::fromUtf8(meshProc->readAllStandardOutput());
    const QString err = QString::fromUtf8(meshProc->readAllStandardError());

    qDebug() << "[mesh]" << (meshAction == MeshAction::On ? "on" :
                             meshAction == MeshAction::Off ? "off" : "none")
             << "exitCode=" << exitCode
             << "status=" << status
             << "stdout=" << out
             << "stderr=" << err;

    if (status != QProcess::NormalExit || exitCode != 0) {
        // 실패 시 UI 복구
        setUiRunning(false);

        // mesh_on 실패면 영상 수신도 시작하면 안됨
        QString title = (meshAction == MeshAction::On) ? "mesh_on failed" : "mesh_off failed";
        QMessageBox::warning(this, title,
                             "스크립트 실행 실패 (sudoers NOPASSWD 설정 확인)\n\n"
                             "stdout:\n" + out + "\n\nstderr:\n" + err);
        meshAction = MeshAction::None;
        return;
    }

    // 성공 처리
    if (meshAction == MeshAction::On) {
        // mesh 올라왔으니 수신 시작
        startReceiver();
        setUiRunning(true);
    } else if (meshAction == MeshAction::Off) {
        // mesh 내렸으니 UI 완전 초기화
        setUiRunning(false);
    }

    meshAction = MeshAction::None;
}

void Tab7CamViewer::onMeshProcError(QProcess::ProcessError err)
{
    qDebug() << "[mesh] process error:" << err;
    setUiRunning(false);

    QMessageBox::warning(this, "mesh script error",
                         "mesh_on/off 실행 중 오류가 발생했습니다.\n"
                         "sudoers NOPASSWD 설정이 되어있는지 확인해줘.\n"
                         "(예: /etc/sudoers.d/mesh_scripts)\n");
    meshAction = MeshAction::None;
}

void Tab7CamViewer::startReceiver()
{
    if (!pMeshRxThread) return;
    if (pMeshRxThread->isRunning()) return;

    // 너 환경에 맞게 설정(필요 없으면 MeshRxThread 기본값 써도 됨)
    // 송신 Pi wlan0 MAC을 넣으면 TQ 라우팅이 더 정확해짐
    pMeshRxThread->phyIface = "wlan0";
    pMeshRxThread->originatorMac = "2c:cf:67:8c:2a:7b"; // TODO: 송신Pi MAC으로 변경
    pMeshRxThread->port = 9999;

    // RSSI/TQ 폴링 ON/OFF (체크박스 상태 반영)
    pMeshRxThread->setLinkPollEnabled(ui->pCBrgb->isChecked());

    pMeshRxThread->camViewFlag = true;
    pMeshRxThread->start();
}

void Tab7CamViewer::stopReceiver()
{
    if (!pMeshRxThread) return;

    pMeshRxThread->camViewFlag = false;
    if (pMeshRxThread->isRunning()) {
        // 너무 길게 기다리지 않게(환경 따라 조절)
        pMeshRxThread->wait(1500);
    }
}

// void Tab7CamViewer::on_pPBsnapShot_clicked()
// {
//     if (pMeshRxThread) pMeshRxThread->snapShot();
// }

void Tab7CamViewer::on_pCBrgb_clicked(bool checked)
{
    // 체크박스를 RSSI/TQ 갱신 ON/OFF로 사용
    if (pMeshRxThread) pMeshRxThread->setLinkPollEnabled(checked);
}

void Tab7CamViewer::tab7RecvDataSlot(QString recvData)
{
    // 예: @ID@SNAPSHOT 같은 프로토콜이 들어오는 경우만 처리
    QStringList strList = recvData.split("@");
    if (strList.size() >= 3 && strList[2] == "SNAPSHOT") {
        if (!ui->pPBcamStart->isChecked()) {
            ui->pPBcamStart->setChecked(true);
            on_pPBcamStart_clicked(true);
            QThread::msleep(500);
        }
        //on_pPBsnapShot_clicked();
    }
}
