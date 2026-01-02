#include "tab7camviewer.h"
#include "ui_tab7camviewer.h"

#include <QPixmap>
#include <QImage>
#include <QThread>
#include <QMessageBox>
#include <QDebug>
#include <QRegularExpression>
#include <QDateTime>


Tab7CamViewer::Tab7CamViewer(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab7CamViewer)
{
    ui->setupUi(this);
    setWindowTitle("CamViewer");

    // 초기 UI
    //ui->pPBsnapShot->setEnabled(false);

    ui->pRightPanel->setStyleSheet(R"(pRightPanel {
    background-color: #2C3E50; /* 어두운 배경 */
    color: white;
})");

    // RX Thread
    pMeshRxThread = new MeshRxThread(this);

    // 워커 → UI (QueuedConnection 강제)
    connect(pMeshRxThread, &MeshRxThread::frameReady, this,
            [this](const QImage &img){
                if (img.isNull()) return;
                ui->plabelCamView->setPixmap(
                    QPixmap::fromImage(img).scaled(
                        ui->plabelCamView->size(),
                        //Qt::KeepAspectRatio,
                        Qt::IgnoreAspectRatio,
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

    connect(pMeshRxThread, &MeshRxThread::linkUpdated, this, &Tab7CamViewer::onLinkUpdated,Qt::QueuedConnection);

    //Toggle Cam 설정
    ui->pPBtoggleCam->setCheckable(true);
    //ui->pPBtoggleCam->setFixedSize(56, 30);
    ui->QlabelWIFIStatusIcon->setPixmap(QPixmap(":/Images/WIFI_OFF.png"));

    // 게이지바  설정
    ui->pTQValuebar->setTextVisible(true);

    // mesh on/off runner
    meshProc = new QProcess(this);
    connect(meshProc, &QProcess::finished, this, &Tab7CamViewer::onMeshProcFinished);
    connect(meshProc, &QProcess::errorOccurred, this, &Tab7CamViewer::onMeshProcError);
    resetLinkUi();
}

Tab7CamViewer::~Tab7CamViewer()
{
    // 안전 종료
    stopReceiver();

    // meshProc는 parent(this)라 자동 해제
    delete ui;
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
        //setUiRunning(false);

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
        startDeviceInfoTimer();

    } else if (meshAction == MeshAction::Off) {
        // mesh 내렸으니 UI 완전 초기화
        //setUiRunning(false);
    }

    meshAction = MeshAction::None;
}

void Tab7CamViewer::onMeshProcError(QProcess::ProcessError err)
{
    qDebug() << "[mesh] process error:" << err;
    //setUiRunning(false);

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

    // 송신 Pi wlan0 MAC을 넣으면 TQ 라우팅이 더 정확해짐
    pMeshRxThread->phyIface = "wlan0";
    pMeshRxThread->originatorMac = "2c:cf:67:8c:2a:7b"; // TODO: 송신Pi MAC으로 변경
    pMeshRxThread->port = 9999;


    // RSSI/TQ 폴링 ON/OFF (체크박스 상태 반영)
    pMeshRxThread->setLinkPollEnabled(ui->pPBtoggleCam->isChecked());
    pMeshRxThread->setLinkPollEnabled(true);
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


void Tab7CamViewer::on_pPBtoggleCam_clicked(bool checked)
{

    if (pMeshRxThread) pMeshRxThread->setLinkPollEnabled(checked);

    // 이미 mesh script가 도는 중이면 무시
    if (meshProc && meshProc->state() != QProcess::NotRunning) {
        // 버튼 상태 되돌리기
        ui->pPBtoggleCam->setChecked(!checked);
        return;
    }

    if (checked) {

        runMeshOn();
    } else {
        stopDeviceInfoTimer();
        m_macs.clear();
        ui->pDeviceList->clear();
        ui->QlabeDeviceInfo->setText("DEVICE INFO (0 Devices)");
        resetLinkUi();
        stopReceiver();
        runMeshOff();

    }
}
void Tab7CamViewer::setWifiIcon(bool connected)
{
    const QString path = connected ? ":/Images/WIFI_ON.png" : ":/Images/WIFI_OFF.png";

    if(connected) ui->QlabelWiFiStatus->setText("Connect");
    else ui->QlabelWiFiStatus->setText("Uncnnect");

    QPixmap pm(path);
    ui->QlabelWIFIStatusIcon->setPixmap(pm.scaled(QSize(40,40),Qt::KeepAspectRatio,Qt::SmoothTransformation));
}



void Tab7CamViewer::resetLinkUi()
{
    setWifiIcon(false);
    ui->pTQValuebar->setValue(0);
    ui->pTQValuebar->setFormat("0/255");
}

void Tab7CamViewer::onLinkUpdated(int tq)
{
    ui->pTQValuebar->setRange(0,255);

    const bool connected = (tq >= 0);
    setWifiIcon(connected);

    if (connected) {
        ui->pTQValuebar->setValue(tq);
        ui->pTQValuebar->setFormat(QString("%1/255").arg(tq));
    } else {
        ui->pTQValuebar->setValue(0);
        ui->pTQValuebar->setFormat("0/255");
    }

}

// 1) 타이머 시작/정지 (토글 ON/OFF에 맞춰 호출 추천)
void Tab7CamViewer::startDeviceInfoTimer()
{
    if (!m_devTimer) {
        m_devTimer = new QTimer(this);
        m_devTimer->setInterval(1000); // 1초
        connect(m_devTimer, &QTimer::timeout, this, &Tab7CamViewer::updateDeviceInfo);
    }
    m_devTimer->start();
    updateDeviceInfo(); // 바로 1회 갱신
}

void Tab7CamViewer::stopDeviceInfoTimer()
{
    if (m_devTimer) m_devTimer->stop();
}

// 2) 1초마다 호출되는 엔트리
void Tab7CamViewer::updateDeviceInfo()
{
    QString currentTime = QDateTime::currentDateTime().toString("hh:mm:ss");
    ui->QlabelClock->setText(currentTime);

    // 프로세스 겹침 방지
    if (m_batProc && m_batProc->state() != QProcess::NotRunning) return;
    if (m_neighProc && m_neighProc->state() != QProcess::NotRunning) return;

    runBatctlO(); // batctl o -> 끝나면 ip neigh 실행 -> 끝나면 render
}

// 3) batctl o 실행 & 파싱
void Tab7CamViewer::runBatctlO()
{
    if (!m_batProc) {
        m_batProc = new QProcess(this);
        connect(m_batProc, QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this](int, QProcess::ExitStatus){
                    const QString out = QString::fromUtf8(m_batProc->readAllStandardOutput());
                    m_macs.clear();

                    // [중요] 1. 내 장치(HOST/pi19)는 batctl n에 안 뜨므로 강제로 추가
                    m_macs.append("2c:cf:67:8c:2a:13");

                    // 2. 이웃 장치 파싱 (batctl n)
                    QRegularExpression re(R"(\S+\s+([0-9a-fA-F:]{17})\s+([0-9.]+)s)");
                    QRegularExpressionMatchIterator it = re.globalMatch(out);

                    while (it.hasNext()) {
                        QRegularExpressionMatch match = it.next();
                        QString mac = match.captured(1).toLower();
                        QString timeStr = match.captured(2);

                        bool ok;
                        double seenSec = timeStr.toDouble(&ok);

                        // 60초 이내인 이웃만 추가
                        if (ok && seenSec <= 60.0) {
                            if (!m_macs.contains(mac)) {
                                m_macs << mac;
                            }
                        }
                    }
                    runIpNeigh();
                });
    }
    m_batProc->start("batctl", QStringList() << "n");
}

// 4) ip neigh 실행 & 파싱 (IP가 없으면 Unknown 유지)
void Tab7CamViewer::runIpNeigh()
{
    if (!m_neighProc) {
        m_neighProc = new QProcess(this);
        connect(m_neighProc, QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this](int, QProcess::ExitStatus){
                    const QString out = QString::fromUtf8(m_neighProc->readAllStandardOutput());
                    m_macToIp.clear();

                    // 예: "10.10.14.79 dev bat0 lladdr 00:14:1b:... REACHABLE"
                    QRegularExpression re(R"(^\s*([0-9]+\.[0-9]+\.[0-9]+\.[0-9]+)\s+dev\s+bat0\s+lladdr\s+([0-9a-fA-F:]{17})\b)");
                    for (const QString &line : out.split('\n')) {
                        auto m = re.match(line);
                        if (m.hasMatch()) {
                            QString ip  = m.captured(1);
                            QString mac = m.captured(2).toLower();
                            m_macToIp.insert(mac, ip);
                        }
                    }
                    renderDeviceInfo();
                });
    }

    m_neighProc->start("ip", QStringList() << "neigh" << "show" << "dev" << "bat0");
}

void Tab7CamViewer::renderDeviceInfo()
{
    ui->pDeviceList->clear();

    // 상단 라벨 업데이트
    ui->QlabeDeviceInfo->setText(QString("DEVICE INFO (%1 Devices)").arg(m_macs.size()));

    int idx = 1;
    for (const QString &mac : m_macs) {

        QString roleName;
        QString ipAddr;
        QString displayMac = mac.toUpper();

        //  MAC 주소에 따라 역할(Role)과 IP를 수동으로 지정
        if (mac == "2c:cf:67:8c:2a:13") {
            roleName = "HOST";  // 수신
            ipAddr   = "10.50.0.10";   //
        }
        else if (mac == "2c:cf:67:8c:2a:7b") {
            roleName = "CAM";   // 송신
            ipAddr   = "10.50.0.11";
        }
        else if (mac == "d8:3a:dd:5a:d8:f8") {
            roleName = "BOT";   // 중계
            ipAddr   = "10.50.0.12";
        }
        else {
            // 표에 없는 새로운 장치가 붙었을 경우
            roleName = QString("Guest Device %1").arg(idx);
            // ip neigh에서 IP 찾기 (없으면 Unknown)
            ipAddr   = m_macToIp.value(mac, "Unknown");
        }

        // 리스트에 보여줄 텍스트 구성
        // 예: HOST (pi19)
        //     MAC: 2C:CF...
        //     IP : 10.50.0.10
        QString itemText = QString("%1\nIP: %2\nMAC: %3\n")
                                .arg(roleName)
                                .arg(ipAddr)
                                .arg(displayMac);

        // 리스트 아이템 생성 및 추가
        QListWidgetItem *item = new QListWidgetItem(itemText, ui->pDeviceList);
        item->setTextAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        ui->pDeviceList->addItem(item);

        idx++;
    }
}
