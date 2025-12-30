// tab7camviewer.h
#ifndef TAB7CAMVIEWER_H
#define TAB7CAMVIEWER_H

#include <QWidget>
#include <QProcess>
#include <QString>
#include <QHash>
#include <QTimer>

#include "mesh_rx_thread.h"

namespace Ui { class Tab7CamViewer; }

class Tab7CamViewer : public QWidget
{
    Q_OBJECT

public:
    explicit Tab7CamViewer(QWidget *parent = nullptr);
    ~Tab7CamViewer() override;

private slots:
    // UI slots (Designer objectName 기반)


    void on_pPBtoggleCam_clicked(bool checked);


    // mesh_on/off 프로세스 처리
    void onMeshProcFinished(int exitCode, QProcess::ExitStatus status);
    void onMeshProcError(QProcess::ProcessError err);

    // TQ 업데이트
    void onLinkUpdated(int Tq);

private:
    enum class MeshAction { None, On, Off };

    void runMeshOn();
    void runMeshOff();
    void startReceiver();
    void stopReceiver();

    void setWifiIcon(bool connected);
    void resetLinkUi();

    //디바이스 출력용
    void startDeviceInfoTimer();
    void stopDeviceInfoTimer();
    void updateDeviceInfo();
    void runBatctlO();
    void runIpNeigh();
    void renderDeviceInfo();



private:
    Ui::Tab7CamViewer *ui {nullptr};
    MeshRxThread *pMeshRxThread {nullptr};

    QProcess *meshProc {nullptr};
    MeshAction meshAction {MeshAction::None};

    // 스크립트 경로 (필요하면 경로만 바꿔서 사용)
    const QString meshOnScript  = "/home/pi/4project/batman/mesh_on.sh";
    const QString meshOffScript = "/home/pi/4project/batman/mesh_off.sh";

    // 디바이스 정보 출력용
    QTimer   *m_devTimer = nullptr;
    QProcess *m_batProc  = nullptr;
    QProcess *m_neighProc= nullptr;

    QStringList m_macs;                 // batctl o에서 얻은 MAC 목록
    QHash<QString, QString> m_macToIp;  // ip neigh에서 얻은 MAC->IP 맵


signals:
    void deviceInfoUpdated(const QString &text);
};

#endif // TAB7CAMVIEWER_H
