// tab7camviewer.h
#ifndef TAB7CAMVIEWER_H
#define TAB7CAMVIEWER_H

#include <QWidget>
#include <QProcess>
#include <QString>

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
    void on_pPBcamStart_clicked(bool checked);
    //void on_pPBsnapShot_clicked();
    void on_pCBrgb_clicked(bool checked);

    // (옵션) 외부 소켓 신호를 쓸 경우
    void tab7RecvDataSlot(QString recvData);

    // mesh_on/off 프로세스 처리
    void onMeshProcFinished(int exitCode, QProcess::ExitStatus status);
    void onMeshProcError(QProcess::ProcessError err);

private:
    enum class MeshAction { None, On, Off };

    void runMeshOn();
    void runMeshOff();
    void startReceiver();
    void stopReceiver();

    void setUiRunning(bool running);
    void setUiBusy(const QString &msg);

private:
    Ui::Tab7CamViewer *ui {nullptr};
    MeshRxThread *pMeshRxThread {nullptr};

    QProcess *meshProc {nullptr};
    MeshAction meshAction {MeshAction::None};

    // 스크립트 경로 (필요하면 경로만 바꿔서 사용)
    const QString meshOnScript  = "/home/pi/4project/batman/mesh_on.sh";
    const QString meshOffScript = "/home/pi/4project/batman/mesh_off.sh";
};

#endif // TAB7CAMVIEWER_H
