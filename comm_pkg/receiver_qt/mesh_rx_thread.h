#ifndef MESH_RX_THREAD_H
#define MESH_RX_THREAD_H

#include <QThread>
#include <QImage>
#include <QMutex>
#include <atomic>
#include <string>
#include <opencv2/opencv.hpp>

class MeshRxThread : public QThread
{
    Q_OBJECT
public:
    explicit MeshRxThread(QObject *parent = nullptr);

    // 동작 제어
    std::atomic<bool> camViewFlag{false};

    // 설정
    std::string phyIface = "wlan0";
    std::string originatorMac = "";  // 송신 Pi wlan0 MAC
    int port = 9999;

    // 기능
    void snapShot();
    void setLinkPollEnabled(bool enable);

signals:
    void frameReady(const QImage &img);  // UI 스레드에서 setPixmap
    void resetToDefault();              // 종료 시 기본 이미지로

    //TQ 전달용 시그널
    void linkUpdated(int tq);

    void deviceInfoUpdated(const QString &text);

protected:
    void run() override;

private:
    // snapshot 공유 프레임 (스레드 안전)
    cv::Mat lastFrameBgr;
    QMutex  frameMtx;
    int     cnt = 0;

    std::atomic<bool> linkPollEnabled{false};

    // overlay 텍스트
    std::string line1 = "HOP RSSI: N/A";
    std::string line2 = "TQ: N/A";

    // TQ 전달용 저장 변수
    std::atomic<int>lastTq{-1};

};

#endif // MESH_RX_THREAD_H
