#ifndef WEBCAMTHREAD_H
#define WEBCAMTHREAD_H

#include <QThread>
#include <QLabel>
#include <QTimer>

#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;
class WebCamThread : public QThread
{
    Q_OBJECT
    void run();
    int cnt;
    string fname;
    QString strColor, strColorPre;
    Mat frame, frameQt;
    void put_string(Mat &frame, string text, Point pt, int value);
    QTimer *pQTimer;
    bool rgbClassifyFlag;

public:
    WebCamThread(QObject *parent = nullptr);
    bool camViewFlag;
    QLabel *pCamView;
    void snapShot();
    void rgbTimerStartStop(bool);

private slots:
    void rgbClassifySlot();

signals:
    void sendSocketDataSig(QString);
};

#endif // WEBCAMTHREAD_H
