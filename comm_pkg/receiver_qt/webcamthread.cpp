#include "webcamthread.h"
#include <opencv2/opencv.hpp>

WebCamThread::WebCamThread(QObject *parent)
    : QThread(parent)
{
    cnt = 0;
    camViewFlag = false;
    strColor = strColorPre = "";
    rgbClassifyFlag = false;
    pQTimer = new QTimer(this);
    connect(pQTimer, SIGNAL(timeout()), this,  SLOT(rgbClassifySlot()));
}

void WebCamThread::run()
{
    VideoCapture  capture(0);
    if (!capture.isOpened())
    {
        cout << "카메라가 연결되지 않았습니다." << endl;
        exit(1);
    }
    while(camViewFlag) {

        capture.read(frame);

        cvtColor(frame, frameQt, COLOR_BGR2RGB);
        int x = frameQt.cols/2;
        int y = frameQt.rows/2;
        if(rgbClassifyFlag) {
            rgbClassifyFlag = false;
            Scalar meanHsv;
            Mat frameRoi, hsvImage;
            frameRoi = frame(Rect((x-32),(y-32), 64, 64));
            cvtColor(frameRoi,hsvImage, COLOR_BGR2HSV);
            meanHsv = mean(hsvImage);

            qDebug() << meanHsv[0];
            if( 170 <= meanHsv[0] || meanHsv[0]  < 10) //Red
                strColor = "RED";
            else if( 50 <= meanHsv[0] && meanHsv[0]  < 70) //Green
                strColor = "GREEN";
            else if( 110 <= meanHsv[0] && meanHsv[0]  < 130) //Blue
                strColor = "BLUE";
            else
                strColor = "NONE";
            if( strColor != strColorPre) {
                strColorPre = strColor;
                qDebug() << strColor;
                emit sendSocketDataSig("[1]COLOR@"+strColor);
            }
        }
        put_string(frameQt, "Count: ", Point(10, 40), cnt);
        fname = "cam_" + to_string(cnt++);
        fname += ".jpg";
        line(frameQt, Point((x-32),y),Point((x+32),y), Scalar(255,0,0),2);
        line(frameQt, Point(x, (y-32)), Point(x, (y+32)), Scalar(255,0,0),2);
        rectangle(frameQt, Point((x-32), (y-32)), Point((x+32), (y+32)), Scalar(0,255,0),2);

//        QImage qImage(frameQt.data, frameQt.cols, frameQt.rows, QImage::Format_BGR888);
        QImage qImage(frameQt.data, frameQt.cols, frameQt.rows, QImage::Format_RGB888);

        pCamView->setPixmap(QPixmap::fromImage(qImage));
    }
    capture.release();
    pCamView->setPixmap(QPixmap(":/Images/Images/display.jpg"));
}

// 문자열 출력 함수 - 그림자 효과
void WebCamThread::put_string(Mat &frame, string text, Point pt, int value)
{
    text += to_string(value);
    Point shade = pt + Point(2, 2);
    int font = FONT_HERSHEY_SIMPLEX;
    putText(frame, text, shade, font, 0.7, Scalar(0, 0, 0), 2);     // 그림자 효
    putText(frame, text, pt, font, 0.7, Scalar(120, 200, 90), 2);// 작성 문자
}
void WebCamThread::snapShot()
{
    imwrite(fname,frame);
}

void WebCamThread::rgbTimerStartStop(bool bCheck) {
    if(bCheck) {
        pQTimer->start(1000);
    }
    else if(pQTimer->isActive()){
        pQTimer->stop();
    }
}

void WebCamThread::rgbClassifySlot() {
    rgbClassifyFlag = true;
}

