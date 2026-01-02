// tab7camviewer.h
#ifndef TAB7CAMVIEWER_H
#define TAB7CAMVIEWER_H

#include <QWidget>
#include <QDebug>
#include "mesh_rx_thread.h"     // ✅ 변경

namespace Ui { class Tab7CamViewer; }

class Tab7CamViewer : public QWidget
{
    Q_OBJECT
public:
    explicit Tab7CamViewer(QWidget *parent = nullptr);
    ~Tab7CamViewer();

    // ✅ mainwidget.cpp에서 더 이상 getpWebCamThread() 안 쓸 거라면 삭제해도 됨
    // MeshRxThread* getpMeshRxThread();

private:
    Ui::Tab7CamViewer *ui;
    MeshRxThread *pMeshRxThread;   // ✅ 변경

private slots:
    void on_pPBcamStart_clicked(bool checked);
    void on_pPBsnapShot_clicked();
    void tab7RecvDataSlot(QString);
    void on_pCBrgb_clicked(bool checked);
};

#endif
