#ifndef TAB2SOCKETCLIENT_H
#define TAB2SOCKETCLIENT_H

#include <QWidget>
#include <QTime>
#include "socketclient.h"
#include "keyboard.h"

namespace Ui {
class Tab2SocketClient;
}

class Tab2SocketClient : public QWidget
{
    Q_OBJECT
    Ui::Tab2SocketClient *ui;
    SocketClient *pSocketClient;
    Keyboard *pKeyboard;

public:
    explicit Tab2SocketClient(QWidget *parent = nullptr);
    ~Tab2SocketClient();
    SocketClient *getpSocketClient();

signals:
    void ledWriteSig(int);
    void tab3RecvDataSig(QString);
    void tab4RecvDataSig(QString);
    void tab5RecvDataSig(QString);
    void mainWidgetRecvDataSig(int);
    void tab7RecvDataSig(QString);

private slots:
    void on_pPBServerConnect_clicked(bool checked);
    void updateRecvDataSlot(QString);
    void on_pPBSend_clicked();

    void on_pLESendData_selectionChanged();

    void on_pLERecvId_selectionChanged();

public slots:
    void sendKeyDataSlot(int);
    void sendSocketDataSlot(QString);
};

#endif // TAB2SOCKETCLIENT_H
