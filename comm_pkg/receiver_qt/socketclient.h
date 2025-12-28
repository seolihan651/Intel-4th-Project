#ifndef SOCKETCLIENT_H
#define SOCKETCLIENT_H

#include <QWidget>
#include <QTcpSocket>
#include <QHostAddress>
#include <QInputDialog>
#include <QDebug>
#include <QMessageBox>

#define BLOCK_SIZE 1024
class SocketClient : public QWidget
{
    Q_OBJECT
    QTcpSocket *pQTcpSocket;
    QString SERVERIP = "10.10.14.251";
    int SERVERPORT = 5000;
    QString LOGID = "KSH_QT";
    QString LOGPW = "PASSWD";

public:
    explicit SocketClient(QWidget *parent = nullptr);
    ~SocketClient();

signals:
    void socketRecvDataSig(QString strRecvData);

private slots:
    void socketReadDataSlot();
    void socketErrorSlot();
    void socketConnectServerSlot();

public slots:
    void connectToServerSlot(bool &);
    void socketClosedServerSlot();
    void socketWriteDataSlot(QString);

signals:
};

#endif // SOCKETCLIENT_H
