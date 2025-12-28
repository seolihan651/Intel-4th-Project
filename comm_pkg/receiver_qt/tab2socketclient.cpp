#include "tab2socketclient.h"
#include "ui_tab2socketclient.h"

Tab2SocketClient::Tab2SocketClient(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab2SocketClient)
{
    ui->setupUi(this);
    pKeyboard = new Keyboard();
    pSocketClient = new SocketClient(this);
    connect(pSocketClient, SIGNAL(socketRecvDataSig(QString)), this ,SLOT(updateRecvDataSlot(QString)));

}

Tab2SocketClient::~Tab2SocketClient()
{
    delete ui;
}

void Tab2SocketClient::on_pPBServerConnect_clicked(bool checked)
{
    bool bFlag;
    if(checked) {
        pSocketClient->connectToServerSlot(bFlag);
        if(bFlag)
        {
            ui->pPBServerConnect->setText("서버 해제");
        }
    } else {
        pSocketClient->socketClosedServerSlot();
        ui->pPBServerConnect->setText("서버 연결");
    }
}

void Tab2SocketClient::updateRecvDataSlot(QString strRecvData) {
    strRecvData.chop(1);       //끝 '\n'문자 제거
    QString tempStr = strRecvData;
    QTime vQTime = QTime::currentTime();
    QString strTime = vQTime.toString() + " " + strRecvData;

    ui->pTERecvData->append(strTime);   //[KSH_QT]LED@0xff ==> @KSH_QT@LED@0xff
    strRecvData.replace("[","@");
    strRecvData.replace("]","@");
    QStringList strList = strRecvData.split("@");  //strList[0]= "";
    //strList[0]= "" ,//strList[1]= "KSH_QT", strList[2]= "LED", strList[3] = "0xff"
    if(strList[2].compare("LED") == 0)
    {
        bool bFlag;
        int ledNo = strList[3].toInt(&bFlag, 16);
        if(bFlag)
            emit ledWriteSig(ledNo);
    } else if((strList[2].indexOf("LAMP") == 0) ||(strList[2].indexOf("GAS") == 0)){
        emit tab3RecvDataSig(strRecvData);
    } else if(strList[2].compare("SENSOR") == 0) {
        emit tab4RecvDataSig(strRecvData);
        emit tab5RecvDataSig(strRecvData);
    } else if(strList[2].compare("TABIDX") == 0) {
//        qDebug() << "tabidx : " << strList[3] << " " << strList[3].toInt();
        emit mainWidgetRecvDataSig(strList[3].toInt());
        sendSocketDataSlot(tempStr);
    } else if(strList[2].compare("SNAPSHOT") == 0) {
        emit tab7RecvDataSig(strRecvData);
        sendSocketDataSlot(tempStr);
    }
}

void Tab2SocketClient::on_pPBSend_clicked()
{
    QString strRecvId = ui->pLERecvId->text();
    QString strSendData = ui->pLESendData->text();
    if(strRecvId.isEmpty()) {
        strSendData = "[ALLMSG]" + strSendData;
    } else {
        strSendData = "["+strRecvId+"]" + strSendData;
    }
    pSocketClient->socketWriteDataSlot(strSendData);
    ui->pLESendData->clear();
}
SocketClient* Tab2SocketClient::getpSocketClient() {
    return pSocketClient;
}

void Tab2SocketClient::sendKeyDataSlot(int keyData) {
    QString strKeyData = "[10]KEY@"+QString::number(keyData);
    pSocketClient->socketWriteDataSlot(strKeyData);
}

void Tab2SocketClient::on_pLESendData_selectionChanged()
{
    QLineEdit *pQLineEdit = (QLineEdit*)sender();
    pKeyboard->setLineEdit(pQLineEdit);
    pKeyboard->show();
}


void Tab2SocketClient::on_pLERecvId_selectionChanged()
{
    QLineEdit *pQLineEdit = (QLineEdit*)sender();
    pKeyboard->setLineEdit(pQLineEdit);
    pKeyboard->show();
}

void Tab2SocketClient::sendSocketDataSlot(QString strData) {
    pSocketClient->socketWriteDataSlot(strData);
}
