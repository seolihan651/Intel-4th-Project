#include "ledkeydev.h"

LedKeyDev::LedKeyDev(QWidget *parent)
    : QWidget{parent}
{
    pQFile = new QFile(DEVFILENAME);
    if(!pQFile->open(QFile::ReadWrite | QFile::Unbuffered))
    {
        QMessageBox::information(this, "open", "open failed " + DEVFILENAME);
    }
    int ledkeyFd = pQFile->handle();
    pQSocketNotifier = new QSocketNotifier(ledkeyFd, QSocketNotifier::Read, this);
    connect(pQSocketNotifier, SIGNAL(activated(int)), this, SLOT(readKeyDataSlot(int)));

}

void LedKeyDev::readKeyDataSlot(int){

    char keyNo = 0;
    int ret = pQFile->read(&keyNo, sizeof(keyNo));
    if(ret > 0)
        emit updateKeyDataSig(int(keyNo));
    qDebug() << "keyNo : " << int(keyNo);

}
void LedKeyDev::writeLedDataSlot(int ledVal){
    char ledNo = (char)ledVal;
    pQFile->write(&ledNo, sizeof(ledNo));
}
