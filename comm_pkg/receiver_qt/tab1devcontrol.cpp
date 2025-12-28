#include "tab1devcontrol.h"
#include "ui_tab1devcontrol.h"

Tab1DevControl::Tab1DevControl(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab1DevControl)
{
    ui->setupUi(this);
    lcdData = 0;
    pQTimer = new QTimer(this);
    pLedKeyDev = new LedKeyDev(this);
    pQButtonGroup = new QButtonGroup(this);

    int rowCount = ui->gridLayout->rowCount();
    int columnCount = ui->gridLayout->columnCount();

    keyCount = rowCount * columnCount;
    for(int i =0 ;i <rowCount ; i++) {
        for(int j=0;j<columnCount;j++) {
            pQCheckBox[--keyCount] = dynamic_cast<QCheckBox*>(ui->gridLayout->itemAtPosition(i,j)->widget());
        }
    }
    keyCount = rowCount * columnCount;
    pQButtonGroup->setExclusive(false);
    for(int i=0; i<keyCount; i++)
        pQButtonGroup->addButton(pQCheckBox[i],i+1);
#if QT_VERSION >= QT_VERSION_CHECK(6,0,0)
    connect(pQButtonGroup,SIGNAL(idClicked(int)), this, SLOT(updateCheckBoxMouseSlot(int)));
#else
    connect(pQButtonGroup,SIGNAL(buttonClicked(int)), this, SLOT(updateCheckBoxMouseSlot(int)));
#endif
    connect(ui->pDialLed, SIGNAL(valueChanged(int)), pLedKeyDev, SLOT(writeLedDataSlot(int)));
    connect(pQTimer, SIGNAL(timeout()), this, SLOT(updateDialValueSlot()));
    connect(pLedKeyDev, SIGNAL(updateKeyDataSig(int)), this, SLOT(updateCheckBoxKeySlot(int)));
}

Tab1DevControl::~Tab1DevControl()
{
    delete ui;
}

void Tab1DevControl::on_pPBQuit_clicked()
{
    qApp->quit();
}


void Tab1DevControl::on_pPBTimerStart_clicked(bool checked)
{
    if(checked) {
        QString strValue = ui->pComboBoxLed->currentText();
        pQTimer->start(strValue.toInt());
        ui->pPBTimerStart->setText("TimerStop");
    } else {
        pQTimer->stop();
        ui->pPBTimerStart->setText("TimerStart");
    }
}

void Tab1DevControl::updateDialValueSlot() {

    int dialValue = ui->pDialLed->value();
    dialValue++;
    if(dialValue > ui->pDialLed->maximum())
        dialValue = 0;
    ui->pDialLed->setValue(dialValue);
    emit sendKeySig(dialValue);
}

void Tab1DevControl::on_pComboBoxLed_currentTextChanged(const QString &arg1)
{
    if(pQTimer->isActive()) {
        pQTimer->stop();
        pQTimer->start(arg1.toInt());
    }
}

void Tab1DevControl::updateCheckBoxMouseSlot(int keyNo) {
    if(pQTimer->isActive()) {
        pQTimer->stop();
        ui->pPBTimerStart->setChecked(false);
        ui->pPBTimerStart->setText("TimerStart");
    }
    lcdData = lcdData ^ (0x01 << (keyNo -1)) ;
    ui->pLcdNumberKey->display(lcdData);
//    pLedKeyDev->writeLedDataSlot((int)lcdData);
    ui->pDialLed->setValue(lcdData);

}

void Tab1DevControl::updateCheckBoxKeySlot(int keyNo) {
    if(pQTimer->isActive()) {
        pQTimer->stop();
        ui->pPBTimerStart->setChecked(false);
        ui->pPBTimerStart->setText("TimerStart");
    }
    lcdData = lcdData ^ (0x01 << (keyNo -1)) ;
    ui->pLcdNumberKey->display(lcdData);
 //   pLedKeyDev->writeLedDataSlot((int)lcdData);
 //   ui->pDialLed->setValue(lcdData);
    emit sendKeySig(keyNo);

    for(int i=0;i<keyCount;i++) {
        if(keyNo == i+1)
        {
            if(pQCheckBox[i]->isChecked())
                pQCheckBox[i]->setChecked(false);
            else
                pQCheckBox[i]->setChecked(true);
        }
    }
}

LedKeyDev* Tab1DevControl::getpLedKeyDev() {
    return pLedKeyDev;
}
