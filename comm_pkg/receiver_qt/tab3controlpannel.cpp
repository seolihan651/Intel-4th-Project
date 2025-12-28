#include "tab3controlpannel.h"
#include "ui_tab3controlpannel.h"

Tab3ControlPannel::Tab3ControlPannel(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Tab3ControlPannel)
{
    ui->setupUi(this);
    paletteOn.setColor(ui->pPBLamp->backgroundRole(),QColor(255,0,0));
    paletteOff.setColor(ui->pPBLamp->backgroundRole(),QColor(0,0,255));
    ui->pPBLamp->setPalette(paletteOff);
    ui->pPBPlug->setPalette(paletteOff);
}

Tab3ControlPannel::~Tab3ControlPannel()
{
    delete ui;
}

void Tab3ControlPannel::on_pPBLamp_clicked(bool checked)
{
    if(checked) {
        emit socketSendDataSig("[HM_CON]LAMPON");
        ui->pPBLamp->setChecked(false);
        ui->pPBLamp->setPalette(paletteOff);
    }
    else {
        emit socketSendDataSig("[HM_CON]LAMPOFF");
        ui->pPBLamp->setChecked(true);
        ui->pPBLamp->setPalette(paletteOn);
    }
}


void Tab3ControlPannel::on_pPBPlug_clicked(bool checked)
{
    if(checked) {
        emit socketSendDataSig("[HM_CON]GASON");
        ui->pPBPlug->setChecked(false);
        ui->pPBPlug->setPalette(paletteOff);
    }
    else {
        emit socketSendDataSig("[HM_CON]GASOFF");
        ui->pPBPlug->setChecked(true);
        ui->pPBPlug->setPalette(paletteOn);
    }
}

void Tab3ControlPannel::tab3RecvDataSlot(QString strData) {
    QStringList strList = strData.split("@");  //strList[0]= "";
    if(strList[2] == "LAMPON") {
        ui->pPBLamp->setChecked(true);
        ui->pPBLamp->setPalette(paletteOn);
    }
    else if(strList[2] == "LAMPOFF") {
        ui->pPBLamp->setChecked(false);
        ui->pPBLamp->setPalette(paletteOff);
    }
    else if(strList[2] == "GASON") {
        ui->pPBPlug->setChecked(true);
        ui->pPBPlug->setPalette(paletteOn);
    }
    else if(strList[2] == "GASOFF") {
        ui->pPBPlug->setChecked(false);
        ui->pPBPlug->setPalette(paletteOff);
    }
}
