#ifndef TAB1DEVCONTROL_H
#define TAB1DEVCONTROL_H

#include <QWidget>
#include <QTimer>
#include <QButtonGroup>
#include <QCheckBox>
#include "ledkeydev.h"

namespace Ui {
class Tab1DevControl;
}

class Tab1DevControl : public QWidget
{
    Q_OBJECT

public:
    explicit Tab1DevControl(QWidget *parent = nullptr);
    ~Tab1DevControl();
    LedKeyDev *getpLedKeyDev();
private slots:
    void on_pPBQuit_clicked();
    void on_pPBTimerStart_clicked(bool checked);
    void updateDialValueSlot();

    void on_pComboBoxLed_currentTextChanged(const QString &arg1);
    void updateCheckBoxMouseSlot(int);
    void updateCheckBoxKeySlot(int);
private:
    Ui::Tab1DevControl *ui;
    LedKeyDev *pLedKeyDev;
    QTimer *pQTimer;
    unsigned char lcdData;
    QButtonGroup *pQButtonGroup;
    QCheckBox *pQCheckBox[8];
    int keyCount ;

signals:
    void sendKeySig(int);

};

#endif // TAB1DEVCONTROL_H
