/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDateTimeEdit>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpacerItem *horizontalSpacer;
    QDateTimeEdit *datetime_edit;
    QGraphicsView *map_view;
    QVBoxLayout *verticalLayout_2;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_7;
    QLabel *master_status_label;
    QLabel *master_status_light;
    QSpacerItem *horizontalSpacer_4;
    QHBoxLayout *horizontalLayout_8;
    QLabel *slave_status_label;
    QLabel *slave_status_light;
    QSpacerItem *horizontalSpacer_5;
    QGroupBox *task_group;
    QVBoxLayout *verticalLayout_3;
    QListWidget *task_list;
    QHBoxLayout *horizontalLayout_3;
    QToolButton *btn_set_goal;
    QToolButton *btn_confirm;
    QToolButton *btn_delete;
    QTextEdit *log_text;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer_2;
    QPushButton *btn_emergency;
    QSpacerItem *horizontalSpacer_3;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(541, 454);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName("gridLayout_2");
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName("horizontalLayout");
        label = new QLabel(centralwidget);
        label->setObjectName("label");
        label->setStyleSheet(QString::fromUtf8("<html><head/><body><p><span style=\" font-size:14pt; font-weight:700;\">\355\231\224\353\254\274 \354\232\264\353\260\230 \353\241\234\353\264\207 \354\240\234\354\226\264 \355\224\204\353\241\234\352\267\270\353\236\250 </span></p></body></html>"));

        horizontalLayout->addWidget(label);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        datetime_edit = new QDateTimeEdit(centralwidget);
        datetime_edit->setObjectName("datetime_edit");

        horizontalLayout->addWidget(datetime_edit);


        gridLayout_2->addLayout(horizontalLayout, 0, 0, 1, 2);

        map_view = new QGraphicsView(centralwidget);
        map_view->setObjectName("map_view");

        gridLayout_2->addWidget(map_view, 1, 0, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        master_status_label = new QLabel(centralwidget);
        master_status_label->setObjectName("master_status_label");

        horizontalLayout_7->addWidget(master_status_label);

        master_status_light = new QLabel(centralwidget);
        master_status_light->setObjectName("master_status_light");
        master_status_light->setStyleSheet(QString::fromUtf8("QWidget {\n"
"	min-width: 20px;\n"
"	min-height: 20px;	\n"
"	max-width: 20px;\n"
"	max-height: 20px;\n"
"	border-radius: 10px;  /* \353\260\230\354\247\200\353\246\204 = \354\247\200\353\246\204\354\235\230 \354\240\210\353\260\230 */\n"
"	border: 1px solid gray;\n"
"    background-color: rgb(0, 255, 0);\n"
"}\n"
""));

        horizontalLayout_7->addWidget(master_status_light);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_4);


        verticalLayout->addLayout(horizontalLayout_7);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName("horizontalLayout_8");
        slave_status_label = new QLabel(centralwidget);
        slave_status_label->setObjectName("slave_status_label");

        horizontalLayout_8->addWidget(slave_status_label);

        slave_status_light = new QLabel(centralwidget);
        slave_status_light->setObjectName("slave_status_light");
        slave_status_light->setStyleSheet(QString::fromUtf8("QWidget {\n"
"	min-width: 20px;\n"
"	min-height: 20px;	\n"
"	max-width: 20px;\n"
"	max-height: 20px;\n"
"	border-radius: 10px;  /* \353\260\230\354\247\200\353\246\204 = \354\247\200\353\246\204\354\235\230 \354\240\210\353\260\230 */\n"
"	border: 1px solid gray;\n"
"    background-color: rgb(255, 0, 0);\n"
"}\n"
""));

        horizontalLayout_8->addWidget(slave_status_light);

        horizontalSpacer_5 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_8->addItem(horizontalSpacer_5);

        horizontalLayout_8->setStretch(2, 2);

        verticalLayout->addLayout(horizontalLayout_8);


        verticalLayout_2->addLayout(verticalLayout);

        task_group = new QGroupBox(centralwidget);
        task_group->setObjectName("task_group");
        verticalLayout_3 = new QVBoxLayout(task_group);
        verticalLayout_3->setObjectName("verticalLayout_3");
        task_list = new QListWidget(task_group);
        task_list->setObjectName("task_list");

        verticalLayout_3->addWidget(task_list);


        verticalLayout_2->addWidget(task_group);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        btn_set_goal = new QToolButton(centralwidget);
        btn_set_goal->setObjectName("btn_set_goal");

        horizontalLayout_3->addWidget(btn_set_goal);

        btn_confirm = new QToolButton(centralwidget);
        btn_confirm->setObjectName("btn_confirm");

        horizontalLayout_3->addWidget(btn_confirm);

        btn_delete = new QToolButton(centralwidget);
        btn_delete->setObjectName("btn_delete");

        horizontalLayout_3->addWidget(btn_delete);


        verticalLayout_2->addLayout(horizontalLayout_3);

        verticalLayout_2->setStretch(1, 2);

        gridLayout_2->addLayout(verticalLayout_2, 1, 1, 1, 1);

        log_text = new QTextEdit(centralwidget);
        log_text->setObjectName("log_text");

        gridLayout_2->addWidget(log_text, 2, 0, 1, 2);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        btn_emergency = new QPushButton(centralwidget);
        btn_emergency->setObjectName("btn_emergency");
        btn_emergency->setStyleSheet(QString::fromUtf8("QWidget {\n"
"    background-color: rgb(225, 0, 0);\n"
"}\n"
""));

        horizontalLayout_2->addWidget(btn_emergency);

        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_3);


        gridLayout_2->addLayout(horizontalLayout_2, 3, 0, 1, 2);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 541, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Turtlebot Control", nullptr));
#if QT_CONFIG(whatsthis)
        label->setWhatsThis(QCoreApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:700;\">\355\231\224\353\254\274 \354\232\264\353\260\230 \353\241\234\353\264\207 \354\240\234\354\226\264 \355\224\204\353\241\234\352\267\270\353\236\250 </span></p></body></html>", nullptr));
#endif // QT_CONFIG(whatsthis)
        label->setText(QCoreApplication::translate("MainWindow", "\355\231\224\353\254\274 \354\232\264\353\260\230 \353\241\234\353\264\207 \354\240\234\354\226\264 \355\224\204\353\241\234\352\267\270\353\236\250", nullptr));
        master_status_label->setText(QCoreApplication::translate("MainWindow", "Robot1 Status - Activate", nullptr));
        master_status_light->setText(QString());
        slave_status_label->setText(QCoreApplication::translate("MainWindow", "Robot2 Status - Non-Activate", nullptr));
        slave_status_light->setText(QString());
        task_group->setTitle(QCoreApplication::translate("MainWindow", "Task List", nullptr));
        btn_set_goal->setText(QCoreApplication::translate("MainWindow", "\353\252\251\354\240\201\354\247\200 \354\204\244\354\240\225", nullptr));
        btn_confirm->setText(QCoreApplication::translate("MainWindow", "\355\231\225\354\235\270", nullptr));
        btn_delete->setText(QCoreApplication::translate("MainWindow", "\354\202\255\354\240\234", nullptr));
        btn_emergency->setText(QCoreApplication::translate("MainWindow", "Emergency!", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
