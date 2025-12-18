#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QTimer>
#include <QEvent>
#include <QMouseEvent>
#include "ros_worker.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void addTask(QString task_name); //task 추가
    void completeCurrentTask(); //task 완료시 제거

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

public slots:
    void updateMapUI(nav_msgs::msg::OccupancyGrid::SharedPtr map_msg); //지도 적재
    void updateRobotUI(double x, double y, double theta); //로봇 표시 UI

    void onBtnSetGoalClicked();
    void onBtnConfirmClicked();
    void onBtnDeleteClicked();
    void onBtnEmergencyClicked();

    void updateTime();

private:
    void appendLog(const QString& msg);

    // 상태 업데이트(라벨+색상 변경)
    void updateMasterStatus(bool is_moving, bool is_loaded);
    void updateSlaveStatus(bool is_moving, bool is_loaded);

private:
    Ui::MainWindow *ui;
    RosWorker* ros_worker_;
    QTimer* ui_timer_;

    QGraphicsScene* scene_;
    QGraphicsPixmapItem* map_item_;
    QGraphicsEllipseItem* robot_item_;
    QGraphicsEllipseItem* goal_item_;

    //좌표 변환용
    double map_resolution_;
    double map_origin_x_;
    double map_origin_y_;
    int map_height_;

    //목표 설정 임시
    double goal_x_;
    double goal_y_;
    bool is_goal_selected_;

    //실제 주행 상태
    bool has_active_goal_;
    double current_goal_x_;
    double current_goal_y_;

    //움직임 감지
    double prev_robot_x_;
    double prev_robot_y_;

    //화물 상태
    bool is_loaded_;
};

#endif // MAIN_WINDOW_H
