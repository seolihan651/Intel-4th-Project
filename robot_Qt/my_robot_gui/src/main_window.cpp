#include "my_robot_gui/main_window.h"
#include "ui_mainwindow.h"
#include <QDateTime>
#include <QDebug>
#include <cmath>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , map_item_(nullptr)
    , robot_item_(nullptr)
    , goal_item_(nullptr)
    , map_resolution_(0.05)
    , is_goal_selected_(false)
    , has_active_goal_(false)
    , prev_robot_x_(0.0)
    , prev_robot_y_(0.0)
    , is_loaded_(false)
{
    //UI 로드
    ui->setupUi(this);

    //맵 뷰어 초기화
    scene_ = new QGraphicsScene(this);
    ui->map_view->setScene(scene_);
    ui->map_view->setDragMode(QGraphicsView::ScrollHandDrag);
    ui->map_view->viewport()->installEventFilter(this);

    //초기 화면
    scene_->addRect(0, 0, 500, 500, QPen(Qt::black, 1), QBrush(Qt::NoBrush));
    QGraphicsTextItem* text_item = scene_->addText("Waiting for Map...");
    text_item->setPos(200, 240);

    //타이머
    ui_timer_ = new QTimer(this);
    connect(ui_timer_, &QTimer::timeout, this, &MainWindow::updateTime);
    ui_timer_->start(1000);

    //버튼 연결
    connect(ui->btn_set_goal, &QPushButton::clicked, this, &MainWindow::onBtnSetGoalClicked);
    connect(ui->btn_confirm, &QPushButton::clicked, this, &MainWindow::onBtnConfirmClicked);
    connect(ui->btn_delete, &QPushButton::clicked, this, &MainWindow::onBtnDeleteClicked);
    connect(ui->btn_emergency, &QPushButton::clicked, this, &MainWindow::onBtnEmergencyClicked);

    //ROS Worker
    ros_worker_ = new RosWorker(this);

    //데이터 수신 연결 (ROS->GUI)
    connect(ros_worker_, &RosWorker::mapUpdated, this, &MainWindow::updateMapUI);
    connect(ros_worker_, &RosWorker::robotPositionUpdated, this, &MainWindow::updateRobotUI);
    //적재 신호 연결
    connect(ros_worker_, &RosWorker::liftStatusUpdated, this, [this](bool is_loaded){
        this->is_loaded_ = is_loaded;
        updateMasterStatus(false, is_loaded_);
    });

    ros_worker_->start();

    // 초기 상태 설정
    updateMasterStatus(false, false);
    updateSlaveStatus(false, false);
    appendLog("[System] GUI Started.");

    ui->btn_emergency->setStyleSheet("background-color: red; color: white; font-weight: bold;");
}
MainWindow::~MainWindow()
{
    //종료 처리
    if(ros_worker_->isRunning()) {
        ros_worker_->quit();
        ros_worker_->wait();
    }
    delete ui;
}

void MainWindow::updateTime()
{

    ui->datetime_edit->setDateTime(QDateTime::currentDateTime());
}

//로그 출력 메세지 형식
void MainWindow::appendLog(const QString& msg)
{
    QString time_str = QDateTime::currentDateTime().toString("[hh:mm:ss] ");
    ui->log_text->append(time_str + msg);
}

//로그 스크롤 최신
void MainWindow::addTask(QString task_name)
{
    ui->task_list->addItem(task_name);
    ui->task_list->scrollToBottom();
}

//테스크 목록 관리
void MainWindow::completeCurrentTask()
{
    if (ui->task_list->count() > 0) {
        delete ui->task_list->takeItem(ui->task_list->count() - 1);
        appendLog("작업 완료: 목적지 도착");
    }
}

// 상태 색상 변경
// 적재됨(노랑), 이동중(빨강), 정지(파랑)
void MainWindow::updateMasterStatus(bool is_moving, bool is_loaded)
{
    QString style;
    QString text;

    if (is_loaded) {
        style = "background-color: #FFFF00; border-radius: 10px; border: 1px solid gray; min-width: 20px; min-height: 20px; max-width: 20px; max-height: 20px;";
        text = "Robot1 - Loaded";
    }
    else if (is_moving) {
        style = "background-color: #FF0000; border-radius: 10px; border: 1px solid gray; min-width: 20px; min-height: 20px; max-width: 20px; max-height: 20px;";
        text = "Robot1 - Moving";
    }
    else {
        style = "background-color: #0000FF; border-radius: 10px; border: 1px solid gray; min-width: 20px; min-height: 20px; max-width: 20px; max-height: 20px;";
        text = "Robot1 - Standby";
    }

    ui->master_status_light->setStyleSheet(style);
    ui->master_status_label->setText(text);
}

void MainWindow::updateSlaveStatus(bool is_moving, bool is_loaded)
{
    // Slave도 Master와 동일한 로직 적용
    QString style;
    QString text;

    if (is_loaded) {
        style = "background-color: #FFFF00; border-radius: 10px; border: 1px solid gray; min-width: 20px; min-height: 20px; max-width: 20px; max-height: 20px;";
        text = "Robot2 - Loaded";
    } else if (is_moving) {
        style = "background-color: #FF0000; border-radius: 10px; border: 1px solid gray; min-width: 20px; min-height: 20px; max-width: 20px; max-height: 20px;";
        text = "Robot2 - Moving";
    } else {
        style = "background-color: #0000FF; border-radius: 10px; border: 1px solid gray; min-width: 20px; min-height: 20px; max-width: 20px; max-height: 20px;";
        text = "Robot2 - Standby";
    }

    ui->slave_status_light->setStyleSheet(style);
    ui->slave_status_label->setText(text);
}

//버튼 이벤트 핸들러
void MainWindow::onBtnSetGoalClicked()
{
    appendLog("목적지 설정 모드: 지도를 클릭하세요.");
}

void MainWindow::onBtnConfirmClicked()
{
    if (!is_goal_selected_) {
        appendLog("[Warning] 지도에서 목표 위치를 먼저 클릭하세요!");
        return;
    }

    //ROS로 목표 좌표 전송 (Publisher)
    ros_worker_->sendGoal(goal_x_, goal_y_, 0.0);

    //활성화된 목표 저장
    has_active_goal_ = true;
    current_goal_x_ = goal_x_;
    current_goal_y_ = goal_y_;

    //작업 목록에 추가
    QString time_str = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString task_str = QString("[%1] 이동: X=%2, Y=%3")
                           .arg(time_str)
                           .arg(goal_x_, 0, 'f', 1)
                           .arg(goal_y_, 0, 'f', 1);

    addTask(task_str);
    appendLog("로봇에게 이동 명령을 보냈습니다.");
}

// 선택된 작업 항목 삭제
void MainWindow::onBtnDeleteClicked()
{
    QList<QListWidgetItem*> items = ui->task_list->selectedItems();
    if (items.isEmpty()) {
        appendLog("삭제할 작업이 선택되지 않았습니다.");
        return;
    }
    foreach(QListWidgetItem* item, items){
        delete ui->task_list->takeItem(ui->task_list->row(item));
    }
    appendLog("선택된 작업이 삭제되었습니다.");
}

// 긴급 정지
void MainWindow::onBtnEmergencyClicked()
{
    appendLog("[EMERGENCY] 긴급 정지!");
    has_active_goal_ = false;
    updateMasterStatus(false, is_loaded_);
}

//지도에 UI 띄우기
void MainWindow::updateMapUI(nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    //지도 데이터 파싱
    map_resolution_ = map_msg->info.resolution;
    map_origin_x_ = map_msg->info.origin.position.x;
    map_origin_y_ = map_msg->info.origin.position.y;
    map_height_ = map_msg->info.height;
    int width = map_msg->info.width;

    QImage map_image(width, map_height_, QImage::Format_RGB888);
    map_image.fill(Qt::gray);

    // ROS 데이터 색상 변환
    const std::vector<int8_t>& data = map_msg->data;
    for (int y = 0; y < map_height_; y++) {
        for (int x = 0; x < width; x++) {
            int index = x + y * width;
            if (index >= data.size()) continue;
            int8_t val = data[index];

            if (val == 0) map_image.setPixelColor(x, map_height_ - 1 - y, Qt::white);
            else if (val == 100) map_image.setPixelColor(x, map_height_ - 1 - y, Qt::black);
        }
    }

    QPixmap pixmap = QPixmap::fromImage(map_image);

    //화면에 로봇 그리기 (지도맵 크기에 따라 점 크기 조정 필요)
    if (!map_item_) {
        scene_->clear();
        map_item_ = scene_->addPixmap(pixmap);
        robot_item_ = scene_->addEllipse(0, 0, 100, 100, QPen(Qt::NoPen), QBrush(Qt::red));
        robot_item_->setZValue(10);
        appendLog("Map Received.");
    } else {
        map_item_->setPixmap(pixmap);
    }

    ui->map_view->fitInView(scene_->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void MainWindow::updateRobotUI(double x, double y, double theta)
{

    (void)theta;
    if (!map_item_ || !robot_item_) return;

    // 좌표 변환
    double grid_x = (x - map_origin_x_) / map_resolution_;
    double grid_y = (y - map_origin_y_) / map_resolution_;
    double pixel_y = map_height_ - 1 - grid_y;

    //로봇 점 이동
    robot_item_->setPos(grid_x - 5, pixel_y - 5);

    //움직임 감지
    bool is_moving = false;
    double diff = std::abs(x - prev_robot_x_) + std::abs(y - prev_robot_y_);
    if (diff > 0.001) {
        is_moving = true;
    }

    //도착 감지 로직
    if (has_active_goal_) {
        double dist_to_goal = std::sqrt(std::pow(x - current_goal_x_, 2) + std::pow(y - current_goal_y_, 2));

        // 목표와의 거리가 이내면 도착으로 간주
        if (dist_to_goal < 0.1) {
            completeCurrentTask(); // 리스트에서 작업 삭제
            has_active_goal_ = false; // 목표 달성 처리

            // 도착했으면 파란 점 삭제
            if(goal_item_) {
                scene_->removeItem(goal_item_);
                delete goal_item_;
                goal_item_ = nullptr;
            }
            is_goal_selected_ = false;
        }
    }

    //상태 업데이트
    updateMasterStatus(is_moving, is_loaded_);

    prev_robot_x_ = x;
    prev_robot_y_ = y;
}

//마우스 클릭 감지
bool MainWindow::eventFilter(QObject *obj, QEvent *event)
{
    //지도 내에서 이벤트
    if (obj == ui->map_view->viewport()) {
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                QPoint view_pos = mouseEvent->pos();
                QPointF scene_pos = ui->map_view->mapToScene(view_pos);

                if (!map_item_) return false;

                double grid_x = scene_pos.x();
                double grid_y = map_height_ - 1 - scene_pos.y();

                double world_x = grid_x * map_resolution_ + map_origin_x_;
                double world_y = grid_y * map_resolution_ + map_origin_y_;

                goal_x_ = world_x;
                goal_y_ = world_y;
                is_goal_selected_ = true;

                //목적지에 파란 점 찍기 (지도맵 크기에 따라 점 크기 조정 필요)
                if (!goal_item_) {
                    goal_item_ = scene_->addEllipse(0, 0, 100, 100, QPen(Qt::blue), QBrush(Qt::blue));
                    goal_item_->setZValue(20);
                }
                goal_item_->setPos(scene_pos.x() - 5, scene_pos.y() - 5);


                return true;
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}
