#include "my_robot_gui/ros_worker.h"
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

// 로그 기록
void writeLog(const std::string& text) {
    std::ofstream log_file("/tmp/qt_log.txt", std::ios_base::app);
    if (log_file.is_open()) {
        log_file << text << std::endl;
        log_file.close();
    }
}

RosWorker::RosWorker(QObject *parent)
    : QThread(parent)
{
    writeLog(">>> [Start] RosWorker Created!");

    // 1. ROS 2 노드 생성
    node_ = rclcpp::Node::make_shared("gui_node");

    // 2. 목표 위치 전송
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    // 3. 로봇 위치 실시간 추적 리스너
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. 지도 데이터 수신
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map",
        qos,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            writeLog(">>> [Event] Map Received!");
            emit mapUpdated(msg);
        });

    // 타이머
    timer_ = node_->create_wall_timer(100ms, std::bind(&RosWorker::checkTransform, this));

    // 화물 적재 상태
    lift_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/lift_status", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            emit liftStatusUpdated(msg->data);
        });
}

RosWorker::~RosWorker()
{
}

// 스레드 실행부
void RosWorker::run()
{
    writeLog(">>> [Thread] Spin Loop Started!");
    rclcpp::spin(node_);
}

//GUI에서 받은 좌표를 로봇에게 전송
void RosWorker::sendGoal(double x, double y, double theta)
{
    if (!goal_pub_) return;

    geometry_msgs::msg::PoseStamped msg;

    // 1. 헤더 설정
    msg.header.frame_id = "map";
    msg.header.stamp = node_->get_clock()->now();

    // 2. 좌표 설정
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0.0;

    // 3. 방향 설정
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    // 4. 전송
    goal_pub_->publish(msg);

    writeLog(">>> [Command] Goal Sent: " + std::to_string(x) + ", " + std::to_string(y));
}

//로봇 위치 확인
void RosWorker::checkTransform()
{
    geometry_msgs::msg::TransformStamped t;

    try {
        t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
        // GUI로 좌표 전송
        emit robotPositionUpdated(t.transform.translation.x, t.transform.translation.y, 0.0);

    } catch (const tf2::TransformException &ex) {
    }
}
