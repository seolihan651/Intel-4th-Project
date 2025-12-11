#ifndef ROS_WORKER_H
#define ROS_WORKER_H

#include <QThread>
#include <QObject>
#include <QImage>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RosWorker : public QThread
{
    Q_OBJECT

public:
    explicit RosWorker(QObject *parent = nullptr);
    ~RosWorker();
    void run() override;
    void sendGoal(double x, double y, double theta);

signals:
    // 로봇 위치
	void robotPositionUpdated(double x, double y, double theta);

    // 지도
    void mapUpdated(nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);

    // 로봇 상태
	void statusUpdated(QString status_msg);

    // 화물 위치
	void cargoPositionUpdated(double x, double y);

    // 화물 적재 상태
    void liftStatusUpdated(bool is_loaded);

protected:
    // 로봇 위치 확인
    void checkTransform();

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

#endif // ROS_WORKER_H
