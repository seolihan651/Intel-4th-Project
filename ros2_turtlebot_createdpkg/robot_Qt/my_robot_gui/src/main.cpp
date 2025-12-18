#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "my_robot_gui/main_window.h"
#include <cstdio>

int main(int argc, char *argv[])
{
    // 출력 버퍼링 끄기
    setvbuf(stdout, NULL, _IONBF, 0);

    // ROS2 초기화
    rclcpp::init(argc, argv);

    // Qt 애플리케이션 초기화
    QApplication a(argc, argv);

    // 메인 윈도우 생성 및 표시
    MainWindow w;
    w.show();

    // Qt 이벤트 루프 실행
    int result = a.exec();

    // 종료 시 ROS2 정리
    rclcpp::shutdown();

    return result;
}
