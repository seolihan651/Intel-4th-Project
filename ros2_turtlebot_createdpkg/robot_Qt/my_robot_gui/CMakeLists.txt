cmake_minimum_required(VERSION 3.8)
project(my_robot_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# [해결책] Ninja 빌드 에러(RPATH)를 해결하는 핵심 명령어
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# 1. 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

# Qt6 찾기
find_package(Qt6 REQUIRED COMPONENTS Widgets Core Gui)

# 2. Qt 자동 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 3. 실행 파일 및 소스 파일 등록
add_executable(gui_node
  src/main.cpp
  src/main_window.cpp
  src/ros_worker.cpp
  src/mainwindow.ui

  include/my_robot_gui/main_window.h
  include/my_robot_gui/ros_worker.h
)

# 헤더 경로 포함
target_include_directories(gui_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 4. 라이브러리 링크
ament_target_dependencies(gui_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_msgs
  tf2_geometry_msgs
)

target_link_libraries(gui_node
  Qt6::Widgets
  Qt6::Core
  Qt6::Gui
)

# 설치 규칙
install(TARGETS gui_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
