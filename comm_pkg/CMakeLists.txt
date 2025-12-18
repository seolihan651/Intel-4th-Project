cmake_minimum_required(VERSION 3.10)
project(AdHocStream)

find_package(OpenCV REQUIRED)

# Sender 실행파일 생성
add_executable(sender sender.cpp)
target_link_libraries(sender ${OpenCV_LIBS})

# Receiver 실행파일 생성
add_executable(receiver receiver.cpp)
target_link_libraries(receiver ${OpenCV_LIBS})