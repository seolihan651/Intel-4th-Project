import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # TurtleBot3 Bringup 패키지 경로 찾기
    bringup_dir = get_package_share_directory('turtlebot3_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    return LaunchDescription([
        # 1. 터틀봇3 기본 드라이버 실행 (Lidar, Motor, TF 등)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot.launch.py')),
            # 필요시 인자 전달 (모델명 등은 환경변수 TURTLEBOT3_MODEL을 따름)
            launch_arguments={'use_sim_time': 'false'}.items(),
        ),

        # 2. RSSI 신호 수집 노드
        Node(
            package='relay_bot_pkg',
            executable='mesh_rssi_publisher',
            name='mesh_rssi_publisher',
            output='screen'
        ),

        # 3. 로봇 제어 및 판단 노드
        Node(
            package='relay_bot_pkg',
            executable='relay_bot',
            name='reactive_relay_bot',
            output='screen'
        ),
    ])