import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. RSSI 신호 수집 노드 (iw 명령어 사용)
        Node(
            package='relay_bot_pkg',
            executable='mesh_rssi_publisher',
            name='mesh_rssi_publisher',
            output='screen'
        ),

        # 2. 로봇 제어 및 판단 노드 (Reactive Node)
        Node(
            package='relay_bot_pkg',
            executable='relay_bot',  # setup.py의 entry_point 이름
            name='reactive_relay_bot',
            output='screen'
        ),
    ])