import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 환경 변수 설정 (안전을 위해 명시)
    # .bashrc에 export 되어 있더라도, 런치 파일 실행 프로세스 내에서 확실하게 잡아줍니다.
    # 사용자의 설정에 맞춰 Burger와 LDS-02를 지정합니다.
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'
    os.environ['LDS_MODEL'] = 'LDS-02'
    os.environ['ROS_DOMAIN_ID'] = '39'

    # 2. Turtlebot3 Gazebo Empty World 실행 포함
    # ros2 launch turtlebot3_gazebo empty_world.launch.py 와 동일
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_file = os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb3_launch_file)
    )

    # 3. 사용자 커스텀 노드 1: twist_rssi_sim
    # ros2 run relay_bot_pkg twist_rssi_sim
    tq_sim_node = Node(
        package='relay_bot_pkg',
        executable='tq_rssi_sim',
        name='tq_rssi_sim',
        output='screen'
    )

    # 4. 사용자 커스텀 노드 2: relay_bot
    # ros2 run relay_bot_pkg relay_bot
    relay_bot_node = Node(
        package='relay_bot_pkg',
        executable='relay_bot',
        name='relay_bot',
        output='screen'
    )

    # 5. Teleop Twist Keyboard (새 터미널 창에서 실행 + 리매핑)
    # ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_cam
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        # 새 터미널 창을 엽니다. (Ubuntu 기본 터미널 사용 시)
        prefix='gnome-terminal --', 
        output='screen',
        # 토픽 리매핑 적용
        remappings=[
            ('/cmd_vel', '/cmd_vel_cam')
        ]
    )

    return LaunchDescription([
        gazebo_cmd,
        tq_sim_node,
        relay_bot_node,
        teleop_node
    ])
