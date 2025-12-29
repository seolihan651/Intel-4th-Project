import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, SetEntityState, DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import math
import random
import time

# 시각적 확인을 위한 Gazebo 모델 (변경 없음)
CAM_SDF = """
<sdf version='1.6'>
  <model name='virtual_camera'>
    <static>true</static>
    <pose>0 0 0.2 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><sphere><radius>0.2</radius></sphere></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Red</name></script></material>
      </visual>
    </link>
  </model>
</sdf>
"""

PC_SDF = """
<sdf version='1.6'>
  <model name='virtual_pc'>
    <static>true</static>
    <pose>0 0 0.2 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>0.3 0.3 0.3</size></box></geometry>
        <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Green</name></script></material>
      </visual>
    </link>
  </model>
</sdf>
"""

class TqSimulatorTwist(Node):
    def __init__(self):
        super().__init__('tq_simulator_twist')

        # --- 설정 ---
        self.pc_position = (-2.0, 0.0) # PC 위치 고정
        
        self.cam_x = 2.0  # 가상 카메라(타겟) 초기 위치
        self.cam_y = 0.0
        
        # --- 상태 변수 ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.spawn_req_sent = False
        self.markers_ready = False

        # --- 통신 ---
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # 가상 카메라를 움직이기 위한 토픽
        self.cam_cmd_sub = self.create_subscription(
            Twist, 'cmd_vel_cam', self.cam_cmd_callback, 10)

        # [중요] 이름은 rssi/pc 지만 실제 데이터는 TQ (0~255)
        self.pub_tq_pc = self.create_publisher(Int32, 'tq/pc', 10)
        self.pub_tq_cam = self.create_publisher(Int32, 'tq/cam', 10)

        # --- Gazebo 클라이언트 ---
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.del_client = self.create_client(DeleteEntity, '/delete_entity')
        
        self.timer = self.create_timer(0.1, self.update_tq)
        
        self.get_logger().info("🦇 TQ Simulator Started. (Range: 0-255)")

    def cam_cmd_callback(self, msg):
        # 가상 카메라 위치 이동 (WASD로 조종 가능하게)
        step_scale = 0.5
        self.cam_x += msg.linear.x * step_scale
        self.cam_y += msg.angular.z * step_scale 

    def spawn_markers(self):
        if not self.spawn_client.service_is_ready():
            return

        # 1. PC 생성
        req_pc = SpawnEntity.Request()
        req_pc.name = 'virtual_pc'
        req_pc.xml = PC_SDF
        req_pc.initial_pose.position.x = self.pc_position[0]
        req_pc.initial_pose.position.y = self.pc_position[1]
        self.spawn_client.call_async(req_pc)

        # 2. Camera 생성
        req_cam = SpawnEntity.Request()
        req_cam.name = 'virtual_camera'
        req_cam.xml = CAM_SDF
        req_cam.initial_pose.position.x = self.cam_x
        req_cam.initial_pose.position.y = self.cam_y
        
        future = self.spawn_client.call_async(req_cam)
        future.add_done_callback(self.spawn_done_callback)
        
        self.spawn_req_sent = True

    def spawn_done_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.markers_ready = True
                self.get_logger().info("✅ Markers Spawned!")
            else:
                pass 
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

    def remove_entities(self):
        if not self.del_client.service_is_ready():
            return

        self.get_logger().info("🧹 Cleaning up Gazebo markers...")
        
        for name in ['virtual_pc', 'virtual_camera']:
            req = DeleteEntity.Request()
            req.name = name
            future = self.del_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def update_marker_pos(self, x, y):
        if not self.markers_ready or not self.set_state_client.service_is_ready():
            return

        req = SetEntityState.Request()
        req.state.name = 'virtual_camera'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.2
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        
        self.set_state_client.call_async(req)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def calculate_tq(self, tx_x, tx_y, rx_x, rx_y):
        """
        [TQ 시뮬레이션 로직]
        거리 기반으로 TQ(0~255) 값을 생성합니다.
        - 가까우면 255 유지
        - 멀어지면 선형적으로 감소
        """
        distance = math.sqrt((tx_x - rx_x)**2 + (tx_y - rx_y)**2)
        
        # 설정값
        PERFECT_DIST = 3.0   # 3m 이내는 TQ 255 (완벽)
        MAX_DIST = 12.0      # 12m 이상은 TQ 0 (끊김)
        
        if distance <= PERFECT_DIST:
            base_tq = 255.0
        elif distance >= MAX_DIST:
            base_tq = 0.0
        else:
            # 3m ~ 12m 사이: 거리 9m 동안 255점이 떨어짐
            # 점수 = 255 - (초과거리 * 기울기)
            slope = 255.0 / (MAX_DIST - PERFECT_DIST)
            base_tq = 255.0 - ((distance - PERFECT_DIST) * slope)
            
        # 노이즈 추가 (TQ는 통신 환경에 따라 튀는 경향이 있음)
        noise = random.randint(-5, 5)
        final_tq = int(base_tq + noise)
        
        # 0 ~ 255 범위 제한 (Clamp)
        return max(0, min(255, final_tq))

    def update_tq(self):
        if not self.spawn_req_sent:
            self.spawn_markers()
            
        current_cam_x = self.cam_x
        current_cam_y = self.cam_y

        # 가상 마커(카메라) 위치 업데이트
        self.update_marker_pos(current_cam_x, current_cam_y)

        # 로봇과 PC, 로봇과 카메라 사이의 TQ 계산
        tq_pc_val = self.calculate_tq(self.pc_position[0], self.pc_position[1], self.robot_x, self.robot_y)
        tq_cam_val = self.calculate_tq(current_cam_x, current_cam_y, self.robot_x, self.robot_y)

        # 발행
        msg_pc = Int32()
        msg_pc.data = tq_pc_val
        self.pub_tq_pc.publish(msg_pc)

        msg_cam = Int32()
        msg_cam.data = tq_cam_val
        self.pub_tq_cam.publish(msg_cam)
        
        # 디버깅 출력 (주석 해제 시 사용)
        # self.get_logger().info(f"PC TQ: {tq_pc_val} | Cam TQ: {tq_cam_val}")

def main(args=None):
    rclpy.init(args=args)
    node = TqSimulatorTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Shutting down...")
        node.remove_entities()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()