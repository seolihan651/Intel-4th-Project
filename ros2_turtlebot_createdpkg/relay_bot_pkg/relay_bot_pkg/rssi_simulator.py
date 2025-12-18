import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, SetEntityState
from geometry_msgs.msg import Pose, Point, Quaternion
import math
import random
import time

# [수정 1] Camera를 static=true로 변경하여 물리 엔진 영향 제거 (좌표 이동이 훨씬 부드러움)
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

# PC 모델 (변경 없음)
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

class RssiSimulator(Node):
    def __init__(self):
        super().__init__('rssi_simulator')

        # --- 설정 ---
        self.pc_position = (-2.0, 0.0)
        self.cam_center = (2.0, 0.0)
        self.cam_radius = 2.0
        self.cam_speed = 0.1
        
        self.tx_power = -30
        self.path_loss_exponent = 4.5
        
        # --- 상태 변수 ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.start_time = time.time()

        # 스폰 상태 관리 플래그
        self.spawn_req_sent = False   # 요청을 보냈는지
        self.markers_ready = False    # 스폰이 완전히 끝났는지

        # --- 통신 ---
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.pub_rssi_pc = self.create_publisher(Int32, 'rssi/pc', 10)
        self.pub_rssi_cam = self.create_publisher(Int32, 'rssi/cam', 10)

        # --- Gazebo 클라이언트 ---
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # 0.1초마다 업데이트
        self.timer = self.create_timer(0.1, self.update_rssi)
        
        self.get_logger().info("📡 RSSI Simulator with Visuals Started.")

    def spawn_markers(self):
        """Gazebo에 마커 생성 요청 (비동기 콜백 패턴 적용)"""
        if not self.spawn_client.service_is_ready():
            return

        # 1. PC 생성 요청
        req_pc = SpawnEntity.Request()
        req_pc.name = 'virtual_pc'
        req_pc.xml = PC_SDF
        req_pc.initial_pose.position.x = self.pc_position[0]
        req_pc.initial_pose.position.y = self.pc_position[1]
        self.spawn_client.call_async(req_pc)

        # 2. Camera 생성 요청
        req_cam = SpawnEntity.Request()
        req_cam.name = 'virtual_camera'
        req_cam.xml = CAM_SDF
        
        # [수정 2] Future 객체를 받아 콜백을 연결함
        future = self.spawn_client.call_async(req_cam)
        future.add_done_callback(self.spawn_done_callback)
        
        self.spawn_req_sent = True
        self.get_logger().info("⏳ Spawning markers...")

    def spawn_done_callback(self, future):
        """스폰이 완료되면 실행되는 콜백"""
        try:
            response = future.result()
            if response.success:
                self.markers_ready = True
                self.get_logger().info("✅ Markers Spawned Successfully! Starting movement.")
            else:
                self.get_logger().warn(f"⚠️ Spawn failed: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

    def update_marker_pos(self, x, y):
        """Camera 마커 위치 이동"""
        # [수정 3] 스폰이 완료되지 않았으면 이동 명령을 보내지 않음
        if not self.markers_ready or not self.set_state_client.service_is_ready():
            return

        req = SetEntityState.Request()
        req.state.name = 'virtual_camera'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.2
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = 'world'
        
        # 비동기 호출 (결과 확인용 콜백 추가 가능하나, 성능을 위해 생략)
        self.set_state_client.call_async(req)

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def calculate_rssi(self, tx_x, tx_y, rx_x, rx_y):
        distance = math.sqrt((tx_x - rx_x)**2 + (tx_y - rx_y)**2)
        if distance < 0.1: distance = 0.1
        rssi = self.tx_power - (10 * self.path_loss_exponent * math.log10(distance))
        noise = random.uniform(-2.0, 2.0)
        return int(rssi + noise)

    def update_rssi(self):
        # 마커 스폰 요청을 아직 안 했으면 시도
        if not self.spawn_req_sent:
            self.spawn_markers()
            
        current_time = time.time() - self.start_time

        # 1. 움직이는 카메라 좌표 계산
        cam_x = self.cam_center[0] + self.cam_radius * math.cos(self.cam_speed * current_time)
        cam_y = self.cam_center[1] + self.cam_radius * math.sin(self.cam_speed * current_time)

        # 2. Gazebo 시각화 업데이트 (스폰 완료 시에만 실행됨)
        self.update_marker_pos(cam_x, cam_y)

        # 3. RSSI 계산
        rssi_pc_val = self.calculate_rssi(self.pc_position[0], self.pc_position[1], self.robot_x, self.robot_y)
        rssi_cam_val = self.calculate_rssi(cam_x, cam_y, self.robot_x, self.robot_y)

        # 4. 발행
        msg_pc = Int32()
        msg_pc.data = rssi_pc_val
        self.pub_rssi_pc.publish(msg_pc)

        msg_cam = Int32()
        msg_cam.data = rssi_cam_val
        self.pub_rssi_cam.publish(msg_cam)
        
        # 디버깅: 스폰 완료 전에는 좌표 변화 로그만 출력
        if not self.markers_ready:
             self.get_logger().info('Waiting for Gazebo spawn...')

def main(args=None):
    rclpy.init(args=args)
    node = RssiSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()