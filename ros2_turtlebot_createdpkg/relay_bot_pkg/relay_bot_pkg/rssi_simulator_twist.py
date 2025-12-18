import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
# [수정 1] DeleteEntity 추가
from gazebo_msgs.srv import SpawnEntity, SetEntityState, DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import math
import random
import time

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

class RssiSimulatorTwist(Node):
    def __init__(self):
        super().__init__('rssi_simulator_twist')

        # --- 설정 ---
        self.pc_position = (-2.0, 0.0)
        
        self.cam_x = 2.0
        self.cam_y = 0.0
        
        self.tx_power = -30
        self.path_loss_exponent = 6.5
        
        # --- 상태 변수 ---
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.spawn_req_sent = False
        self.markers_ready = False

        # --- 통신 ---
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        self.cam_cmd_sub = self.create_subscription(
            Twist, 'cmd_vel_cam', self.cam_cmd_callback, 10)

        self.pub_rssi_pc = self.create_publisher(Int32, 'rssi/pc', 10)
        self.pub_rssi_cam = self.create_publisher(Int32, 'rssi/cam', 10)

        # --- Gazebo 클라이언트 ---
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        # [수정 2] 삭제용 클라이언트 생성
        self.del_client = self.create_client(DeleteEntity, '/delete_entity')
        
        self.timer = self.create_timer(0.1, self.update_rssi)
        
        self.get_logger().info("🎮 Manual RSSI Simulator Started. Waiting for cmd_vel_cam...")

    def cam_cmd_callback(self, msg):
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
                # 이미 존재한다는 에러는 무시 (재시작 시 흔함)
                pass 
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

    # [수정 3] 종료 시 객체 삭제 함수
    def remove_entities(self):
        if not self.del_client.service_is_ready():
            self.get_logger().warn("⚠️ Delete service not ready.")
            return

        self.get_logger().info("🧹 Cleaning up Gazebo markers...")
        
        for name in ['virtual_pc', 'virtual_camera']:
            req = DeleteEntity.Request()
            req.name = name
            
            # 비동기 요청을 보내지만, 프로그램이 종료되기 전에 완료되어야 하므로
            # spin_until_future_complete를 사용해 잠시 대기합니다.
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

    def calculate_rssi(self, tx_x, tx_y, rx_x, rx_y):
        distance = math.sqrt((tx_x - rx_x)**2 + (tx_y - rx_y)**2)
        if distance < 0.1: distance = 0.1
        rssi = self.tx_power - (10 * self.path_loss_exponent * math.log10(distance))
        noise = random.uniform(-2.0, 2.0)
        return int(rssi + noise)

    def update_rssi(self):
        if not self.spawn_req_sent:
            self.spawn_markers()
            
        current_cam_x = self.cam_x
        current_cam_y = self.cam_y

        self.update_marker_pos(current_cam_x, current_cam_y)

        rssi_pc_val = self.calculate_rssi(self.pc_position[0], self.pc_position[1], self.robot_x, self.robot_y)
        rssi_cam_val = self.calculate_rssi(current_cam_x, current_cam_y, self.robot_x, self.robot_y)

        msg_pc = Int32()
        msg_pc.data = rssi_pc_val
        self.pub_rssi_pc.publish(msg_pc)

        msg_cam = Int32()
        msg_cam.data = rssi_cam_val
        self.pub_rssi_cam.publish(msg_cam)
        
        if not self.markers_ready:
             self.get_logger().info('Waiting for Gazebo spawn...')

def main(args=None):
    rclpy.init(args=args)
    node = RssiSimulatorTwist()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # [수정 4] Ctrl+C 감지 시 삭제 수행
        node.get_logger().info("\n🛑 Shutting down...")
        node.remove_entities()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()