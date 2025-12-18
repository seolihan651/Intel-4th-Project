import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32

class ReactiveRelayBot(Node):
    def __init__(self):
        super().__init__('reactive_relay_bot')
        
        # QoS 설정 (센서 데이터는 최신값이 중요하므로)
        qos_policy = QoSProfile(depth=10)
        
        # Subscriber: 라이다 데이터 수신
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_policy)
            
        # Publisher: 로봇 속도 명령
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 타이머: 0.1초마다 제어 판단
        self.timer = self.create_timer(0.1, self.control_loop)

        # 상태 변수
        self.obstacle_detected = False
        self.escape_direction = 0.0 # 회피할 방향 (양수: 좌회전, 음수: 우회전)
        
        # RSSI 변수 (실제 구현 시 Wifi 스캐너에서 업데이트 받아야 함)
        self.rssi_pc = -99 
        self.rssi_cam = -99

        self.prev_score = -200.0  # 이전 평가 점수
        self.action_state = 'FORWARD' # 현재 행동 상태 (FORWARD, TURN)
        self.state_timer = 0  # 행동 지속 시간 카운터
        self.turn_direction = 1.0 # 1.0(좌), -1.0(우)
        
        self.rssi_pc_sub = self.create_subscription(
            Int32, 'rssi/pc', self.rssi_pc_callback, 10)
            
        self.rssi_cam_sub = self.create_subscription(
            Int32, 'rssi/cam', self.rssi_cam_callback, 10)

    def scan_callback(self, msg):
        """
        LIDAR 데이터를 받아 전방 장애물 여부를 판단합니다.
        Turtlebot3 LDS-01/02 기준: ranges[0]이 정면입니다.
        """
        scan_ranges = msg.ranges
        
        # 노이즈(0.0)나 무한대(inf) 처리 -> 최대 거리로 치환
        cleaned_ranges = [r if r > 0.0 else 10.0 for r in scan_ranges]
        
        # 1. 전방 감지 (정면 기준 좌우 30도)
        # Python 리스트 슬라이싱: 뒤쪽 30개 + 앞쪽 30개
        front_ranges = cleaned_ranges[-30:] + cleaned_ranges[:30]
        min_front_dist = min(front_ranges)
        
        # 2. 좌/우 거리 감지 (회피 방향 결정을 위해)
        left_dist = min(cleaned_ranges[30:90])
        right_dist = min(cleaned_ranges[270:330])

        # 3. 충돌 위험 판단 (0.35m 이내에 물체 감지 시)
        collision_threshold = 0.35
        
        if min_front_dist < collision_threshold:
            self.obstacle_detected = True
            # 더 넓은 쪽으로 회전 방향 결정
            if left_dist > right_dist:
                self.escape_direction = 0.5  # 좌회전 (왼쪽이 넓으니까)
            else:
                self.escape_direction = -0.5 # 우회전
        else:
            self.obstacle_detected = False

    def get_rssi_command(self):
        """
        이전에 논의했던 RSSI 기반 Gradient Ascent 로직이 들어갈 곳
        여기서는 예시로 직진 명령만 반환합니다.
        """
        cmd = Twist()
        # 1. 현재 상태 평가 (Objective Function)
        # 통신은 둘 중 하나라도 끊기면 안 되므로, 더 낮은 신호를 기준으로 삼습니다.
        # 예: PC(-40), Cam(-80) -> 점수는 -80. 로봇은 Cam 쪽으로 이동해야 함.
        current_score = min(self.rssi_pc, self.rssi_cam)
        
        # 2. 목표 도달 확인 (신호가 충분히 좋으면 정지 - 배터리 절약 및 진동 방지)
        target_rssi = -45 # 목표 감도
        if current_score > target_rssi:
            self.get_logger().info(f"✅ 위치 양호 (Score: {current_score}). 대기 중...")
            return cmd # 정지 상태 반환

        # 3. 그라디언트 판단 (이전보다 좋아졌는가?)
        # 노이즈를 고려하여 변화량이 2dB 이상일 때만 유의미하게 판단
        diff = current_score - self.prev_score
        
        # 4. 행동 결정 로직 (Finite State Machine)
        
        # 상태 전환 주기: 매 0.1초마다 판단하면 로봇이 떨기만 하므로, 
        # 한 동작을 최소 10틱(1초) 정도는 유지하게 합니다.
        if self.state_timer > 0:
            self.state_timer -= 1
        else:
            # 행동 결정 시점 도달
            if self.action_state == 'FORWARD':
                if diff >= 0: 
                    # 상황이 좋아지거나 같음 -> 하던 대로 계속 직진
                    self.get_logger().info(f"👍 신호 개선중 ({diff:+.2f}dB). 직진.")
                    self.state_timer = 5 # 0.5초 더 직진
                else:
                    # 상황이 나빠짐 -> 후퇴 혹은 회전 필요
                    self.get_logger().info(f"👎 신호 악화 ({diff:+.2f}dB). 탐색 시작.")
                    self.action_state = 'TURN'
                    self.state_timer = 15 # 1.5초간 회전
                    
                    # 회전 방향 랜덤 결정 (또는 이전에 좋았던 방향)
                    import random
                    self.turn_direction = random.choice([0.5, -0.5])

            elif self.action_state == 'TURN':
                # 회전 후에는 다시 직진해봐야 함
                self.action_state = 'FORWARD'
                self.state_timer = 10 # 1초간 직진 시도
        
        # 5. 현재 상태 기록 업데이트
        self.prev_score = current_score

        # 6. 실제 속도 명령 생성
        if self.action_state == 'FORWARD':
            cmd.linear.x = 0.15 # 전진 속도
            cmd.angular.z = 0.0
        elif self.action_state == 'TURN':
            cmd.linear.x = 0.0
            cmd.angular.z = self.turn_direction # 회전 속도

        return cmd

    def control_loop(self):
        cmd = Twist()

        # [우선순위 1] 생존 본능: 장애물 회피
        if self.obstacle_detected:
            self.get_logger().info("🚧 장애물 감지! 회피 동작 중...")
            cmd.linear.x = 0.0
            cmd.angular.z = self.escape_direction # 계산된 방향으로 회전

        # [우선순위 2] 목표 수행: RSSI 추적
        else:
            # 장애물이 없을 때만 RSSI 알고리즘 실행
            cmd = self.get_rssi_command()
        
        self.cmd_pub.publish(cmd)
    
    def rssi_pc_callback(self, msg):
        self.rssi_pc = msg.data
        # 로그로 확인하고 싶으면 아래 주석 해제
        # self.get_logger().info(f'Received PC RSSI: {self.rssi_pc}')

    def rssi_cam_callback(self, msg):
        self.rssi_cam = msg.data
        self.get_logger().info(f'Received CAM RSSI: {self.rssi_cam}')

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveRelayBot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
