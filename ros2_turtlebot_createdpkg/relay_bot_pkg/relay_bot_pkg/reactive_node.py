import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
import random

# ---------------------------------------------------------
# 1. 1D 칼만 필터 클래스 (수정됨)
# ---------------------------------------------------------
class SimpleKalmanFilter:
    # 변수명을 호출할 때 쓰는 Q, R, P, initial_value와 똑같이 맞췄습니다.
    def __init__(self, Q, R, P, initial_value):
        self.Q = Q              # 프로세스 노이즈 (Process Noise)
        self.R = R              # 측정 노이즈 (Measurement Noise)
        self.P = P              # 추정 오차 (Estimation Error)
        self.X = initial_value  # 초기값 (Initial Value)

    def update(self, measurement):
        # 1. Prediction Update
        self.P = self.P + self.Q

        # 2. Measurement Update
        K = self.P / (self.P + self.R)      # 칼만 이득 계산
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        
        return self.X

# ---------------------------------------------------------
# 2. 메인 로봇 제어 노드
# ---------------------------------------------------------
class ReactiveRelayBot(Node):
    def __init__(self):
        super().__init__('reactive_relay_bot')
        
        # QoS 설정
        qos_policy = QoSProfile(depth=10)
        
        # Subscriber & Publisher
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_policy)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # RSSI 구독객체와 값 변수 분리
        self.rssi_pc_sub = self.create_subscription(Int32, 'rssi/pc', self.rssi_pc_callback, 10)
        self.rssi_cam_sub = self.create_subscription(Int32, 'rssi/cam', self.rssi_cam_callback, 10)
        
        self.rssi_pc = -60.0
        self.rssi_cam = -60.0
        
        # 타이머 (0.1초 주기)
        self.timer = self.create_timer(0.1, self.control_loop)

        # [필터 설정] Wifi 신호는 노이즈가 심하므로 R값을 높게 설정
        # Q=0.1: 로봇이 움직이며 신호가 서서히 변함
        # R=10.0: 측정값이 많이 튐
        self.kf_pc = SimpleKalmanFilter(Q=0.1, R=10.0, P=1.0, initial_value=-60.0)
        self.kf_cam = SimpleKalmanFilter(Q=0.1, R=10.0, P=1.0, initial_value=-60.0)

        # 상태 변수
        self.obstacle_detected = False
        self.escape_direction = 0.0 
        

        self.prev_score = 0.0     # 이전 점수 (0~100점)
        self.action_state = 'FORWARD' 
        self.state_timer = 0      # 상태 유지 타이머
        self.turn_direction = 1.0 # 1.0(좌), -1.0(우)
        self.last_cmd = Twist()   # 이전 명령 저장용
        
        # 기본 주행 파라미터
        self.BASE_SPEED = 0.1
        self.MAX_SPEED = 0.22
        self.MAX_ANGULAR_SPEED = 1.0 # 최대 회전 속도 추가
        self.scan_direction = 1.0 
        self.is_moving_forward = True # [추가] 현재 이동 방향 기억 (True: 전진, False: 후진)

    def scan_callback(self, msg):
        """ 장애물 감지 로직 (기존 유지) """
        scan_ranges = msg.ranges
        cleaned_ranges = [r if r > 0.0 else 10.0 for r in scan_ranges]
        
        front_ranges = cleaned_ranges[-30:] + cleaned_ranges[:30]
        min_front_dist = min(front_ranges)
        left_dist = min(cleaned_ranges[30:90])
        right_dist = min(cleaned_ranges[270:330])

        collision_threshold = 0.35
        
        if min_front_dist < collision_threshold:
            self.obstacle_detected = True
            if left_dist > right_dist:
                self.escape_direction = 0.5 
            else:
                self.escape_direction = -0.5
        else:
            self.obstacle_detected = False

    def calculate_quality(self):
        """ 점수 계산은 물리적 신호 변화만 반영하도록 단순화 """
        def to_score(rssi):
            if rssi >= -30: return 100.0
            if rssi <= -70: return 0.0
            return (rssi + 70) * 2.5 

        s_pc = to_score(self.rssi_pc)
        s_cam = to_score(self.rssi_cam)
        
        # 멤버 변수로 저장해두어 get_rssi_command에서 개별 값을 확인할 수 있게 함
        self.current_s_pc = s_pc 
        
        # [수정] 페널티 로직 제거 -> 항상 연속적인 값 반환
        # PC 점수 비중을 높게(0.7) 유지하여 PC 변화에 민감하게 반응
        return (s_pc * 0.7) + (s_cam * 0.3)
    
    def get_rssi_command(self):
        cmd = Twist()
        
        # 1. 계산 (이제 급격한 점수 널뛰기가 없음)
        current_score = self.calculate_quality()
        diff = current_score - self.prev_score
        
        # 2. 목표 도달 정지
        if current_score > 90.0:
            self.last_cmd = Twist()
            return cmd

        # 3. 타이머 유지
        if self.state_timer > 0:
            self.state_timer -= 1
            cmd = self.last_cmd
            return cmd 

        # -----------------------------------------------------------
        # [A] 급락 감지 (Sudden Drop) -> 행동 반전
        # -----------------------------------------------------------
        if diff < -2.0:
            self.get_logger().warn(f"📉 신호 급락! ({diff:.2f}) 행동 반전.")
            if self.is_moving_forward:
                self.is_moving_forward = False 
                cmd.linear.x = -0.2
                self.action_state = 'INVERT_BACK'
            else:
                self.is_moving_forward = True
                cmd.linear.x = 0.25
                self.action_state = 'INVERT_FWD'
            
            cmd.angular.z = 0.0
            self.state_timer = 5
            
        # -----------------------------------------------------------
        # [B] 생존 모드 (PC 신호 위험) -> 스마트 후진
        # -----------------------------------------------------------
        # [핵심] 통합 점수가 아니라 'PC 개별 점수'를 기준으로 판단
        elif self.current_s_pc < 45.0:
            
            # B-1. 이미 후진 중인데 상황이 안 좋아짐 (Death Spiral 방지)
            if not self.is_moving_forward and diff < -0.2:
                self.get_logger().warn(f"🚫 후진 실패 (Diff {diff:.2f}). 비상 전진 전환!")
                self.is_moving_forward = True # 전진으로 강제 전환
                cmd.linear.x = 0.25 # 탈출 속도
                cmd.angular.z = 0.0
                self.action_state = 'ESCAPE_FWD'
                self.state_timer = 8 # 길게 전진
            
            # B-2. 일반적인 위험 상황 -> 후진 시도
            else:
                self.get_logger().info(f"🚫 PC 신호 위험 ({self.current_s_pc:.1f}). 후진.")
                self.is_moving_forward = False
                cmd.linear.x = -0.15
                cmd.angular.z = random.choice([0.1, -0.1]) # 직선에 가깝게 후진
                self.action_state = 'BACKWARD'
                self.state_timer = 3

        # -----------------------------------------------------------
        # [C] 아크 탐색 및 주행 (Forward)
        # -----------------------------------------------------------
        else:
            self.is_moving_forward = True 
            self.action_state = 'FORWARD'

            if diff > 0.2:
                # 신호 좋음
                cmd.linear.x = self.BASE_SPEED + 0.05
                cmd.angular.z = 0.3 * self.scan_direction
                self.state_timer = 3

            elif diff < -0.2:
                # 방향 틀림 -> 아크 반전
                self.scan_direction *= -1.0 
                cmd.linear.x = 0.05 
                cmd.angular.z = 0.8 * self.scan_direction
                self.state_timer = 3

            else:
                # 탐색 지속
                cmd.linear.x = self.BASE_SPEED
                cmd.angular.z = 0.5 * self.scan_direction
                self.state_timer = 2

        self.last_cmd = cmd
        self.prev_score = current_score
        return cmd

    def control_loop(self):
        cmd = Twist()

        # [우선순위 1] 장애물 회피
        if self.obstacle_detected:
            self.get_logger().info("🚧 장애물 회피 중")
            cmd.linear.x = 0.0
            cmd.angular.z = self.escape_direction
            # 회피 중에는 이전 점수 리셋 (회피 후 엉뚱한 판단 방지)
            self.prev_score = self.calculate_quality() 

        # [우선순위 2] RSSI 추적
        else:
            rssi_cmd = self.get_rssi_command()
            cmd.linear.x = rssi_cmd.linear.x 
            cmd.angular.z = rssi_cmd.angular.z
        
        # [안전 장치] 속도 제한 적용
        cmd.linear.x = max(min(cmd.linear.x, self.MAX_SPEED), -self.MAX_SPEED)
        cmd.angular.z = max(min(cmd.angular.z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)

        self.cmd_pub.publish(cmd)
    
    def rssi_pc_callback(self, msg):
        # 칼만 필터로 업데이트
        self.rssi_pc = self.kf_pc.update(float(msg.data))

    def rssi_cam_callback(self, msg):
        # 칼만 필터로 업데이트
        self.rssi_cam = self.kf_cam.update(float(msg.data))

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
