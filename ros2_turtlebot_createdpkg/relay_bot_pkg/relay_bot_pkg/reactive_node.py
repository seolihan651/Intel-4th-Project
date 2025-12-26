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
        self.BASE_SPEED = 0.15
        self.MAX_SPEED = 0.22
        self.scan_direction = 1.0

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
        """ 
        [PC 우선순위 강화 버전] 
        PC 신호가 생존선(Safety Line)을 넘지 못하면 카메라는 쳐다보지도 않음
        """
        def to_score(rssi):
            if rssi >= -30: return 100.0
            if rssi <= -70: return 0.0
            return (rssi + 70) * 2.5 

        s_pc = to_score(self.rssi_pc)
        s_cam = to_score(self.rssi_cam)

        # -------------------------------------------------------------
        # 1. [최우선] PC 생존선 검사 (Survival Mode)
        # -------------------------------------------------------------
        # PC 점수가 45점(약 -72dBm) 미만이면 '비상 복귀' 모드
        if s_pc < 45.0:
            # s_cam 값은 완전히 무시합니다.
            # 0.5를 곱하는 이유: 점수를 의도적으로 낮게 만들어(최대 22.5점), 
            # 로봇이 "지금 상태가 매우 나쁘다"고 느끼게 하여 개선 의지를 높임
            return s_pc * 0.5 

        # -------------------------------------------------------------
        # 2. [차순위] 카메라 신호 관리 (Service Mode)
        # -------------------------------------------------------------
        # PC는 안전하므로(45점 이상), 이제 카메라 신호가 약한지 봅니다.
        if s_cam < 40.0:
            # PC는 괜찮은데 카메라가 끊길 것 같으면, 카메라 쪽으로 이동
            return s_cam * 0.8 
        
        # -------------------------------------------------------------
        # 3. [안전 구역] 위치 최적화 (Safe Zone)
        # -------------------------------------------------------------
        # 둘 다 신호가 충분한 경우입니다.
        # 여기서 PC 쪽에 가중치를 더 주면(0.7), 로봇이 PC 쪽에 더 가깝게 머뭅니다.
        # (PC: 70%, CAM: 30% 비중)
        return (s_pc * 0.7) + (s_cam * 0.3)
    def get_rssi_command(self):
        cmd = Twist()
        
        # 1. RSSI 점수 및 변화량 계산
        current_score = self.calculate_quality()
        diff = current_score - self.prev_score
        
        # 2. 목표 도달 시 정지
        if current_score > 90.0:
            self.last_cmd = Twist()
            return cmd

        # 3. 상태 유지 타이머 처리
        if self.state_timer > 0:
            self.state_timer -= 1
            cmd = self.last_cmd
        
        else:
            # -----------------------------------------------------------
            # [A] 비상 후진 (Emergency) - 점수가 너무 낮을 때
            # -----------------------------------------------------------
            if current_score < 40.0:
                self.get_logger().warn(f"🚫 비상! 점수 저조 ({current_score:.1f}). 후진.")
                cmd.linear.x = -0.15 # 확실한 후진
                cmd.angular.z = 0.0
                self.state_timer = 5
                self.action_state = 'SEARCH' # 후진 후 탐색 모드로

            # -----------------------------------------------------------
            # [B] 아크 탐색 (Arc Search) - 신호가 애매하거나 하락세일 때
            # -----------------------------------------------------------
            # 반원을 그리며(이동하며) 신호 변화를 측정합니다.
            elif self.action_state == 'SEARCH' or current_score < 55.0:
                
                # B-1. 신호가 확실히 좋아짐 (찾았다!)
                if diff > 0.5: 
                    self.get_logger().info(f"✨ 경로 발견! ({self.scan_direction} 방향)")
                    # 가속하며 해당 방향으로 주행 전환
                    cmd.linear.x = self.BASE_SPEED
                    cmd.angular.z = 0.3 * self.scan_direction 
                    self.action_state = 'FORWARD'
                    self.state_timer = 5

                # B-2. 신호가 계속 나빠짐 (여기가 아닌가봐)
                elif diff < -0.2:
                    self.get_logger().info("↩️ 방향 전환 (Arc Flip)")
                    self.scan_direction *= -1.0 # 반대 방향으로 아크 뒤집기
                    
                    # 방향을 바꿀 때는 제자리에서 살짝 돌려줌 (즉각 반응)
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.8 * self.scan_direction
                    self.state_timer = 2
                
                # B-3. 탐색 진행 (천천히 움직이며 데이터 수집)
                else:
                    self.get_logger().info(f"📡 아크 탐색 중... ({current_score:.1f})")
                    # [핵심] 위치를 바꾸기 위해 전진 성분을 섞음
                    cmd.linear.x = 0.08  # 천천히 전진
                    cmd.angular.z = 0.6 * self.scan_direction # 강하게 회전
                    self.state_timer = 2 # 짧게 끊어서 자주 판단

            # -----------------------------------------------------------
            # [C] 일반 주행 (FORWARD)
            # -----------------------------------------------------------
            else: # action_state == 'FORWARD' (점수 양호)
                self.action_state = 'FORWARD'
                
                if diff > 0:
                    # 신호 좋음: 속도 높여서 직진
                    cmd.linear.x = self.BASE_SPEED + 0.05
                    cmd.angular.z = 0.0
                    self.state_timer = 3
                
                elif diff > -1.5:
                    # 신호 유지: 완만한 커브로 넓게 이동
                    cmd.linear.x = self.BASE_SPEED
                    cmd.angular.z = random.choice([0.2, -0.2])
                    self.state_timer = 5
                
                else:
                    # 신호 급락: 즉시 탐색 모드 전환
                    self.get_logger().info("📉 신호 유실 감지. 아크 탐색 시작.")
                    self.action_state = 'SEARCH'
                    self.state_timer = 0

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
            cmd.linear.x = rssi_cmd.linear.x * 1.5
            cmd.angular.z = rssi_cmd.angular.z
        
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
