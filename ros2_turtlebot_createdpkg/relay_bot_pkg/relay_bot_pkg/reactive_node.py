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
        self.rssi_pc = self.create_subscription(Int32, 'rssi/pc', self.rssi_pc_callback, 10)
        self.rssi_cam = self.create_subscription(Int32, 'rssi/cam', self.rssi_cam_callback, 10)
        
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
        
        # 기본 주행 파라미터
        self.BASE_SPEED = 0.15
        self.MAX_SPEED = 0.22

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
        
        # 1. 현재 품질 점수 계산 (필터링된 RSSI 사용)
        current_score = self.calculate_quality()
        self.get_logger().info(f"RSSI 신호 (PC : {self.rssi_pc:.2f}, CAM : {self.rssi_cam:.2f})")
        
        # 2. 변화량(Gradient) 계산
        diff = current_score - self.prev_score
        
        # 3. 목표 도달 시 정지 (배터리 절약)
        if current_score > 90.0:
            self.get_logger().info(f"✅ 위치 최적 (Score: {current_score:.1f}). 대기.")
            return cmd

        # 4. 행동 결정 로직 (가변 속도 적용)
        if self.state_timer > 0:
            self.state_timer -= 1
        else:
            # --- 결정의 순간 ---
            if self.action_state == 'RECOVER' :
                # 신호가 급락했으므로, 방금 온 방향이 잘못됨 -> 후진 후 회전
                self.get_logger().info(f"🚫 급락 발생 ({diff:.2f}). 후진 및 탐색.")
                
                # 1단계: 잠깐 후진 (직전의 좋은 위치로)
                cmd.linear.x = cmd.linear.x * (-1.0)
                cmd.angular.z = 0.0
                self.state_timer = 8 # 0.8초 후진
                
                # 다음 상태 예약
                self.action_state = 'TURN'
            elif self.action_state == 'FORWARD':
                
                if diff > 0:
                    # [상황 A] 신호가 좋아짐 -> 가속 (Sprint)
                    # 변화량이 클수록 더 확신을 가짐
                    speed_boost = min(diff * 0.02, 0.07) # 최대 0.07m/s 추가
                    cmd.linear.x = self.BASE_SPEED + speed_boost
                    cmd.angular.z = 0.0
                    
                    self.get_logger().info(f"🚀 개선중 (+{diff:.2f}). 가속 직진.")
                    self.state_timer = 3 # 0.3초간 유지 (반응성 높임)

                elif diff > -2.0:
                    # [상황 B] 신호가 '약간' 나빠짐 -> 감속 및 커브 (Curve/Wander)
                    # 바로 회전하지 말고 부드럽게 궤적 수정
                    cmd.linear.x = self.BASE_SPEED * 0.6 # 속도 줄임
                    cmd.angular.z = random.choice([0.3, -0.3]) # 약간 비틀기
                    
                    self.get_logger().info(f"📉 약간 하락 ({diff:.2f}). 커브 주행.")
                    self.state_timer = 5 # 0.5초간 유지

                else:
                    # [상황 C] 신호가 '급격히' 나빠짐 -> 즉시 탐색 모드
                    self.action_state = 'RECOVER'
                    self.state_timer = 0 # 즉시 실행

            

            elif self.action_state == 'TURN':
                # 제자리 회전으로 새로운 방향 모색
                cmd.linear.x = 0.0
                cmd.angular.z = random.choice([0.6, -0.6]) # 회전
                self.state_timer = 10 # 1초 회전
                
                self.action_state = 'FORWARD' # 다음엔 직진 시도

        # 5. 상태 기록 업데이트
        self.prev_score = current_score
        
        # (타이머가 남아있을 때 실행할 명령 유지)
        if self.state_timer > 0 and self.action_state != 'RECOVER': 
             # RECOVER는 위에서 직접 할당했으므로 제외, 나머지는 유지
             pass 

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
            cmd = self.get_rssi_command()
            cmd.linear.x = cmd.linear.x * 1.5
        
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
