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
        """ 
        [수정됨] 장애물 회피 벡터 계산
        단순 감지가 아니라, '어느 쪽으로 얼마나 피해야 하는지' 계산
        """
        scan_ranges = msg.ranges
        # 무한대(inf)나 0.0을 10.0으로 치환하여 계산 오류 방지
        cleaned_ranges = [r if (r > 0.01 and r < 10.0) else 10.0 for r in scan_ranges]
        
        # 1. 전방 거리 (충돌 방지용 급제동)
        # 전방 60도(-30 ~ +30)의 최소거리
        front_ranges = cleaned_ranges[-30:] + cleaned_ranges[:30]
        self.front_min_dist = min(front_ranges)
        
        # 2. 좌우 척력(Repulsive Force) 계산
        # 왼쪽이 가까우면 오른쪽으로 회전력 발생 (음수), 오른쪽이 가까우면 왼쪽으로 (양수)
        
        # 좌측 90도 영역 (0 ~ 90) / 우측 90도 영역 (270 ~ 360)
        left_ranges = cleaned_ranges[0:90]
        right_ranges = cleaned_ranges[270:360]
        
        # 거리가 가까울수록 가중치를 높임 (1/distance)
        # 평균 거리를 사용하여 노이즈를 줄임
        avg_left = sum(left_ranges) / len(left_ranges)
        avg_right = sum(right_ranges) / len(right_ranges)
        
        # [회피 벡터] 
        # 왼쪽이 가까우면(작으면) -> 값 커짐 -> 오른쪽으로 가야 함(Minus Turn)
        # 오른쪽이 가까우면(작으면) -> 값 커짐 -> 왼쪽으로 가야 함(Plus Turn)
        
        force_left = 1.0 / (avg_left + 0.1)  # 0.1은 분모 0 방지
        force_right = 1.0 / (avg_right + 0.1)
        
        # 회피 가중치 (Gain): 장애물에 얼마나 민감하게 반응할지
        AVOID_GAIN = 1.5
        self.avoid_angular_z = (force_right - force_left) * AVOID_GAIN

    def calculate_quality(self):
        """ 
        [수정됨] 목줄(Leash) 기능 추가
        PC 신호가 끊어질 위험이 있으면 점수를 급격히 낮춰 복귀를 강제함
        """
        def to_score(rssi):
            if rssi >= -30: return 100.0
            if rssi <= -70: return 0.0
            return (rssi + 70) * 2.5 

        s_pc = to_score(self.rssi_pc)
        s_cam = to_score(self.rssi_cam)
        
        self.current_s_pc = s_pc 
        
        # ---------------------------------------------------------
        # 1. 목줄 (The Leash) - 안전장치
        # ---------------------------------------------------------
        # PC RSSI가 -65dBm 보다 낮아지면(더 나빠지면) 비상 상황으로 간주
        # 끊어지기 직전(-70dBm)보다 약간 여유를 둠 (-65dBm)
        SAFE_THRESHOLD = -65.0 
        
        if self.rssi_pc < SAFE_THRESHOLD:
            # 점수를 음수로 만들어버림 -> 이전 점수(양수)와의 차이(Diff)가 
            # 거대한 마이너스 값이 됨 -> 로봇은 '급락(Sudden Drop)'으로 인식하고 반전(Invert) 수행
            return -100.0 

        # ---------------------------------------------------------
        # 2. 안전 구역 내에서의 행동 (PC <-> CAM 균형 탐색)
        # ---------------------------------------------------------
        # PC 신호가 안전하다면, CAM 신호를 찾아 멀리 나가는 것을 허용
        # min() 함수를 사용하여 두 신호의 균형점을 찾음
        final_score = min(s_pc, s_cam)
        
        # 약간의 탐색 동력
        final_score = final_score * 0.9 + (s_pc + s_cam) * 0.05
        
        return final_score
    
    def get_rssi_command(self):
        cmd = Twist()
        
        # 1. 점수 계산 및 델타 확인
        current_score = self.calculate_quality()
        diff = current_score - self.prev_score
        
        # [목표 도달] 점수가 충분히 높으면 정지 (사용자 요청 반영)
        if current_score > 55.0:
            self.get_logger().info(f"✅ 목표 신호 도달! (점수: {current_score:.1f})")
            self.last_cmd = Twist()
            return cmd

        # [상태 유지 타이머] 회전이나 후진 동작을 일정 시간 수행하기 위함
        if self.state_timer > 0:
            self.state_timer -= 1
            cmd = self.last_cmd
            # 상태가 바뀌는 순간에 prev_score 갱신을 막기 위해 여기서 리턴
            # (단, 회전 중에는 RSSI 변화가 없으므로 점수 로직에 영향 없음)
            return cmd 

        # -----------------------------------------------------------
        # [A] 탐색 로직 (State Machine)
        # -----------------------------------------------------------
        
        # 기본 가정: 이전 동작이 '전진'이었다고 가정하고 평가
        # 노이즈를 고려하여 임계값(threshold) 설정 (0.0이 아니라 약간의 마진)
        NOISE_MARGIN = 0.5 

        # 상황 1: 신호가 좋아지거나 유지됨 (Keep Going)
        # 점수가 낮더라도 상승세라면 절대 멈추지 않음이 핵심
        if diff >= -NOISE_MARGIN:
            self.action_state = 'FORWARD_TRACKING'
            cmd.linear.x = self.BASE_SPEED
            
            # 방향 미세 조정 (Gradient Ascent 가속)
            # 신호가 확 좋아지면 속도를 조금 더 냄
            if diff > 1.0:
                cmd.linear.x = self.MAX_SPEED
            
            # 아주 약간의 조향을 섞어서 완만한 곡선을 그리며 탐색 (직선 고착 방지)
            # 현재 스캔 방향으로 미세 회전
            cmd.angular.z = 0.05 * self.scan_direction 

            # 로그 간소화 (너무 자주 뜨지 않게)
            if self.current_s_pc < 45.0 and diff > 0.5:
                self.get_logger().info(f"✨ 신호 추적 중... (점수 {current_score:.1f} / 변화 +{diff:.2f})")

        # 상황 2: 신호가 나빠짐 (Wrong Direction)
        # 전진했더니 점수가 떨어짐 -> 이 방향 아님 -> 후진 후 방향 전환
        else:
            self.get_logger().warn(f"📉 방향 이탈 (변화 {diff:.2f}). 재설정 시도.")
            
            # 2-1. 위치 복구 (Recovery)
            # 잘못 간 만큼 살짝 뒤로 물러남 (신호가 좋았던 위치로 복귀)
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0
            
            # 2-2. 다음 틱을 위해 회전 준비 (상태 타이머 설정)
            # 이 함수가 다음 호출될 때(0.1초 뒤)가 아니라, 
            # 지금 결정을 내려서 타이머를 걸어야 함.
            
            # 논리: 이번 틱은 '후진' 명령을 내리고, 
            # 다음 몇 틱 동안은 '회전'을 하도록 유도해야 함.
            # 하지만 단순화를 위해 여기서는 "일단 후진"만 수행하고
            # 후진이 끝난 직후(다음 로직 진입 시) 회전하도록 플래그를 쓰거나,
            # 랜덤 회전 + 후진을 섞음.
            
            # [수정된 전략] 제자리 회전은 RSSI 변화가 없으므로 
            # "후진하면서 회전"하여 위치와 각도를 동시에 바꿈
            turn_direction = random.choice([1.0, -1.0])
            self.scan_direction = turn_direction # 다음 탐색 방향 결정
            
            cmd.linear.x = -0.10 # 천천히 후진
            cmd.angular.z = 0.8 * turn_direction # 강하게 회전
            
            self.state_timer = 5 # 0.5초 동안 이 동작 수행 (후진+회전)
            self.action_state = 'RECOVER_TURN'
            
            # 회전 후에는 이전 점수와의 비교가 무의미해질 수 있으므로
            # 현재 점수를 기준점으로 재설정하는 효과

        self.last_cmd = cmd
        self.prev_score = current_score
        return cmd
def control_loop(self):
        """
        [수정됨] 벡터 합성 제어 (Vector Fusion)
        최종 명령 = (RSSI 추적 벡터) + (장애물 회피 벡터)
        """
        cmd = Twist()
        
        # 1. 글로벌 플래너 (RSSI 추적 명령)
        rssi_cmd = self.get_rssi_command()
        
        # 2. 벡터 합성 (Vector Addition)
        # RSSI가 가고 싶은 회전 방향 + 장애물을 피해야 하는 회전 방향
        final_angular_z = rssi_cmd.angular.z + self.avoid_angular_z
        
        # 3. 선속도 제어 (Smart Velocity)
        # 장애물이 아주 가까우면 속도를 줄이고, 멀면 RSSI 명령을 따름
        if self.front_min_dist < 0.4:
            # [위험] 전방이 막힘 -> 제자리 회전 혹은 후진으로 전환
            # RSSI고 뭐고 일단 살아야 함
            final_linear_x = -0.05 
            # 막혔을 때는 회피 벡터를 더 강하게 반영
            final_angular_z = self.avoid_angular_z * 2.0 
            if abs(final_angular_z) < 0.1: # 정면 벽이면 강제로 틂
                 final_angular_z = 1.0 
                 
        elif self.front_min_dist < 0.7:
            # [경고] 장애물 접근 중 -> 속도 줄이며 부드럽게 회피
            final_linear_x = rssi_cmd.linear.x * 0.3
        else:
            # [안전] RSSI 명령대로 주행
            final_linear_x = rssi_cmd.linear.x

        # 4. 값 적용 및 제한 (Saturation)
        cmd.linear.x = max(min(final_linear_x, self.MAX_SPEED), -self.MAX_SPEED)
        cmd.angular.z = max(min(final_angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)

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
