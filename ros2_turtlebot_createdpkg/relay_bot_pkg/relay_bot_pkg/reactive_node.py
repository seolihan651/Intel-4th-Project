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
        
        # 타이머 (0.5초 주기)
        self.timer = self.create_timer(0.5, self.control_loop)

        # [필터 설정] Wifi 신호는 노이즈가 심하므로 R값을 높게 설정
        # Q=0.1: 로봇이 움직이며 신호가 서서히 변함
        # R=10.0: 측정값이 많이 튐
        self.kf_pc = SimpleKalmanFilter(Q=0.1, R=6.0, P=1.0, initial_value=-60.0)
        self.kf_cam = SimpleKalmanFilter(Q=0.1, R=6.0, P=1.0, initial_value=-60.0)

        # 상태 변수
        self.obstacle_detected = False
        self.escape_direction = 0.0 

        self.avoid_angular_z = 0.0  # 회피 벡터 초기값 (0 = 회전 안 함)
        self.front_min_dist = 10.0  # 전방 거리 초기값 (10m = 안전함)
        

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
        
        # 1. 점수 계산
        current_score = self.calculate_quality()
        diff = current_score - self.prev_score
        
        # [목표 도달]
        if current_score > 85.0:
            self.last_cmd = Twist()
            return cmd

        # -----------------------------------------------------------
        # [상태 머신] Pivot & Probe 시퀀스 제어
        # -----------------------------------------------------------
        if self.state_timer > 0:
            self.state_timer -= 1
            
            # [단계 1] 제자리 회전 (Pivot Turn)
            # 지형지물에 걸리지 않도록 선속도를 0으로 하고 회전만 수행
            if self.action_state == 'PIVOT_TURN':
                cmd.linear.x = 0.0
                cmd.angular.z = self.stored_turn_speed # 저장된 방향으로 회전
                
                # 회전 타이머가 끝나면 -> 즉시 '직진 탐색' 모드로 전환 (연계 동작)
                if self.state_timer == 0:
                    self.action_state = 'STRAIGHT_PROBE'
                    self.state_timer = 15  # 1.5초 동안 직진 예약
                    self.get_logger().info("🚀 각도 변경 완료. 직진 탐색(Probe) 시작.")
                
                return cmd

            # [단계 2] 직선 탐색 (Straight Probe)
            # 변경된 각도로 일정 거리를 이동해봐야 신호 변화를 알 수 있음
            elif self.action_state == 'STRAIGHT_PROBE':
                # 안전장치: 탐색 중이라도 신호가 급락하면 즉시 중단
                if diff < -5.0:
                    self.get_logger().warn(f"🚫 탐색 중 급락! ({diff:.2f}) 중단.")
                    self.state_timer = 0
                    return self.last_cmd
                
                cmd.linear.x = 0.20  # 과감하게 전진
                cmd.angular.z = 0.0  # 회전 없이 직진만
                self.last_cmd = cmd
                return cmd
                
            # 그 외 상태 (후진 등)
            else:
                return self.last_cmd

        # -----------------------------------------------------------
        # [A] 판단 로직 (Evaluation)
        # -----------------------------------------------------------
        
        NOISE_MARGIN = 0.5 

        # 상황 1: 신호가 좋아지거나 유지됨 (Keep Going)
        if diff >= -NOISE_MARGIN:
            self.action_state = 'FORWARD_TRACKING'
            
            # 기본 전진
            cmd.linear.x = self.BASE_SPEED
            
            # 아주 미세한 조향만 허용 (지형 극복용)
            cmd.angular.z = 0.05 * self.scan_direction 

            if diff > 1.0: # 신호가 확 좋아지면 속도 증가
                cmd.linear.x = self.MAX_SPEED

        # 상황 2: 신호가 나빠짐 -> 전략 수정
        else:
            self.get_logger().warn(f"📉 방향 이탈 (변화 {diff:.2f}). 각도 재설정.")
            
            # -------------------------------------------------------
            # [수정됨] Pivot & Probe 전략 진입
            # -------------------------------------------------------
            
            # 1. 일단 멈춤 (운동량 제거)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            # 2. 새로운 회전 방향 결정
            # - 랜덤으로 90도 정도 확 꺾는 것이 로컬 미니마 탈출에 좋음
            # - 0.8 rad/s * 1.0 sec = 약 45~50도 회전
            turn_dir = random.choice([1.0, -1.0])
            self.stored_turn_speed = 1.0 * turn_dir # 회전 속도 저장
            
            # 3. 상태 설정: 'PIVOT_TURN'으로 진입
            self.action_state = 'PIVOT_TURN'
            self.state_timer = 8  # 0.8초 동안 제자리 회전
            
            # (타이머가 끝나면 자동으로 STRAIGHT_PROBE로 넘어감)

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
