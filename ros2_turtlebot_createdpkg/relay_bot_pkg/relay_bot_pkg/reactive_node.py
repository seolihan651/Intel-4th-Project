import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
import random

# ---------------------------------------------------------
# 1. 1D 칼만 필터 클래스 (초기값 자동 설정 기능 추가)
# ---------------------------------------------------------
class SimpleKalmanFilter:
    def __init__(self, Q, R, P, initial_value=None):
        self.Q = Q              # 프로세스 노이즈
        self.R = R              # 측정 노이즈
        self.P = P              # 추정 오차
        self.X = initial_value  # 초기값 (None이면 첫 데이터로 설정)

    def update(self, measurement):
        # [수정] 첫 데이터가 들어오면 그것을 초기값으로 설정 (초기 급락 방지)
        if self.X is None:
            self.X = measurement
            return self.X

        # 1. Prediction Update
        self.P = self.P + self.Q

        # 2. Measurement Update
        K = self.P / (self.P + self.R)      
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        
        return self.X

# ---------------------------------------------------------
# 2. 메인 로봇 제어 노드 (TQ 기반 변수명 수정됨)
# ---------------------------------------------------------
class ReactiveRelayBot(Node):
    def __init__(self):
        super().__init__('reactive_relay_bot')
        
        # QoS 설정: 라이다(LDS)와 호환되도록 BEST_EFFORT 사용
        qos_policy = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        # Subscriber & Publisher
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_policy)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # [수정] 변수명 RSSI -> TQ 변경
        self.tq_pc_sub = self.create_subscription(Int32, '/tq/pc', self.tq_pc_callback, 10)
        self.tq_cam_sub = self.create_subscription(Int32, '/tq/cam', self.tq_cam_callback, 10)
        
        # [수정] 초기값을 None으로 설정하여 첫 데이터 수신 시 초기화
        self.tq_pc = None
        self.tq_cam = None
        
        # 타이머 (0.5초 주기)
        self.timer = self.create_timer(0.5, self.control_loop)

        # [필터 설정] TQ값(0~255) 대응
        # initial_value=None으로 설정하여 첫 수신값에 맞춤
        self.kf_pc = SimpleKalmanFilter(Q=0.5, R=10.0, P=5.0, initial_value=None)
        self.kf_cam = SimpleKalmanFilter(Q=0.5, R=10.0, P=5.0, initial_value=None)

        # 상태 변수
        self.obstacle_detected = False
        
        self.avoid_angular_z = 0.0  
        self.front_min_dist = 10.0  
        
        self.prev_score = 0.0     
        self.action_state = 'FORWARD' 
        self.state_timer = 0      
        self.stored_turn_speed = 0.0
        self.last_cmd = Twist()   
        
        # 기본 주행 파라미터
        self.BASE_SPEED = 0.1
        self.MAX_SPEED = 0.22
        self.MAX_ANGULAR_SPEED = 1.0 
        self.scan_direction = 1.0 

    def scan_callback(self, msg):
        """ 장애물 회피 로직 (Division by Zero 방지 포함) """
        scan_ranges = msg.ranges
        if not scan_ranges: return
            
        cleaned_ranges = [r if (r > 0.01 and r < 10.0) else 10.0 for r in scan_ranges]
        num_ranges = len(cleaned_ranges)
        
        # 1. 전방 거리
        if num_ranges > 60:
            front_ranges = cleaned_ranges[-30:] + cleaned_ranges[:30]
        else:
            front_ranges = cleaned_ranges
        
        self.front_min_dist = min(front_ranges) if front_ranges else 10.0
        
        # 2. 좌우 척력 계산
        quarter = num_ranges // 4
        if quarter == 0: return

        left_ranges = cleaned_ranges[0:quarter] 
        right_ranges = cleaned_ranges[-quarter:]
        
        avg_left = sum(left_ranges) / len(left_ranges) if left_ranges else 10.0
        avg_right = sum(right_ranges) / len(right_ranges) if right_ranges else 10.0
        
        force_left = 1.0 / (avg_left + 0.1)
        force_right = 1.0 / (avg_right + 0.1)
        
        AVOID_GAIN = 1.5
        self.avoid_angular_z = (force_right - force_left) * AVOID_GAIN

    def calculate_quality(self):
        """ 
        [수정] batman-adv TQ 기반 품질 계산 (변수명 tq로 변경)
        TQ: 0 (Bad) ~ 255 (Perfect)
        """
        # 아직 데이터가 안 들어왔으면 0점 처리 (출발 방지)
        if self.tq_pc is None or self.tq_cam is None:
            return 0.0

        def to_score(tq_val):
            if tq_val >= 220: return 100.0
            if tq_val <= 80: return 0.0
            # 80~220 구간을 0~100점으로 변환
            score = (tq_val - 80.0) / 140.0 * 100.0
            return score

        s_pc = to_score(self.tq_pc)
        s_cam = to_score(self.tq_cam)
        
        # ---------------------------------------------------------
        # 1. 목줄 (The Leash) - TQ 버전
        # ---------------------------------------------------------
        # [수정] 너무 쉽게 멈추지 않도록 임계값 120 -> 90으로 완화
        SAFE_THRESHOLD_TQ = 90.0 
        
        if self.tq_pc < SAFE_THRESHOLD_TQ:
            # 위험 상황: 복귀 강제 (강한 음수 점수 반환)
            return -100.0 

        # ---------------------------------------------------------
        # 2. 점수 계산 (PC와 CAM의 균형)
        # ---------------------------------------------------------
        final_score = min(s_pc, s_cam)
        final_score = final_score * 0.9 + (s_pc + s_cam) * 0.05
        
        return final_score
    

    def get_tq_command(self):
        cmd = Twist()
        
        # 데이터 없으면 대기
        if self.tq_pc is None: # or self.tq_cam is None (조건 완화 여부에 따라)
            return cmd

        current_score = self.calculate_quality()
        diff = current_score - self.prev_score
        
        # [수정] 목표 도달(>95점) 시에도 점수는 업데이트하고 종료해야 함
        if current_score > 95.0:
            self.last_cmd = Twist()
            self.prev_score = current_score  # <--- [중요] 이 줄을 꼭 추가해주세요!
            return cmd

        # -----------------------------------------------------------
        # [상태 머신] Pivot & Probe
        # -----------------------------------------------------------
        if self.state_timer > 0:
            self.state_timer -= 1
            
            # [단계 1] 제자리 회전 (Pivot Turn)
            if self.action_state == 'PIVOT_TURN':
                cmd.linear.x = 0.0
                cmd.angular.z = self.stored_turn_speed
                
                # 회전 끝 -> 직진 탐색 시작
                if self.state_timer == 0:
                    self.action_state = 'STRAIGHT_PROBE'
                    self.state_timer = 15  # 1.5초 직진 예약
                    self.get_logger().info("🚀 방향 전환 완료 -> 직진 탐색(Probe) 시작")
                return cmd

            # [단계 2] 직선 탐색 (Straight Probe)
            elif self.action_state == 'STRAIGHT_PROBE':
                # [수정] 탐색 중 신호 급락 시 -> 멈추지 말고 즉시 재탐색(PIVOT) 시도
                if diff < -5.0:
                    self.get_logger().warn(f"🚫 탐색 실패(TQ 급락 {diff:.2f}). 다시 회전합니다.")
                    self.state_timer = 0 # 현재 상태 종료
                    # 재귀적으로 다시 판단 로직으로 넘기거나, 여기서 바로 회전 명령을 줄 수 있음
                    # 여기서는 루프를 끝내고 다음 주기(0.5초 뒤)에 else문(상황 2)으로 빠지게 유도
                    return self.last_cmd 
                
                cmd.linear.x = 0.20
                cmd.angular.z = 0.0
                self.last_cmd = cmd
                return cmd
            else:
                return self.last_cmd

        # -----------------------------------------------------------
        # [A] 판단 로직 (Evaluation)
        # -----------------------------------------------------------
        NOISE_MARGIN = 0.5 

        # 상황 1: TQ가 유지되거나 좋아짐 (Go)
        if diff >= -NOISE_MARGIN:
            self.action_state = 'FORWARD_TRACKING'
            cmd.linear.x = self.BASE_SPEED
            cmd.angular.z = 0.05 * self.scan_direction 

            if diff > 1.0: 
                cmd.linear.x = self.MAX_SPEED

        # 상황 2: TQ 나빠짐 (Turn)
        else:
            self.get_logger().warn(f"📉 TQ 하락 (변화 {diff:.2f}). 회전 시도.")
            
            # 일단 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            # 랜덤 회전 방향 설정
            turn_dir = random.choice([1.0, -1.0])
            self.stored_turn_speed = 1.0 * turn_dir 
            
            # 상태 변경 -> PIVOT_TURN
            self.action_state = 'PIVOT_TURN'
            self.state_timer = 8  # 4초 회전 (타이머 주기가 0.5초라면 8틱 = 4초)
            # 8틱이 0.5초 주기면 4초가 맞습니다. (너무 길면 줄이세요)

        self.last_cmd = cmd
        self.prev_score = current_score
        return cmd
    
    
    def control_loop(self):
        """ 벡터 합성 제어 """
        cmd = Twist()
        # [수정] 함수 이름 변경 반영
        tq_cmd = self.get_tq_command()
        
        final_angular_z = tq_cmd.angular.z + self.avoid_angular_z
        self.get_logger().info(f"거리: {self.front_min_dist:.2f}m | TQ점수: {self.prev_score:.1f}")
        
        # 장애물 회피 우선순위 처리
        if self.front_min_dist < 0.4:
            final_linear_x = -0.05 
            final_angular_z = self.avoid_angular_z * 2.0 
            if abs(final_angular_z) < 0.1: 
                 final_angular_z = 1.0 
                 
        elif self.front_min_dist < 0.7:
            final_linear_x = tq_cmd.linear.x * 0.3
        else:
            final_linear_x = tq_cmd.linear.x

        cmd.linear.x = max(min(final_linear_x, self.MAX_SPEED), -self.MAX_SPEED)
        cmd.angular.z = max(min(final_angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)

        self.cmd_pub.publish(cmd)
    
    # [수정] 콜백 함수 이름 변경
    def tq_pc_callback(self, msg):
        self.tq_pc = self.kf_pc.update(float(msg.data))
        # [디버깅] 데이터 수신 확인 로그
        #self.get_logger().info(f"PC 데이터 수신: {msg.data}",once = True)

    def tq_cam_callback(self, msg):
        self.tq_cam = self.kf_cam.update(float(msg.data))
        #self.get_logger().info(f"CAM 데이터 수신: {msg.data}",once = True)

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