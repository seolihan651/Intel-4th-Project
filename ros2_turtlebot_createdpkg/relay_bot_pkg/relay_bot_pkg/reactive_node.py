import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
import random

# ---------------------------------------------------------
# 1. 1D 칼만 필터 클래스
# ---------------------------------------------------------
class SimpleKalmanFilter:
    def __init__(self, Q, R, P, initial_value):
        self.Q = Q              # 프로세스 노이즈
        self.R = R              # 측정 노이즈
        self.P = P              # 추정 오차
        self.X = initial_value  # 초기값

    def update(self, measurement):
        # 1. Prediction Update
        self.P = self.P + self.Q

        # 2. Measurement Update
        K = self.P / (self.P + self.R)      
        self.X = self.X + K * (measurement - self.X)
        self.P = (1 - K) * self.P
        
        return self.X

# ---------------------------------------------------------
# 2. 메인 로봇 제어 노드 (TQ 기반 수정)
# ---------------------------------------------------------
class ReactiveRelayBot(Node):
    def __init__(self):
        super().__init__('reactive_relay_bot')
        
        # QoS 설정
        qos_policy = QoSProfile(depth=10)
        
        # Subscriber & Publisher
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_policy)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # RSSI/TQ 구독 (이름은 RSSI지만 실제로는 TQ값 0~255가 들어옴)
        self.rssi_pc_sub = self.create_subscription(Int32, 'rssi/pc', self.rssi_pc_callback, 10)
        self.rssi_cam_sub = self.create_subscription(Int32, 'rssi/cam', self.rssi_cam_callback, 10)
        
        # [초기값 수정] TQ는 0~255 범위. 초기값은 양호한 상태(255)로 가정
        self.rssi_pc = 255.0
        self.rssi_cam = 255.0
        
        # 타이머 (0.5초 주기)
        self.timer = self.create_timer(0.5, self.control_loop)

        # [필터 설정] TQ값(0~255)은 스케일이 크므로 R값(노이즈)을 조금 더 키움
        # Q=0.5: TQ 변화에 적당히 민감하게
        # R=10.0: 일시적인 TQ 드롭을 필터링
        self.kf_pc = SimpleKalmanFilter(Q=0.5, R=10.0, P=5.0, initial_value=255.0)
        self.kf_cam = SimpleKalmanFilter(Q=0.5, R=10.0, P=5.0, initial_value=255.0)

        # 상태 변수
        self.obstacle_detected = False
        self.escape_direction = 0.0 

        self.avoid_angular_z = 0.0  
        self.front_min_dist = 10.0  
        
        self.prev_score = 0.0     
        self.action_state = 'FORWARD' 
        self.state_timer = 0      
        self.turn_direction = 1.0 
        self.last_cmd = Twist()   
        
        # 기본 주행 파라미터
        self.BASE_SPEED = 0.1
        self.MAX_SPEED = 0.22
        self.MAX_ANGULAR_SPEED = 1.0 
        self.scan_direction = 1.0 
        self.is_moving_forward = True 

    def scan_callback(self, msg):
        """ 장애물 회피 로직 (이전과 동일) """
        scan_ranges = msg.ranges
        cleaned_ranges = [r if (r > 0.01 and r < 10.0) else 10.0 for r in scan_ranges]
        
        front_ranges = cleaned_ranges[-30:] + cleaned_ranges[:30]
        self.front_min_dist = min(front_ranges)
        
        left_ranges = cleaned_ranges[0:90]
        right_ranges = cleaned_ranges[270:360]
        
        avg_left = sum(left_ranges) / len(left_ranges)
        avg_right = sum(right_ranges) / len(right_ranges)
        
        force_left = 1.0 / (avg_left + 0.1)
        force_right = 1.0 / (avg_right + 0.1)
        
        AVOID_GAIN = 1.5
        self.avoid_angular_z = (force_right - force_left) * AVOID_GAIN

    def calculate_quality(self):
        """ 
        [수정됨] batman-adv TQ 기반 품질 계산
        TQ: 0 (Bad) ~ 255 (Perfect)
        """
        def to_score(tq_val):
            # TQ 220 이상이면 만점 (매우 안정적)
            if tq_val >= 220: return 100.0
            # TQ 80 이하면 0점 (패킷 손실 심각)
            if tq_val <= 80: return 0.0
            
            # 80~220 사이를 0~100점으로 선형 변환
            # (val - 80) / (220 - 80) * 100
            score = (tq_val - 80.0) / 140.0 * 100.0
            return score

        s_pc = to_score(self.rssi_pc)
        s_cam = to_score(self.rssi_cam)
        
        self.current_s_pc = s_pc 
        
        # ---------------------------------------------------------
        # 1. 목줄 (The Leash) - TQ 버전
        # ---------------------------------------------------------
        # batman-adv에서 TQ 150 이하는 링크 품질이 떨어지기 시작하는 구간
        # TQ 120 미만은 위험으로 간주 (SAFE_THRESHOLD)
        SAFE_THRESHOLD_TQ = 120.0 
        
        if self.rssi_pc < SAFE_THRESHOLD_TQ:
            # 위험 상황: 복귀 강제
            return -100.0 

        # ---------------------------------------------------------
        # 2. 안전 구역 내에서의 행동
        # ---------------------------------------------------------
        final_score = min(s_pc, s_cam)
        final_score = final_score * 0.9 + (s_pc + s_cam) * 0.05
        
        return final_score
    

    def get_rssi_command(self):
        cmd = Twist()
        
        current_score = self.calculate_quality()
        diff = current_score - self.prev_score
        
        # [목표 도달]
        if current_score > 90.0: # 기준 점수 약간 상향
            self.last_cmd = Twist()
            return cmd

        # [상태 머신] Pivot & Probe (로직 구조 동일)
        if self.state_timer > 0:
            self.state_timer -= 1
            
            if self.action_state == 'PIVOT_TURN':
                cmd.linear.x = 0.0
                cmd.angular.z = self.stored_turn_speed
                if self.state_timer == 0:
                    self.action_state = 'STRAIGHT_PROBE'
                    self.state_timer = 15
                    self.get_logger().info("🚀 TQ 탐색: 직진(Probe) 시작.")
                return cmd

            elif self.action_state == 'STRAIGHT_PROBE':
                if diff < -5.0:
                    self.get_logger().warn(f"🚫 TQ 급락! ({diff:.2f}) 탐색 중단.")
                    self.state_timer = 0
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

        # 상황 1: TQ가 유지되거나 좋아짐
        if diff >= -NOISE_MARGIN:
            self.action_state = 'FORWARD_TRACKING'
            cmd.linear.x = self.BASE_SPEED
            cmd.angular.z = 0.05 * self.scan_direction 

            if diff > 1.0: 
                cmd.linear.x = self.MAX_SPEED

        # 상황 2: TQ 나빠짐
        else:
            self.get_logger().warn(f"📉 TQ 하락 (변화 {diff:.2f}). 재설정.")
            
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            turn_dir = random.choice([1.0, -1.0])
            self.stored_turn_speed = 1.0 * turn_dir 
            
            self.action_state = 'PIVOT_TURN'
            self.state_timer = 8 

        self.last_cmd = cmd
        self.prev_score = current_score
        return cmd
    
    
    def control_loop(self):
        """ 벡터 합성 제어 (동일) """
        cmd = Twist()
        rssi_cmd = self.get_rssi_command()
        
        final_angular_z = rssi_cmd.angular.z + self.avoid_angular_z
        
        if self.front_min_dist < 0.4:
            final_linear_x = -0.05 
            final_angular_z = self.avoid_angular_z * 2.0 
            if abs(final_angular_z) < 0.1: 
                 final_angular_z = 1.0 
                 
        elif self.front_min_dist < 0.7:
            final_linear_x = rssi_cmd.linear.x * 0.3
        else:
            final_linear_x = rssi_cmd.linear.x

        cmd.linear.x = max(min(final_linear_x, self.MAX_SPEED), -self.MAX_SPEED)
        cmd.angular.z = max(min(final_angular_z, self.MAX_ANGULAR_SPEED), -self.MAX_ANGULAR_SPEED)

        self.cmd_pub.publish(cmd)
    
    def rssi_pc_callback(self, msg):
        self.rssi_pc = self.kf_pc.update(float(msg.data))

    def rssi_cam_callback(self, msg):
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