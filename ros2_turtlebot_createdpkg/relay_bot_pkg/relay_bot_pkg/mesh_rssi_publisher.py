import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import re

class MeshRssiPublisher(Node):
    def __init__(self):
        super().__init__('mesh_rssi_publisher')
        
        # ---------------------------------------------------------
        # [설정] PC와 카메라(혹은 다른 로봇)의 MAC 주소 입력
        # batman-adv mesh 인터페이스(bat0)가 아닌 
        # 실제 물리 인터페이스(wlan0)의 MAC 주소가 출력되는 경우가 많으므로 확인 필요
        # ---------------------------------------------------------
        self.TARGET_MACS = {
            'pc':  '2c:cf:67:8c:2a:13',  
            'cam': '2c:cf:67:8c:29:c8'   
        }
        
        # Publisher 생성 (이름은 rssi로 유지하지만 내용은 TQ값입니다)
        self.pub_pc = self.create_publisher(Int32, 'rssi/pc', 10)
        self.pub_cam = self.create_publisher(Int32, 'rssi/cam', 10)
        
        # 0.5초마다 TQ 확인
        self.timer = self.create_timer(0.5, self.update_tq)
        
        self.get_logger().info("🦇 Mesh TQ Publisher Started (using batctl)")

    def get_tq_from_batctl(self):
        """
        'sudo batctl n' 명령어를 실행하여 TQ(Transmission Quality)를 파싱합니다.
        TQ 값은 0~255 사이의 정수이며, 255가 최상의 품질입니다.
        """
        tq_data = {}
        try:
            # batctl n 실행 (Neighbors table)
            # 출력 예시: wlan0  02:11:22:33:44:55    0.200s   (245) ...
            result = subprocess.check_output(
                ['sudo', 'batctl', 'n'], 
                stderr=subprocess.STDOUT
            ).decode('utf-8')
            
            lines = result.split('\n')
            for line in lines:
                line = line.strip()
                # 헤더 건너뛰기
                if "Neigh" in line or "IF" in line:
                    continue
                
                # 정규표현식으로 MAC과 (...) 괄호 안의 TQ 값 추출
                # 예: 02:11:22:33:44:55 ... (245)
                # MAC 패턴: 2자리 6개
                match = re.search(r'([0-9a-fA-F:]{17}).*\(\s*(\d+)\s*\)', line)
                
                if match:
                    found_mac = match.group(1).lower()
                    tq_val = int(match.group(2))
                    tq_data[found_mac] = tq_val

        except subprocess.CalledProcessError:
            self.get_logger().error("Failed to execute batctl. 'sudo' 권한이 있나요?")
        except Exception as e:
            self.get_logger().error(f"Error parsing TQ: {e}")
            
        return tq_data

    def update_tq(self):
        # 1. batctl n 스캔
        current_tq_map = self.get_tq_from_batctl()
        
        # 2. PC TQ 발행
        pc_mac = self.TARGET_MACS['pc'].lower()
        if pc_mac in current_tq_map:
            msg = Int32()
            msg.data = current_tq_map[pc_mac]
            self.pub_pc.publish(msg)
            # self.get_logger().info(f"PC TQ: {msg.data}/255")
        else:
            # 연결 끊김 -> TQ 0 처리
            msg = Int32()
            msg.data = 0
            self.pub_pc.publish(msg)

        # 3. CAM TQ 발행
        cam_mac = self.TARGET_MACS['cam'].lower()
        if cam_mac in current_tq_map:
            msg = Int32()
            msg.data = current_tq_map[cam_mac]
            self.pub_cam.publish(msg)
        else:
            msg = Int32()
            msg.data = 0
            self.pub_cam.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MeshRssiPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()