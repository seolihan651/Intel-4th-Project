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
        # 터미널에서 'ip link' 또는 'ifconfig'로 확인한 HWaddr
        # ---------------------------------------------------------
        self.TARGET_MACS = {
            'pc':  '2c:cf:67:8c:2a:13',  # <--- PC의 무선랜 MAC 주소로 변경!
            'cam': '2c:cf:67:8c:29:c8'   # <--- BodyCam(RPi)의 무선랜 MAC 주소로 변경!
        }
        
        # 인터페이스 이름 (보통 라즈베리파이는 wlan0)
        self.INTERFACE = 'wlan0' 

        # Publisher 생성
        self.pub_pc = self.create_publisher(Int32, 'rssi/pc', 10)
        self.pub_cam = self.create_publisher(Int32, 'rssi/cam', 10)
        
        # 0.5초마다 RSSI 확인 (너무 자주하면 CPU 부담)
        self.timer = self.create_timer(0.5, self.update_rssi)
        
        self.get_logger().info("📡 Mesh RSSI Publisher Started")

    def get_rssi_from_iw(self):
        """
        'iw dev wlan0 station dump' 명령어를 실행하여
        연결된 모든 메쉬 이웃의 신호 세기를 파싱합니다.
        """
        rssi_data = {}
        try:
            # 리눅스 명령어 실행
            result = subprocess.check_output(
                ['iw', 'dev', self.INTERFACE, 'station', 'dump'], 
                stderr=subprocess.STDOUT
            ).decode('utf-8')
            
            # 파싱 로직: 'Station'으로 시작해서 MAC이 나오고, 뒤이어 'signal:'이 나옴
            current_mac = None
            
            for line in result.split('\n'):
                line = line.strip()
                
                # 1. MAC 주소 찾기 (예: Station 12:34:56:78:90:ab (on wlan0))
                if line.startswith('Station'):
                    parts = line.split()
                    if len(parts) >= 2:
                        current_mac = parts[1].lower() # 소문자로 통일
                
                # 2. 신호 세기 찾기 (예: signal:  -54 dBm)
                if current_mac and line.startswith('signal:'):
                    # 'signal:', '-54', 'dBm' 등으로 쪼개짐
                    parts = line.split()
                    if len(parts) >= 2:
                        try:
                            # 'avg:' 같은 게 붙어있을 수 있으므로 숫자만 추출
                            # 보통 parts[1]이 '-54' 임
                            rssi_val = int(parts[1])
                            rssi_data[current_mac] = rssi_val
                        except ValueError:
                            pass
                            
        except subprocess.CalledProcessError:
            self.get_logger().error("Failed to execute iw command. sudo 권한이 필요한가요?")
        except Exception as e:
            self.get_logger().error(f"Error parsing RSSI: {e}")
            
        return rssi_data

    def update_rssi(self):
        # 1. 전체 스캔
        current_rssi_map = self.get_rssi_from_iw()
        
        # 2. PC 신호 발행
        pc_mac = self.TARGET_MACS['pc'].lower()
        if pc_mac in current_rssi_map:
            msg = Int32()
            msg.data = current_rssi_map[pc_mac]
            self.pub_pc.publish(msg)
            # self.get_logger().info(f"PC RSSI: {msg.data} dBm")
        else:
            # 연결 끊김 혹은 감지 안됨 -> 안전을 위해 매우 낮은 값 발행
            msg = Int32()
            msg.data = -99
            self.pub_pc.publish(msg)

        # 3. CAM 신호 발행
        cam_mac = self.TARGET_MACS['cam'].lower()
        if cam_mac in current_rssi_map:
            msg = Int32()
            msg.data = current_rssi_map[cam_mac]
            self.pub_cam.publish(msg)
        else:
            msg = Int32()
            msg.data = -99
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