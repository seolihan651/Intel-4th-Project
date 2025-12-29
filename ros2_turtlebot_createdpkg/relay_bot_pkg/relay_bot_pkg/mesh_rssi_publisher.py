import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import re

class MeshRssiPublisher(Node):
    def __init__(self):
        super().__init__('mesh_rssi_publisher')
        
        self.TARGET_MACS = {
            'pc':  '2c:cf:67:8c:2a:13',  
            'cam': '2c:cf:67:8c:2a:7b'   
        }
        
        self.pub_pc = self.create_publisher(Int32, 'rssi/pc', 10)
        self.pub_cam = self.create_publisher(Int32, 'rssi/cam', 10)
        
        self.timer = self.create_timer(1.0, self.update_tq)
        
        # [설정] 데이터 유효 시간 (초)
        # 이 시간보다 last-seen이 길면 연결 끊김으로 간주
        self.TIMEOUT_SEC = 2.0 
        
        self.get_logger().info("🦇 Mesh TQ Publisher Started (Timeout Protection ON)")

    def get_tq_from_batctl(self):
        tq_data = {}
        try:
            output_bytes = subprocess.check_output(
                ['sudo', 'batctl', 'o'], 
                stderr=subprocess.STDOUT
            )
            result = output_bytes.decode('utf-8')
            
            lines = result.split('\n')
            for line in lines:
                clean_line = line.strip()
                if not clean_line or "Originator" in clean_line: continue
                
                # Best Path(*) 체크
                if not clean_line.startswith('*'):
                    continue

                # 정규식 파싱: MAC, Last-seen, TQ
                # 예: * 2c:cf:67:8c:2a:7b    0.324s   (190) ...
                
                # 1. MAC 추출
                mac_match = re.search(r'([0-9a-fA-F:]{17})', clean_line)
                # 2. Last-seen 시간 추출 (숫자 + s)
                time_match = re.search(r'(\d+\.\d+)s', clean_line)
                # 3. TQ 값 추출 (괄호 안 숫자)
                tq_match = re.search(r'\(\s*(\d+)\s*\)', clean_line)
                
                if mac_match and tq_match and time_match:
                    found_mac = mac_match.group(1).lower()
                    last_seen = float(time_match.group(1))
                    tq_val = int(tq_match.group(1))
                    
                    # [핵심 로직] 시간이 너무 오래되었으면 TQ를 0으로 강제!
                    if last_seen > self.TIMEOUT_SEC:
                        self.get_logger().warn(f"⚠️ Stale data for {found_mac}: {last_seen}s ago. TQ->0")
                        tq_val = 0
                    
                    tq_data[found_mac] = tq_val

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            
        return tq_data

    def update_tq(self):
        current_tq_map = self.get_tq_from_batctl()
        
        # PC
        pc_mac = self.TARGET_MACS['pc'].lower()
        msg_pc = Int32()
        msg_pc.data = current_tq_map.get(pc_mac, 0) 
        self.pub_pc.publish(msg_pc)

        # CAM
        cam_mac = self.TARGET_MACS['cam'].lower()
        msg_cam = Int32()
        msg_cam.data = current_tq_map.get(cam_mac, 0)
        self.pub_cam.publish(msg_cam)

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