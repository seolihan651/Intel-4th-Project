import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import subprocess
import re

class MeshRssiPublisher(Node):
    def __init__(self):
        super().__init__('mesh_rssi_publisher')
        
        # ---------------------------------------------------------
        # [설정] MAC 주소 (소문자로 입력 권장)
        # ---------------------------------------------------------
        self.TARGET_MACS = {
            'pc':  '2c:cf:67:8c:2a:13',  
            'cam': '2c:cf:67:8c:29:c8'   
        }
        
        # Publisher
        self.pub_pc = self.create_publisher(Int32, 'rssi/pc', 10)
        self.pub_cam = self.create_publisher(Int32, 'rssi/cam', 10)
        
        # 1.0초마다 TQ 확인 (디버그 로그가 너무 많이 뜨지 않게 주기를 약간 늘림)
        self.timer = self.create_timer(1.0, self.update_tq)
        
        self.get_logger().info("🦇 Mesh TQ Publisher Started (Debug Mode ON)")

    def get_tq_from_batctl(self):
        """
        'sudo batctl o' 실행 후, '*' (Best Path)가 있는 라인의 TQ만 추출
        """
        tq_data = {}
        try:
            # 1. 명령어 실행
            output_bytes = subprocess.check_output(
                ['sudo', 'batctl', 'o'], 
                stderr=subprocess.STDOUT
            )
            result = output_bytes.decode('utf-8')
            
            # [디버그] 원본 출력 확인 (너무 길면 주석 처리 하세요)
            print("\n--- [DEBUG] Raw batctl output ---")
            print(result)
            print("---------------------------------")
            
            lines = result.split('\n')
            for line in lines:
                clean_line = line.strip()
                if not clean_line: continue
                
                # 헤더 건너뛰기
                if "Originator" in clean_line or "MainIF" in clean_line:
                    continue
                
                # 2. Best Path 확인
                # batctl o 에서 선택된 경로는 줄의 시작 부분에 '*'가 있음
                # 예: "* 2c:cf:67:8c:29:c8    0.632s   (249) ..."
                is_best_path = clean_line.startswith('*')
                
                if not is_best_path:
                    # Best path가 아니면 스킵 (혹은 디버그용으로 로그만 찍음)
                    # print(f"[DEBUG] Skipping non-best path: {clean_line[:30]}...")
                    continue

                # 3. 파싱 (MAC 주소 및 TQ 값)
                # 정규식: 
                #  MAC: 2자리 16진수와 콜론이 5번 반복되고 마지막 2자리
                #  TQ : 괄호 '(' 뒤에 숫자 '\d+' 뒤에 ')'
                
                mac_match = re.search(r'([0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2}:[0-9a-fA-F]{2})', clean_line)
                tq_match = re.search(r'\(\s*(\d+)\s*\)', clean_line)
                
                if mac_match and tq_match:
                    found_mac = mac_match.group(1).lower()
                    tq_val = int(tq_match.group(1))
                    
                    tq_data[found_mac] = tq_val
                    print(f"✅ [Parsed] MAC: {found_mac} | TQ: {tq_val} (Best Path)")
                else:
                    print(f"⚠️ [Parse Fail] Line has '*' but regex failed: {clean_line}")

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Command failed: {e.output}")
        except Exception as e:
            self.get_logger().error(f"Error parsing TQ: {e}")
            
        return tq_data

    def update_tq(self):
        # 1. 파싱 데이터 가져오기
        current_tq_map = self.get_tq_from_batctl()
        
        # 2. PC TQ 발행
        target_pc = self.TARGET_MACS['pc'].lower()
        msg_pc = Int32()
        if target_pc in current_tq_map:
            msg_pc.data = current_tq_map[target_pc]
        else:
            msg_pc.data = 0 # 데이터 없음
            # print(f"❌ PC MAC ({target_pc}) not found in best paths.")
            
        self.pub_pc.publish(msg_pc)

        # 3. CAM TQ 발행
        target_cam = self.TARGET_MACS['cam'].lower()
        msg_cam = Int32()
        if target_cam in current_tq_map:
            msg_cam.data = current_tq_map[target_cam]
        else:
            msg_cam.data = 0 # 데이터 없음
            
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