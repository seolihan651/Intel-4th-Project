import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import sys
import time

class MapSender(Node):
    def __init__(self):
        super().__init__('map_sender')
        
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)
        
    def send_image(self, image_path):
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return

        img = cv2.flip(img, 0) # 상하 반전
        height, width = img.shape

        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg() # 시간 도장 찍기
        msg.info.resolution = 0.05
        msg.info.width = width
        msg.info.height = height
        msg.info.origin = Pose()
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0

        flat_img = img.flatten()
        grid_data = []
        
        # 픽셀값 변환
        for pixel in flat_img:
            if pixel > 200:   
                grid_data.append(0)   # 흰색 -> 이동 가능
            elif pixel < 50:  
                grid_data.append(100) # 검은색 -> 벽
            else:             
                grid_data.append(-1)  # 그 외 -> 미탐색

        msg.data = grid_data

        for i in range(1):
            self.publisher_.publish(msg)
            print(f"지도 데이터 전송 중")
            time.sleep(1)
            
        print("전송 완료")

def main():
    rclpy.init()
    node = MapSender()
    
    node.send_image('mymap.png')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
