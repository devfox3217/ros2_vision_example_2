import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LaneData
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        # 구독자 (Subscribers)
        self.sub_image = self.create_subscription(
            Image,
            '/lane_debug_img',
            self.image_callback,
            10)
            
        self.sub_data = self.create_subscription(
            LaneData,
            '/lane_data',
            self.data_callback,
            10)
            
        # 발행자 (Publishers) - 시각화된 결과 이미지
        self.pub_visual = self.create_publisher(Image, '/lane_visual', 10)
        
        self.cv_bridge = CvBridge()
        
        # 최신 차선 데이터 저장용 변수
        self.current_lane_data = None

    def data_callback(self, msg):
        # 최신 차선 데이터를 업데이트합니다.
        self.current_lane_data = msg

    def image_callback(self, msg):
        # 이미지를 받아서 텍스트 정보만 표시하고 재발행합니다.
        try:
            # 1. ROS 이미지를 OpenCV 이미지로 변환
            if msg.encoding == 'mono8':
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")
                color_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                color_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        # 2. 정보 표시
        if self.current_lane_data is not None and self.current_lane_data.lane_detected:
            height, width = color_image.shape[:2]
            
            # 각도 계산
            # dx: 상단 중심점과 하단 중심점의 x좌표 차이 (픽셀)
            dx = self.current_lane_data.curve_radius
            # dy: 이미지 높이의 절반 (lane_processor에서 설정한 y좌표 차이)
            dy = height / 2.0
            
            # 아크탄젠트로 각도 계산 (라디안 -> 도)
            # dx가 양수(오른쪽)면 각도도 양수, 음수(왼쪽)면 각도도 음수
            angle_rad = math.atan2(dx, dy)
            angle_deg = math.degrees(angle_rad)
            
            # 텍스트 정보 표시
            text_center = f"Center: {self.current_lane_data.lane_center:.2f}"
            text_curve = f"Curve Val: {self.current_lane_data.curve_radius:.1f}"
            text_angle = f"Angle: {angle_deg:.1f} deg"
            
            # 초록색 텍스트
            cv2.putText(color_image, text_center, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(color_image, text_curve, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 노란색 텍스트 (각도)
            cv2.putText(color_image, text_angle, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        else:
            height, width = color_image.shape[:2]
            # 차선 미검출 시 빨간색 텍스트
            cv2.putText(color_image, "NO LANE DETECTED", (50, int(height/2)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        # 3. 결과 이미지 발행
        visual_msg = self.cv_bridge.cv2_to_imgmsg(color_image, "bgr8")
        self.pub_visual.publish(visual_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()