import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LaneData
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisualizerNode(Node):
    def __init__(self):
        super().__init__('visualizer_node')
        
        # 구독 (Subscribe)
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
            
        # 발행 (Publish) - 시각화된 결과 이미지
        self.pub_visual = self.create_publisher(Image, '/lane_visual', 10)
        
        self.cv_bridge = CvBridge()
        
        # 최신 차선 데이터 저장용 변수
        self.current_lane_data = None

    def data_callback(self, msg):
        """최신 차선 데이터를 업데이트합니다."""
        self.current_lane_data = msg

    def image_callback(self, msg):
        """이미지를 받아서 화살표를 그리고 재발행합니다."""
        try:
            # 1. ROS 이미지를 OpenCV 이미지로 변환
            # 입력이 mono8(흑백)일 수 있으므로 컬러(BGR)로 변환해야 초록색을 그릴 수 있음
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")
            color_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # 2. 화살표 그리기
        if self.current_lane_data is not None:
            height, width = color_image.shape[:2]
            
            # 화살표 시작점 (이미지 하단 중앙)
            start_point = (int(width / 2), height)
            
            # 화살표 끝점 계산
            # lane_center는 -1.0(좌) ~ 1.0(우) 범위, 0.0이 중앙
            # 화면 좌표계로 변환: (lane_center * 반너비) + 중앙값
            center_offset = self.current_lane_data.lane_center * (width / 2)
            end_x = int((width / 2) + center_offset)
            end_y = int(height / 2) # 화면 중간까지만 화살표 그리기
            
            end_point = (end_x, end_y)
            
            # 초록색 화살표 그리기 (BGR: 0, 255, 0), 두께 5
            cv2.arrowedLine(color_image, start_point, end_point, (0, 255, 0), 5, tipLength=0.1)
            
            # 텍스트 정보 표시
            text = f"Center: {self.current_lane_data.lane_center:.2f}"
            cv2.putText(color_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.7, (0, 255, 0), 2)

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