import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        # 토픽 이름: /camera_open
        self.publisher_ = self.create_publisher(Image, '/camera_open', 10)
        
        # 30 FPS (약 0.033초 주기)
        self.timer_period = 0.033
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.cv_bridge = CvBridge()
        
        # 비디오 파일 경로 (프로젝트 내 sample 폴더)
        # 실제 환경에 맞게 경로를 수정하거나 파라미터로 받을 수 있습니다.
        self.video_path = 'D:/Projects/ros2_vision_example_2/sample/camera_open.mp4'
        
        self.cap = cv2.VideoCapture(self.video_path)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open video file: {self.video_path}')
        else:
            self.get_logger().info(f'Video file opened: {self.video_path}')

    def timer_callback(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        
        if not ret:
            # 영상이 끝나면 처음으로 되감기 (무한 루프)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to read video frame after rewind')
                return

        # OpenCV 이미지(BGR)를 ROS Image 메시지로 변환
        msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # 메시지 발행
        self.publisher_.publish(msg)
        # 디버깅을 위해 로그를 너무 자주 찍지 않도록 주석 처리하거나 필요시 활성화
        # self.get_logger().info('Publishing video frame')

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()