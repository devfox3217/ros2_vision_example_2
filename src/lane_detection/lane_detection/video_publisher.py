import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        # QoS Settings (Best Effort for Video)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Topic Name: /camera_open
        self.publisher_ = self.create_publisher(Image, '/camera_open', qos_profile)
        
        # 15 FPS (approx 0.066s) - Reduced from 30 to prevent lag
        self.timer_period = 0.066
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.cv_bridge = CvBridge()
        
        # Video File Path
        self.video_path = os.path.join(os.getcwd(), 'sample', 'camera_open.mp4')
        
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
            # Rewind
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to read video frame after rewind')
                return

        # Resize to 640x480
        frame_resized = cv2.resize(frame, (640, 480))

        # Convert to ROS Image
        msg = self.cv_bridge.cv2_to_imgmsg(frame_resized, encoding='bgr8')
        
        # Publish
        self.publisher_.publish(msg)

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