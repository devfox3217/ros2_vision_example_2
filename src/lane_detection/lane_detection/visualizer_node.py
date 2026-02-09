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
        
        # Subscribe
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
            
        # Publish
        self.pub_visual = self.create_publisher(Image, '/lane_visual', 10)
        
        self.cv_bridge = CvBridge()
        
        # Data Holder
        self.current_lane_data = None

    def data_callback(self, msg):
        self.current_lane_data = msg

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            if msg.encoding == 'mono8':
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")
                color_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                color_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
                
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # Draw Info
        if self.current_lane_data is not None and self.current_lane_data.lane_detected:
            height, width = color_image.shape[:2]
            
            # Calculate Angle
            dx = self.current_lane_data.curve_radius
            dy = height / 2.0
            
            angle_rad = math.atan2(dx, dy)
            angle_deg = math.degrees(angle_rad)
            
            # Text Info
            text_center = f"Center: {self.current_lane_data.lane_center:.2f}"
            text_curve = f"Curve Val: {self.current_lane_data.curve_radius:.1f}"
            text_angle = f"Angle: {angle_deg:.1f} deg"
            
            # Green Text
            cv2.putText(color_image, text_center, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(color_image, text_curve, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Yellow Text (Angle)
            cv2.putText(color_image, text_angle, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
        else:
            height, width = color_image.shape[:2]
            # Red Warning
            cv2.putText(color_image, "NO LANE DETECTED", (50, int(height/2)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)

        # Publish Result
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