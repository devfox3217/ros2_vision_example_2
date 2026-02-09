import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LaneParams, LaneData
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        
        # Subscribers
        self.sub_camera = self.create_subscription(Image, '/camera_open', self.camera_callback, 10)
        self.sub_debug = self.create_subscription(Image, '/lane_debug_img', self.debug_callback, 10)
        self.sub_visual = self.create_subscription(Image, '/lane_visual', self.visual_callback, 10)
        
        # Lane Data Subscriber (For Angle Display)
        self.sub_lane_data = self.create_subscription(LaneData, '/lane_data', self.lane_data_callback, 10)
        
        # Publisher (Parameter Control)
        self.pub_params = self.create_publisher(LaneParams, '/lane_params', 10)
        
        self.cv_bridge = CvBridge()
        
        # Image Data Holders
        self.img_camera = None
        self.img_debug = None
        self.img_visual = None
        self.current_lane_data = None
        
        # Default Params (Matched with lane_processor.py)
        self.default_params = {
            'h_min': 0, 'h_max': 180,
            's_min': 0, 's_max': 50,
            'v_min': 180, 'v_max': 255,
            'canny_low': 80, 'canny_high': 200,
            'roi_height_percent': 70, 'warp_offset': 200
        }
        
        self.params = LaneParams()
        # Set initial values
        for key, val in self.default_params.items():
            setattr(self.params, key, val)

        # Initialize OpenCV Window
        self.window_name = "Lane Detection Dashboard"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # Increase window height to fit all images and text
        cv2.resizeWindow(self.window_name, 500, 950)

        # Create Trackbars
        self.create_trackbars()

    def create_trackbars(self):
        def nothing(x):
            pass
            
        # Create trackbars
        # Reset Button as a Trackbar (Switch)
        cv2.createTrackbar("Reset Params", self.window_name, 0, 1, nothing)
        
        cv2.createTrackbar("H Min", self.window_name, self.params.h_min, 179, nothing)
        cv2.createTrackbar("H Max", self.window_name, self.params.h_max, 179, nothing)
        cv2.createTrackbar("S Min", self.window_name, self.params.s_min, 255, nothing)
        cv2.createTrackbar("S Max", self.window_name, self.params.s_max, 255, nothing)
        cv2.createTrackbar("V Min", self.window_name, self.params.v_min, 255, nothing)
        cv2.createTrackbar("V Max", self.window_name, self.params.v_max, 255, nothing)
        cv2.createTrackbar("Canny Low", self.window_name, self.params.canny_low, 255, nothing)
        cv2.createTrackbar("Canny High", self.window_name, self.params.canny_high, 255, nothing)
        cv2.createTrackbar("ROI Height %", self.window_name, self.params.roi_height_percent, 100, nothing)
        cv2.createTrackbar("Warp Offset", self.window_name, self.params.warp_offset, 200, nothing)

    def reset_params(self):
        self.get_logger().info("Resetting parameters to default")
        for key, val in self.default_params.items():
            setattr(self.params, key, val)
            # Update trackbar positions
            tb_name = key.replace('_', ' ').title().replace('H Min', 'H Min').replace('Roi', 'ROI')
            mapping = {
                'h_min': 'H Min', 'h_max': 'H Max',
                's_min': 'S Min', 's_max': 'S Max',
                'v_min': 'V Min', 'v_max': 'V Max',
                'canny_low': 'Canny Low', 'canny_high': 'Canny High',
                'roi_height_percent': 'ROI Height %', 'warp_offset': 'Warp Offset'
            }
            if key in mapping:
                cv2.setTrackbarPos(mapping[key], self.window_name, val)
        
        # Reset the switch trackbar back to 0
        cv2.setTrackbarPos("Reset Params", self.window_name, 0)
        
        self.pub_params.publish(self.params)

    def camera_callback(self, msg):
        try:
            self.img_camera = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def debug_callback(self, msg):
        try:
            # Debug image is now BGR (with boxes)
            if msg.encoding == 'mono8':
                self.img_debug = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")
                self.img_debug = cv2.cvtColor(self.img_debug, cv2.COLOR_GRAY2BGR)
            else:
                self.img_debug = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def visual_callback(self, msg):
        try:
            self.img_visual = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
            
    def lane_data_callback(self, msg):
        self.current_lane_data = msg
            
    def update_params(self):
        # Read trackbar positions
        try:
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                return

            # Check Reset Switch
            reset_val = cv2.getTrackbarPos("Reset Params", self.window_name)
            if reset_val == 1:
                self.reset_params()
                return

            self.params.h_min = cv2.getTrackbarPos("H Min", self.window_name)
            self.params.h_max = cv2.getTrackbarPos("H Max", self.window_name)
            self.params.s_min = cv2.getTrackbarPos("S Min", self.window_name)
            self.params.s_max = cv2.getTrackbarPos("S Max", self.window_name)
            self.params.v_min = cv2.getTrackbarPos("V Min", self.window_name)
            self.params.v_max = cv2.getTrackbarPos("V Max", self.window_name)
            self.params.canny_low = cv2.getTrackbarPos("Canny Low", self.window_name)
            self.params.canny_high = cv2.getTrackbarPos("Canny High", self.window_name)
            self.params.roi_height_percent = cv2.getTrackbarPos("ROI Height %", self.window_name)
            self.params.warp_offset = cv2.getTrackbarPos("Warp Offset", self.window_name)
            
            self.pub_params.publish(self.params)
        except cv2.error:
            pass

    def show_images(self):
        # Create a canvas
        # Height: 950, Width: 500
        canvas_h = 950
        canvas_w = 500
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        
        # Define image size for display (e.g., 320x240)
        disp_w, disp_h = 320, 240
        
        # Helper to resize and place image
        def place_image(img, x, y, title):
            if img is not None:
                resized = cv2.resize(img, (disp_w, disp_h))
                canvas[y:y+disp_h, x:x+disp_w] = resized
            else:
                # Placeholder
                cv2.rectangle(canvas, (x, y), (x+disp_w, y+disp_h), (50, 50, 50), -1)
                cv2.putText(canvas, "No Signal", (x+100, y+120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            
            # Draw border and title
            cv2.rectangle(canvas, (x, y), (x+disp_w, y+disp_h), (255, 255, 255), 2)
            cv2.putText(canvas, title, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Center the images horizontally
        x_left = 90
        y_start = 40
        gap = 30
        
        place_image(self.img_camera, x_left, y_start, "Original Camera")
        place_image(self.img_debug, x_left, y_start + disp_h + gap, "Processed (Lane Mask)")
        place_image(self.img_visual, x_left, y_start + (disp_h + gap) * 2, "Final Visualization")
        
        # Display Steering Angle at the bottom
        y_text = y_start + (disp_h + gap) * 3 + 20
        
        if self.current_lane_data is not None and self.current_lane_data.lane_detected:
            # Calculate Angle
            # Assuming image height used in processor was 480 (standard VGA) or similar ratio
            # We use the same logic as visualizer_node
            # But we don't know the exact height used in processor here easily without image
            # However, we can use a fixed reference or just display the raw value if needed
            # Let's use a standard assumption or just display the curve_radius for now
            # Better: Calculate angle assuming standard aspect ratio or use the value directly
            
            dx = self.current_lane_data.curve_radius
            # Approximate dy as half of standard height (e.g. 240 if 480p)
            # Or better, if we have img_camera, use its height
            dy = 240.0 
            if self.img_camera is not None:
                dy = self.img_camera.shape[0] / 2.0
            
            angle_rad = math.atan2(dx, dy)
            angle_deg = math.degrees(angle_rad)
            
            text = f"Steering Angle: {angle_deg:.1f} deg"
            
            # Center the text
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            tx = (canvas_w - tw) // 2
            
            # Color based on angle (Green: Straight, Red: Turn)
            color = (0, 255, 0)
            if abs(angle_deg) > 10:
                color = (0, 165, 255) # Orange
            if abs(angle_deg) > 20:
                color = (0, 0, 255) # Red
                
            cv2.putText(canvas, text, (tx, y_text), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        else:
            text = "NO LANE DETECTED"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            tx = (canvas_w - tw) // 2
            cv2.putText(canvas, text, (tx, y_text), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        
        cv2.imshow(self.window_name, canvas)

        key = cv2.waitKey(1)
        if key == 27: # ESC
            rclpy.shutdown()
        elif key == ord('r') or key == ord('R'):
            self.reset_params()

def main(args=None):
    rclpy.init(args=args)
    node = GUINode()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_params()
            node.show_images()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()