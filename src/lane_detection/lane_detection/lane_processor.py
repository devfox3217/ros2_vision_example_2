import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LaneParams, LaneData
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneProcessor(Node):
    def __init__(self):
        super().__init__('lane_processor')
        
        # Subscribers
        self.subscription_video = self.create_subscription(
            Image,
            '/camera_open',
            self.image_callback,
            10)
        
        self.subscription_params = self.create_subscription(
            LaneParams,
            '/lane_params',
            self.params_callback,
            10)
            
        # Publishers
        self.publisher_lane_data = self.create_publisher(LaneData, '/lane_data', 10)
        self.publisher_debug_img = self.create_publisher(Image, '/lane_debug_img', 10)
        
        self.cv_bridge = CvBridge()
        
        # Default Parameters
        self.params = LaneParams()
        self.params.h_min = 0
        self.params.h_max = 179
        self.params.s_min = 0
        self.params.s_max = 50
        self.params.v_min = 200
        self.params.v_max = 255
        self.params.canny_low = 50
        self.params.canny_high = 150
        self.params.roi_height_percent = 60
        self.params.warp_offset = 150

    def params_callback(self, msg):
        self.params = msg

    def image_callback(self, msg):
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # 차선 인식 로직 수행
        processed_frame, lane_data = self.process_lane(frame)
        
        # 데이터 발행
        self.publisher_lane_data.publish(lane_data)
        
        # 디버깅 이미지 발행
        if processed_frame is not None:
            # processed_frame은 이제 BGR 컬러 이미지입니다 (박스가 그려진)
            debug_msg = self.cv_bridge.cv2_to_imgmsg(processed_frame, "bgr8")
            self.publisher_debug_img.publish(debug_msg)

    def process_lane(self, frame):
        height, width = frame.shape[:2]
        
        # 1. Preprocessing
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        
        # 2. Color Thresholding
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower = np.array([self.params.h_min, self.params.s_min, self.params.v_min])
        upper = np.array([self.params.h_max, self.params.s_max, self.params.v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 3. ROI
        roi_h = int(height * (self.params.roi_height_percent / 100.0))
        roi_mask = np.zeros_like(mask)
        if roi_h < height:
            roi_mask[roi_h:, :] = mask[roi_h:, :]
        else:
            roi_mask = mask
        
        # 4. Perspective Transform
        offset = self.params.warp_offset
        src_pts = np.float32([
            [width * 0.4, roi_h],
            [width * 0.6, roi_h],
            [width * 0.1, height],
            [width * 0.9, height]
        ])
        dst_pts = np.float32([
            [width * 0.2, 0],
            [width * 0.8, 0],
            [width * 0.2, height],
            [width * 0.8, height]
        ])
        
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(roi_mask, M, (width, height))
        
        # 시각화를 위해 흑백 -> 컬러 변환
        out_img = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        
        # 5. Sliding Window Search
        histogram = np.sum(warped[warped.shape[0]//2:, :], axis=0)
        
        if np.max(histogram) == 0:
            lane_data = LaneData()
            lane_data.lane_detected = False
            return out_img, lane_data

        midpoint = int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        nwindows = 9
        window_height = int(height / nwindows)
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        margin = 100
        minpix = 50
        
        left_lane_inds = []
        right_lane_inds = []
        
        for window in range(nwindows):
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # 박스 그리기 (Sliding Window 시각화)
            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
        
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        lane_detected = False
        curve_radius = 0.0
        lane_center_norm = 0.0
        
        if len(leftx) > 0 and len(rightx) > 0:
            lane_detected = True
            
            # 차선 픽셀 색칠 (왼쪽: 빨강, 오른쪽: 파랑)
            out_img[lefty, leftx] = [0, 0, 255]
            out_img[righty, rightx] = [255, 0, 0]
            
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
            
            y_eval = height
            left_fitx = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            right_fitx = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            
            y_eval_top = height / 2
            left_fitx_top = left_fit[0]*y_eval_top**2 + left_fit[1]*y_eval_top + left_fit[2]
            right_fitx_top = right_fit[0]*y_eval_top**2 + right_fit[1]*y_eval_top + right_fit[2]
            
            lane_center_px = (left_fitx + right_fitx) / 2.0
            image_center_px = width / 2.0
            
            lane_center_px_top = (left_fitx_top + right_fitx_top) / 2.0
            
            lane_center_norm = (lane_center_px - image_center_px) / (width / 2.0)
            
            curve_val = (lane_center_px_top - lane_center_px)
            curve_radius = float(curve_val)
            
            # 추정된 차선 중심선 그리기 (노란색)
            cv2.line(out_img, (int(lane_center_px), int(y_eval)), 
                     (int(lane_center_px_top), int(y_eval_top)), (0, 255, 255), 2)

        lane_data = LaneData()
        lane_data.lane_center = float(lane_center_norm)
        lane_data.curve_radius = curve_radius
        lane_data.lane_detected = bool(lane_detected)
        
        return out_img, lane_data

def main(args=None):
    rclpy.init(args=args)
    node = LaneProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()