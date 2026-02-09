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
        # 원본 영상 구독
        self.subscription_video = self.create_subscription(
            Image,
            '/camera_open',
            self.image_callback,
            10)
        
        # 파라미터 설정 구독
        self.subscription_params = self.create_subscription(
            LaneParams,
            '/lane_params',
            self.params_callback,
            10)
            
        # Publishers
        # 처리된 차선 데이터 발행
        self.publisher_lane_data = self.create_publisher(LaneData, '/lane_data', 10)
        # 디버깅용 처리 이미지 발행 (Bird's Eye View 등)
        self.publisher_debug_img = self.create_publisher(Image, '/lane_debug_img', 10)
        
        self.cv_bridge = CvBridge()
        
        # Default Parameters (초기값)
        self.params = LaneParams()
        self.params.h_min = 0
        self.params.h_max = 179
        self.params.s_min = 0
        self.params.s_max = 255
        self.params.v_min = 0
        self.params.v_max = 255
        self.params.roi_height_percent = 60
        self.params.warp_offset = 50

    def params_callback(self, msg):
        """파라미터 노드로부터 업데이트된 설정값을 받습니다."""
        self.params = msg

    def image_callback(self, msg):
        """영상을 수신하여 차선 인식을 수행합니다."""
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
            return

        # 차선 인식 로직 수행
        processed_frame, lane_data = self.process_lane(frame)
        
        # 데이터 발행
        self.publisher_lane_data.publish(lane_data)
        
        # 디버깅 이미지 발행 (이진화 및 시점 변환된 이미지)
        if processed_frame is not None:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(processed_frame, "mono8")
            self.publisher_debug_img.publish(debug_msg)

    def process_lane(self, frame):
        height, width = frame.shape[:2]
        
        # 1. Preprocessing (Blur)
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        
        # 2. Color Thresholding (HSV)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower = np.array([self.params.h_min, self.params.s_min, self.params.v_min])
        upper = np.array([self.params.h_max, self.params.s_max, self.params.v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 3. ROI (Region of Interest)
        # 이미지 상단 불필요한 부분 제거
        roi_h = int(height * (self.params.roi_height_percent / 100.0))
        roi_mask = np.zeros_like(mask)
        # ROI 영역만 마스크 복사
        if roi_h < height:
            roi_mask[roi_h:, :] = mask[roi_h:, :]
        else:
            roi_mask = mask
        
        # 4. Perspective Transform (Bird's Eye View)
        # 소스 좌표: 이미지 하단의 사다리꼴 영역
        # 목적 좌표: 직사각형 (위에서 내려다본 뷰)
        
        # 파라미터로 조절 가능한 여백
        offset = self.params.warp_offset
        
        # 간단한 하드코딩된 비율 (실제로는 캘리브레이션 필요)
        src_pts = np.float32([
            [width * 0.4, roi_h],           # Top Left
            [width * 0.6, roi_h],           # Top Right
            [width * 0.1, height],          # Bottom Left
            [width * 0.9, height]           # Bottom Right
        ])
        
        dst_pts = np.float32([
            [width * 0.2, 0],               # Top Left
            [width * 0.8, 0],               # Top Right
            [width * 0.2, height],          # Bottom Left
            [width * 0.8, height]           # Bottom Right
        ])
        
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(roi_mask, M, (width, height))
        
        # 5. Feature Extraction (Histogram Sliding Window or Peak Finding)
        # 히스토그램을 계산하여 차선 위치 추정
        histogram = np.sum(warped[warped.shape[0]//2:, :], axis=0)
        
        midpoint = int(histogram.shape[0] / 2)
        
        # 히스토그램이 비어있는 경우 예외 처리
        if np.max(histogram) == 0:
            lane_data = LaneData()
            lane_data.lane_detected = False
            return warped, lane_data

        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        
        # 차선 중심 계산
        lane_center_px = (leftx_base + rightx_base) / 2.0
        image_center_px = width / 2.0
        
        # 정규화된 차선 중심 (-1.0: 좌측 끝, 0.0: 중앙, 1.0: 우측 끝)
        lane_center_norm = (lane_center_px - image_center_px) / (width / 2.0)
        
        # 차선 감지 여부 판단 (픽셀 수가 일정 이상이어야 함)
        lane_detected = np.max(histogram) > 500
        
        lane_data = LaneData()
        lane_data.lane_center = float(lane_center_norm)
        lane_data.curve_radius = 0.0 # 곡률 계산은 추후 구현
        lane_data.lane_detected = bool(lane_detected)
        
        return warped, lane_data

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