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
        
        # 구독자 (Subscribers)
        # 원본 카메라 영상 구독
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
            
        # 발행자 (Publishers)
        # 처리된 차선 데이터 발행
        self.publisher_lane_data = self.create_publisher(LaneData, '/lane_data', 10)
        # 디버깅용 처리 이미지 발행
        self.publisher_debug_img = self.create_publisher(Image, '/lane_debug_img', 10)
        
        self.cv_bridge = CvBridge()
        
        # 기본 파라미터 설정 (흰색 차선 기준)
        self.params = LaneParams()
        self.params.h_min = 0
        self.params.h_max = 179     # 모든 색상
        self.params.s_min = 0
        self.params.s_max = 50      # 흰색을 위해 낮은 채도 설정
        self.params.v_min = 200     # 높은 명도 (밝은 색)
        self.params.v_max = 255
        self.params.roi_height_percent = 60
        self.params.warp_offset = 150
        # 슬라이딩 윈도우 기본값
        self.params.sw_margin = 100
        self.params.sw_minpix = 50
        self.params.debug_view = 1 # 기본값: 켜짐

    def params_callback(self, msg):
        # 파라미터 노드로부터 업데이트된 설정값을 받습니다.
        self.params = msg

    def image_callback(self, msg):
        # 영상을 수신하여 차선 인식을 수행합니다.
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'이미지 변환 실패: {e}')
            return

        # 차선 인식 로직 수행
        processed_frame, lane_data = self.process_lane(frame)
        
        # 데이터 발행
        self.publisher_lane_data.publish(lane_data)
        
        # 디버깅 이미지 발행 (옵션)
        if self.params.debug_view == 1 and processed_frame is not None:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(processed_frame, "bgr8")
            self.publisher_debug_img.publish(debug_msg)

    def process_lane(self, frame):
        height, width = frame.shape[:2]
        
        # 1. 전처리 (블러링)
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        
        # 2. 색상 필터링 (HSV)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        lower = np.array([self.params.h_min, self.params.s_min, self.params.v_min])
        upper = np.array([self.params.h_max, self.params.s_max, self.params.v_max])
        mask = cv2.inRange(hsv, lower, upper)
        
        # 3. 관심 영역 (ROI) 설정
        # 이미지 상단 불필요한 부분 제거
        roi_h = int(height * (self.params.roi_height_percent / 100.0))
        roi_mask = np.zeros_like(mask)
        # ROI 영역만 마스크 복사
        if roi_h < height:
            roi_mask[roi_h:, :] = mask[roi_h:, :]
        else:
            roi_mask = mask
        
        # 4. 시점 변환 (Bird's Eye View)
        # 소스 좌표: 이미지 하단의 사다리꼴 영역
        # 목적 좌표: 직사각형 (위에서 내려다본 뷰)
        
        # 파라미터로 조절 가능한 여백
        offset = self.params.warp_offset
        
        # 간단한 하드코딩된 비율 (실제로는 캘리브레이션 필요)
        src_pts = np.float32([
            [width * 0.4, roi_h],           # 좌상단
            [width * 0.6, roi_h],           # 우상단
            [width * 0.1, height],          # 좌하단
            [width * 0.9, height]           # 우하단
        ])
        
        dst_pts = np.float32([
            [width * 0.2, 0],               # 좌상단
            [width * 0.8, 0],               # 우상단
            [width * 0.2, height],          # 좌하단
            [width * 0.8, height]           # 우하단
        ])
        
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)
        warped = cv2.warpPerspective(roi_mask, M, (width, height))
        
        # 5. 슬라이딩 윈도우 탐색
        # 히스토그램을 계산하여 차선 시작 위치 추정
        histogram = np.sum(warped[warped.shape[0]//2:, :], axis=0)
        
        # 히스토그램이 비어있는 경우 예외 처리
        if np.max(histogram) == 0:
            lane_data = LaneData()
            lane_data.lane_detected = False
            return None, lane_data

        midpoint = int(histogram.shape[0] / 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # 슬라이딩 윈도우 설정
        nwindows = 9
        window_height = int(height / nwindows)
        nonzero = warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        margin = self.params.sw_margin
        minpix = self.params.sw_minpix
        
        left_lane_inds = []
        right_lane_inds = []
        
        # 디버그 이미지가 필요할 때만 생성
        out_img = None
        if self.params.debug_view == 1:
            out_img = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
        
        for window in range(nwindows):
            # 윈도우 경계 설정
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin
            
            # 박스 그리기 (디버그 뷰 ON일 때만)
            if self.params.debug_view == 1:
                cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 2)
                cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            # 윈도우 내의 비트 픽셀 인덱스 추출
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            # 픽셀이 충분하면 다음 윈도우의 중심을 평균 위치로 이동
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))
        
        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)
        
        # 차선 픽셀 좌표 추출
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        lane_detected = False
        curve_radius = 0.0
        lane_center_norm = 0.0
        
        if len(leftx) > 0 and len(rightx) > 0:
            lane_detected = True
            
            # 2차 함수 피팅 (x = ay^2 + by + c)
            left_fit = np.polyfit(lefty, leftx, 2)
            right_fit = np.polyfit(righty, rightx, 2)
            
            # 이미지 하단(차량 바로 앞)에서의 차선 위치
            y_eval = height
            left_fitx = left_fit[0]*y_eval**2 + left_fit[1]*y_eval + left_fit[2]
            right_fitx = right_fit[0]*y_eval**2 + right_fit[1]*y_eval + right_fit[2]
            
            # 이미지 상단(먼 곳)에서의 차선 위치 (예: 이미지의 1/2 지점)
            y_eval_top = height / 2
            left_fitx_top = left_fit[0]*y_eval_top**2 + left_fit[1]*y_eval_top + left_fit[2]
            right_fitx_top = right_fit[0]*y_eval_top**2 + right_fit[1]*y_eval_top + right_fit[2]
            
            # 차선 중심 계산 (하단)
            lane_center_px = (left_fitx + right_fitx) / 2.0
            image_center_px = width / 2.0
            
            # 차선 중심 계산 (상단) - 곡률 반영을 위해
            lane_center_px_top = (left_fitx_top + right_fitx_top) / 2.0
            
            # 정규화된 차선 중심 (-1.0 ~ 1.0)
            lane_center_norm = (lane_center_px - image_center_px) / (width / 2.0)
            
            # 곡률(Curve Radius) 대신 조향각에 가까운 값 계산
            # 상단 중심점과 하단 중심점의 차이를 이용하여 회전 정도 추정
            curve_val = (lane_center_px_top - lane_center_px)
            
            # curve_radius 필드에 임시로 회전 정도를 담아서 보냄 (단위는 픽셀 차이)
            curve_radius = float(curve_val)
            
            # 시각화 (디버그 뷰 ON일 때만)
            if self.params.debug_view == 1:
                out_img[lefty, leftx] = [0, 0, 255]
                out_img[righty, rightx] = [255, 0, 0]
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