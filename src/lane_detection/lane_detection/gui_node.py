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
        
        # 구독자 (Subscribers)
        self.sub_camera = self.create_subscription(Image, '/camera_open', self.camera_callback, 10)
        self.sub_debug = self.create_subscription(Image, '/lane_debug_img', self.debug_callback, 10)
        self.sub_visual = self.create_subscription(Image, '/lane_visual', self.visual_callback, 10)
        
        # 차선 데이터 구독 (조향각 표시용)
        self.sub_lane_data = self.create_subscription(LaneData, '/lane_data', self.lane_data_callback, 10)
        
        # 발행자 (Publishers) - 파라미터 제어
        self.pub_params = self.create_publisher(LaneParams, '/lane_params', 10)
        
        self.cv_bridge = CvBridge()
        
        # 이미지 데이터 저장소
        self.img_camera = None
        self.img_debug = None
        self.img_visual = None
        self.current_lane_data = None
        
        # 기본 파라미터 (lane_processor.py와 일치)
        self.default_params = {
            'h_min': 0, 'h_max': 180,
            's_min': 0, 's_max': 50,
            'v_min': 180, 'v_max': 255,
            'roi_height_percent': 70, 'warp_offset': 200,
            'sw_margin': 100, 'sw_minpix': 50,
            'debug_view': 1 # 기본값: 켜짐
        }
        
        self.params = LaneParams()
        # 초기값 설정
        for key, val in self.default_params.items():
            setattr(self.params, key, val)

        # OpenCV 윈도우 초기화
        self.window_name = "Lane Detection Dashboard"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # 모든 이미지와 텍스트가 들어갈 수 있도록 윈도우 크기 설정
        cv2.resizeWindow(self.window_name, 500, 950)

        # 트랙바 생성
        self.create_trackbars()

    def create_trackbars(self):
        def nothing(x):
            pass
            
        # 트랙바 생성
        # 디버그 뷰 스위치
        cv2.createTrackbar("Debug View", self.window_name, self.params.debug_view, 1, nothing)
        
        # 리셋 버튼 (스위치 형태)
        cv2.createTrackbar("Reset Params", self.window_name, 0, 1, nothing)
        
        cv2.createTrackbar("H Min", self.window_name, self.params.h_min, 179, nothing)
        cv2.createTrackbar("H Max", self.window_name, self.params.h_max, 179, nothing)
        cv2.createTrackbar("S Min", self.window_name, self.params.s_min, 255, nothing)
        cv2.createTrackbar("S Max", self.window_name, self.params.s_max, 255, nothing)
        cv2.createTrackbar("V Min", self.window_name, self.params.v_min, 255, nothing)
        cv2.createTrackbar("V Max", self.window_name, self.params.v_max, 255, nothing)
        cv2.createTrackbar("ROI Height %", self.window_name, self.params.roi_height_percent, 100, nothing)
        cv2.createTrackbar("Warp Offset", self.window_name, self.params.warp_offset, 200, nothing)
        
        # 슬라이딩 윈도우 파라미터
        cv2.createTrackbar("SW Margin", self.window_name, self.params.sw_margin, 200, nothing)
        cv2.createTrackbar("SW MinPix", self.window_name, self.params.sw_minpix, 200, nothing)

    def reset_params(self):
        self.get_logger().info("파라미터를 기본값으로 초기화합니다.")
        for key, val in self.default_params.items():
            setattr(self.params, key, val)
            # 트랙바 위치 업데이트
            tb_name = key.replace('_', ' ').title().replace('H Min', 'H Min').replace('Roi', 'ROI')
            mapping = {
                'h_min': 'H Min', 'h_max': 'H Max',
                's_min': 'S Min', 's_max': 'S Max',
                'v_min': 'V Min', 'v_max': 'V Max',
                'roi_height_percent': 'ROI Height %', 'warp_offset': 'Warp Offset',
                'sw_margin': 'SW Margin', 'sw_minpix': 'SW MinPix',
                'debug_view': 'Debug View'
            }
            if key in mapping:
                cv2.setTrackbarPos(mapping[key], self.window_name, val)
        
        # 리셋 스위치를 다시 0으로 복귀
        cv2.setTrackbarPos("Reset Params", self.window_name, 0)
        
        self.pub_params.publish(self.params)

    def camera_callback(self, msg):
        try:
            self.img_camera = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def debug_callback(self, msg):
        try:
            # 디버그 이미지는 BGR 포맷 (박스 포함)
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
        # 트랙바 위치 읽기
        try:
            if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) < 1:
                return

            # 리셋 스위치 확인
            reset_val = cv2.getTrackbarPos("Reset Params", self.window_name)
            if reset_val == 1:
                self.reset_params()
                return

            self.params.debug_view = cv2.getTrackbarPos("Debug View", self.window_name)
            self.params.h_min = cv2.getTrackbarPos("H Min", self.window_name)
            self.params.h_max = cv2.getTrackbarPos("H Max", self.window_name)
            self.params.s_min = cv2.getTrackbarPos("S Min", self.window_name)
            self.params.s_max = cv2.getTrackbarPos("S Max", self.window_name)
            self.params.v_min = cv2.getTrackbarPos("V Min", self.window_name)
            self.params.v_max = cv2.getTrackbarPos("V Max", self.window_name)
            self.params.roi_height_percent = cv2.getTrackbarPos("ROI Height %", self.window_name)
            self.params.warp_offset = cv2.getTrackbarPos("Warp Offset", self.window_name)
            self.params.sw_margin = cv2.getTrackbarPos("SW Margin", self.window_name)
            self.params.sw_minpix = cv2.getTrackbarPos("SW MinPix", self.window_name)
            
            self.pub_params.publish(self.params)
        except cv2.error:
            pass

    def show_images(self):
        # 캔버스 생성
        # 높이: 950, 너비: 500
        canvas_h = 950
        canvas_w = 500
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        
        # 디스플레이용 이미지 크기 정의 (예: 320x240)
        disp_w, disp_h = 320, 240
        
        # 이미지 배치 및 리사이징 헬퍼 함수
        def place_image(img, x, y, title):
            if img is not None:
                resized = cv2.resize(img, (disp_w, disp_h))
                canvas[y:y+disp_h, x:x+disp_w] = resized
            else:
                # 플레이스홀더 (이미지 없음)
                cv2.rectangle(canvas, (x, y), (x+disp_w, y+disp_h), (50, 50, 50), -1)
                
                # 디버그 뷰가 꺼져있으면 특정 메시지 표시
                msg = "No Signal"
                if self.params.debug_view == 0 and title != "Original Camera":
                    msg = "Debug OFF"
                    
                cv2.putText(canvas, msg, (x+100, y+120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            
            # 테두리 및 제목 그리기
            cv2.rectangle(canvas, (x, y), (x+disp_w, y+disp_h), (255, 255, 255), 2)
            cv2.putText(canvas, title, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 이미지 중앙 정렬
        x_left = 90
        y_start = 40
        gap = 30
        
        # 1. 원본 카메라 (항상 표시)
        place_image(self.img_camera, x_left, y_start, "Original Camera")
        
        # 2. 처리된 이미지 (디버그 뷰 켜짐 상태일 때만 표시)
        img_proc = self.img_debug if self.params.debug_view == 1 else None
        place_image(img_proc, x_left, y_start + disp_h + gap, "Processed (Lane Mask)")
        
        # 3. 시각화 이미지 (디버그 뷰 켜짐 상태일 때만 표시)
        img_vis = self.img_visual if self.params.debug_view == 1 else None
        place_image(img_vis, x_left, y_start + (disp_h + gap) * 2, "Final Visualization")
        
        # 데이터 패널 표시 (데이터가 있으면 항상 표시)
        y_text_start = y_start + (disp_h + gap) * 3 + 20
        
        if self.current_lane_data is not None and self.current_lane_data.lane_detected:
            dx = self.current_lane_data.curve_radius
            dy = 240.0 
            if self.img_camera is not None:
                dy = self.img_camera.shape[0] / 2.0
            
            angle_rad = math.atan2(dx, dy)
            angle_deg = math.degrees(angle_rad)
            
            # 조향각 (큰 텍스트)
            text_angle = f"Steering: {angle_deg:.1f} deg"
            (tw, th), _ = cv2.getTextSize(text_angle, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            tx = (canvas_w - tw) // 2
            
            # 각도에 따른 색상 변경 (직진: 초록, 회전: 주황/빨강)
            color = (0, 255, 0)
            if abs(angle_deg) > 10: color = (0, 165, 255)
            if abs(angle_deg) > 20: color = (0, 0, 255)
            
            cv2.putText(canvas, text_angle, (tx, y_text_start), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
            
            # 추가 데이터 (작은 텍스트)
            y_data = y_text_start + 50
            text_center = f"Center Offset: {self.current_lane_data.lane_center:.2f}"
            text_curve = f"Curve Radius: {self.current_lane_data.curve_radius:.1f}"
            
            cv2.putText(canvas, text_center, (50, y_data), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            cv2.putText(canvas, text_curve, (50, y_data + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            
        else:
            text = "NO LANE DETECTED"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.2, 3)
            tx = (canvas_w - tw) // 2
            cv2.putText(canvas, text, (tx, y_text_start), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        
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