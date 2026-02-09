import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lane_msgs.msg import LaneParams
from cv_bridge import CvBridge
import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QSlider, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap
from threading import Thread

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        
        # Subscribers
        self.sub_camera = self.create_subscription(Image, '/camera_open', self.camera_callback, 10)
        self.sub_debug = self.create_subscription(Image, '/lane_debug_img', self.debug_callback, 10)
        self.sub_visual = self.create_subscription(Image, '/lane_visual', self.visual_callback, 10)
        
        # Publisher (Parameter Control)
        self.pub_params = self.create_publisher(LaneParams, '/lane_params', 10)
        
        self.cv_bridge = CvBridge()
        
        # Image Data Holders
        self.img_camera = None
        self.img_debug = None
        self.img_visual = None
        
        # Default Params
        self.params = LaneParams()
        self.params.h_min = 0
        self.params.h_max = 179
        self.params.s_min = 0
        self.params.s_max = 255
        self.params.v_min = 0
        self.params.v_max = 255
        self.params.canny_low = 50
        self.params.canny_high = 150
        self.params.roi_height_percent = 60
        self.params.warp_offset = 50

    def camera_callback(self, msg):
        try:
            self.img_camera = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def debug_callback(self, msg):
        try:
            self.img_debug = self.cv_bridge.imgmsg_to_cv2(msg, "mono8")
        except Exception:
            pass

    def visual_callback(self, msg):
        try:
            self.img_visual = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass
            
    def publish_params(self):
        self.pub_params.publish(self.params)


class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Lane Detection Dashboard")
        self.setGeometry(100, 100, 1200, 800)
        
        # Main Layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left Side (Images)
        left_layout = QVBoxLayout()
        
        self.label_camera = QLabel("Camera Input")
        self.label_camera.setAlignment(Qt.AlignCenter)
        self.label_camera.setMinimumSize(320, 240)
        self.label_camera.setStyleSheet("border: 1px solid black; background-color: #ddd;")
        
        self.label_debug = QLabel("Processed (Bird's Eye)")
        self.label_debug.setAlignment(Qt.AlignCenter)
        self.label_debug.setMinimumSize(320, 240)
        self.label_debug.setStyleSheet("border: 1px solid black; background-color: #ddd;")
        
        self.label_visual = QLabel("Visualized Output")
        self.label_visual.setAlignment(Qt.AlignCenter)
        self.label_visual.setMinimumSize(320, 240)
        self.label_visual.setStyleSheet("border: 1px solid black; background-color: #ddd;")
        
        left_layout.addWidget(QLabel("<b>Original Camera</b>"))
        left_layout.addWidget(self.label_camera)
        left_layout.addWidget(QLabel("<b>Processed (Lane Mask)</b>"))
        left_layout.addWidget(self.label_debug)
        left_layout.addWidget(QLabel("<b>Final Visualization</b>"))
        left_layout.addWidget(self.label_visual)
        
        # Right Side (Controls)
        right_layout = QVBoxLayout()
        
        # HSV Controls
        hsv_group = QGroupBox("HSV Thresholds")
        hsv_layout = QGridLayout()
        self.sliders = {}
        
        self.add_slider(hsv_layout, "H Min", 0, 179, 0, 0, "h_min")
        self.add_slider(hsv_layout, "H Max", 0, 179, 179, 1, "h_max")
        self.add_slider(hsv_layout, "S Min", 0, 255, 0, 2, "s_min")
        self.add_slider(hsv_layout, "S Max", 0, 255, 255, 3, "s_max")
        self.add_slider(hsv_layout, "V Min", 0, 255, 0, 4, "v_min")
        self.add_slider(hsv_layout, "V Max", 0, 255, 255, 5, "v_max")
        
        hsv_group.setLayout(hsv_layout)
        right_layout.addWidget(hsv_group)
        
        # Canny Controls
        canny_group = QGroupBox("Canny Edge")
        canny_layout = QGridLayout()
        self.add_slider(canny_layout, "Low", 0, 255, 50, 0, "canny_low")
        self.add_slider(canny_layout, "High", 0, 255, 150, 1, "canny_high")
        canny_group.setLayout(canny_layout)
        right_layout.addWidget(canny_group)
        
        # Geometry Controls
        geo_group = QGroupBox("Geometry (ROI & Warp)")
        geo_layout = QGridLayout()
        self.add_slider(geo_layout, "ROI Height %", 0, 100, 60, 0, "roi_height_percent")
        self.add_slider(geo_layout, "Warp Offset", 0, 200, 50, 1, "warp_offset")
        geo_group.setLayout(geo_layout)
        right_layout.addWidget(geo_group)
        
        right_layout.addStretch()
        
        # Add layouts to main
        main_layout.addLayout(left_layout, 2) # Image area takes 2/3
        main_layout.addLayout(right_layout, 1) # Control area takes 1/3
        
        # Timer for updating GUI images
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_images)
        self.timer.start(30) # 30ms interval (~33 FPS)

    def add_slider(self, layout, label, min_val, max_val, init_val, row, param_name):
        lbl = QLabel(f"{label}: {init_val}")
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(init_val)
        
        # Callback with closure to capture specific slider/label
        def value_changed(val):
            lbl.setText(f"{label}: {val}")
            setattr(self.ros_node.params, param_name, val)
            self.ros_node.publish_params()
            
        slider.valueChanged.connect(value_changed)
        
        layout.addWidget(lbl, row, 0)
        layout.addWidget(slider, row, 1)
        
        self.sliders[param_name] = slider

    def update_images(self):
        # Update Camera Image
        if self.ros_node.img_camera is not None:
            self.display_image(self.ros_node.img_camera, self.label_camera)
            
        # Update Debug Image
        if self.ros_node.img_debug is not None:
            self.display_image(self.ros_node.img_debug, self.label_debug, is_mono=True)
            
        # Update Visual Image
        if self.ros_node.img_visual is not None:
            self.display_image(self.ros_node.img_visual, self.label_visual)

    def display_image(self, cv_img, label_widget, is_mono=False):
        h, w = cv_img.shape[:2]
        if is_mono:
            bytes_per_line = w
            q_img = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_Grayscale8)
        else:
            bytes_per_line = 3 * w
            # OpenCV is BGR, Qt is RGB
            rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            q_img = QImage(rgb_img.data, w, h, bytes_per_line, QImage.Format_RGB888)
            
        pixmap = QPixmap.fromImage(q_img)
        # Scale to fit label while keeping aspect ratio
        scaled_pixmap = pixmap.scaled(label_widget.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label_widget.setPixmap(scaled_pixmap)

def main(args=None):
    rclpy.init(args=args)
    
    # Create ROS Node
    gui_node = GUINode()
    
    # Run ROS node in a separate thread
    thread = Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    thread.start()
    
    # Create PyQt Application
    app = QApplication(sys.argv)
    window = MainWindow(gui_node)
    window.show()
    
    try:
        sys.exit(app.exec_())
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()