import rclpy
from rclpy.node import Node
from lane_msgs.msg import LaneParams

class ParamNode(Node):
    def __init__(self):
        super().__init__('param_node')
        
        # 파라미터 선언 및 기본값 설정
        self.declare_parameter('h_min', 0)
        self.declare_parameter('h_max', 179)
        self.declare_parameter('s_min', 0)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 0)
        self.declare_parameter('v_max', 255)
        
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        
        self.declare_parameter('roi_height_percent', 60)
        self.declare_parameter('warp_offset', 50)

        # 파라미터 메시지 발행자
        self.publisher_ = self.create_publisher(LaneParams, '/lane_params', 10)
        
        # 10Hz 주기로 파라미터 발행 (UI에서 변경된 값을 지속적으로 전송)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Param Node Started')

    def timer_callback(self):
        msg = LaneParams()
        
        # 현재 ROS 파라미터 값을 읽어서 메시지에 담음
        msg.h_min = self.get_parameter('h_min').value
        msg.h_max = self.get_parameter('h_max').value
        msg.s_min = self.get_parameter('s_min').value
        msg.s_max = self.get_parameter('s_max').value
        msg.v_min = self.get_parameter('v_min').value
        msg.v_max = self.get_parameter('v_max').value
        
        msg.canny_low = self.get_parameter('canny_low').value
        msg.canny_high = self.get_parameter('canny_high').value
        
        msg.roi_height_percent = self.get_parameter('roi_height_percent').value
        msg.warp_offset = self.get_parameter('warp_offset').value
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ParamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()