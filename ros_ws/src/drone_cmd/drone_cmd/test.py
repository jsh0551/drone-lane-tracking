import rclpy
from rclpy.node import Node
from mavros_msgs.srv import (
    CommandBool,
    CommandTOL,
    CommandTOLLocal,
    SetMode
)
from mavros_msgs.msg import State
import time
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')                                        
        self.client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm node : service not available, waiting again...')
        self.request = CommandBool.Request()

    def send_request(self, arm_state):
        self.request.value = arm_state
        self.future = self.client.call_async(self.request)

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')                                                             # ROS2节点父类初始化
        self.client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff node : service not available, waiting again...')
        self.request = CommandTOL.Request()

    def send_request(self, data):
        # self.request.min_pitch = data['min_pitch']
        self.request.yaw = data['yaw']
        # self.request.latitude = data['latitude']
        # self.request.longitude = data['longitude']
        self.request.altitude = data['altitude']
        self.future = self.client.call_async(self.request)

class TakeoffLocalNode(Node):
    def __init__(self):
        super().__init__('takeoff_local_node')                                                             # ROS2节点父类初始化
        self.client = self.create_client(CommandTOLLocal, '/mavros/cmd/takeoff_local')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff node : service not available, waiting again...')
        self.request = CommandTOLLocal.Request()

    def send_request(self, data):
        self.request.min_pitch = data['min_pitch']
        self.request.yaw = data['yaw']
        self.request.rate = data['rate']
        self.request.position.x = data['x']
        self.request.position.y = data['y']
        self.request.position.z = data['z']
        self.future = self.client.call_async(self.request)

class LandNode(Node):
    def __init__(self):
        super().__init__('land_node')                                                             # ROS2节点父类初始化
        self.client = self.create_client(CommandTOL, '/mavros/cmd/land')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('land node : service not available, waiting again...')
        self.request = CommandTOL.Request()

    def send_request(self, data):
        self.request.min_pitch = data['min_pitch']
        self.request.yaw = data['yaw']
        self.request.latitude = data['latitude']
        self.request.longitude = data['longitude']
        self.request.altitude = data['altitude']
        self.future = self.client.call_async(self.request)

class StateListener(Node):
    def __init__(self):
        super().__init__('state_listener')
        self.subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_armed,
            10)
        self.armed = False
        self.flag = 0
    def state_armed(self, msg):
        msg_armed = msg.armed
        self.flag = 0
        if msg_armed != self.armed:
            self.get_logger().info('armed state changed : "%s"' % msg.armed)
            self.armed = msg_armed
            self.flag = 1

class ModeChanger(Node):
    def __init__(self):
        super().__init__('mode_changer')
        # `set_mode` 서비스 클라이언트 생성
        self.client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        self.req = SetMode.Request()

    def send_mode_change_request(self, custom_mode_name):
        self.req.custom_mode = custom_mode_name
        self.future = self.client.call_async(self.req)

class DroneControl(Node):
    def __init__(self):
        super().__init__('drone_control')
        # '/mavros/setpoint_velocity/cmd_vel_unstamped' 토픽으로 퍼블리셔 생성
        self.publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        # 1초마다 publish_twist 메서드를 호출하는 타이머 생성
        self.timer = self.create_timer(1.0, self.publish_twist)

    def publish_twist(self):
        msg = Twist()
        # X축으로 1 m/s 속도로 전진 설정
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def request_to_server(node, req):
    node.send_request(req)
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info('Service call failed %r' % (e,))
            else:
                node.get_logger().info('Success: %s' % response.success)
                return response.success
            break
    return False

def main(args=None):
    land_data = {'min_pitch': 0., 'yaw': 0., 'latitude': 0., 'longitude': 0., 'altitude': 0.}
    takeoff_data = {'min_pitch': 10., 'yaw': 10., 'latitude': 8., 'longitude': 47., 'altitude': 550.}
    takeoff_local_data = {'min_pitch': 0., 'rate': 0.5, 'yaw': 0., 'x':0., 'y':0., 'z':3.}

    rclpy.init(args=args) 
    arm_node = ArmNode()
    takeoff_node = TakeoffNode()
    takeoff_local_node = TakeoffLocalNode()
    land_node = LandNode()
    state_listener = StateListener()
    drone_control = DroneControl()

    # takeoff
    request_to_server(takeoff_node, takeoff_data)
    # request_to_server(takeoff_local_node, takeoff_local_data)
    # arming

    request_to_server(arm_node, True)
    arm_count = 0

    while True:
        rclpy.spin_once(state_listener)
        if state_listener.flag == 1:
            arm_count += 1
            if arm_count >= 2:
                # request_to_server(takeoff_node, takeoff_data)
                request_to_server(arm_node, True)
                break
    time.sleep(10)
    request_to_server(takeoff_local_node, takeoff_local_data)
    request_to_server(arm_node, True)
    # mode_changer.send_mode_change_request(offb_set_mode)
    # time.sleep(15)
    # while rclpy.ok():
    #     rclpy.spin_once(mode_changer)
    #     if mode_changer.future.done():
    #         try:
    #             response = mode_changer.future.result()
    #             if response.mode_sent:
    #                 mode_changer.get_logger().info('Mode change successful')
    #             else:
    #                 mode_changer.get_logger().info('Mode change failed')
    #         except Exception as e:
    #             mode_changer.get_logger().info('Service call failed %r' % (e,))
    #         break
    time.sleep(5)
    # rclpy.spin(drone_control)
    # time.sleep(15)  # 직진 실행 시간
    # landing
    request_to_server(land_node, land_data)

    arm_node.destroy_node()
    takeoff_local_node.destroy_node()
    takeoff_node.destroy_node()
    land_node.destroy_node()
    state_listener.destroy_node()
    drone_control.destroy_node()
    rclpy.shutdown()    