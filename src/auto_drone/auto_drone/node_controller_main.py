import rclpy
import time
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from mavros_msgs.srv import (
    CommandBool,
    CommandTOL,
    SetMode,
    CommandLong
)

qos_profile = QoSProfile(
reliability=ReliabilityPolicy.BEST_EFFORT,
durability=DurabilityPolicy.VOLATILE,
history=HistoryPolicy.KEEP_LAST,
depth=1
)
stablizing_time = 5
mode_response_time = 10
alt_threshold = 0.5
land_threshold = 0.0
max_count = 240

class MainServer(Node):
    def __init__(self):
        super().__init__('main_server')
        self.takeoff_service = self.create_service(SetBool, 'server/takeoff', self.get_takeoff)
        self.drive_service = self.create_service(SetBool, 'server/drive', self.get_drive)
        self.drive_auto_service = self.create_service(SetBool, 'server/drive_auto', self.get_drive_auto)
        self.land_service = self.create_service(SetBool, 'server/land', self.get_land)
        self.cmd_state = 0

    def get_takeoff(self, request, response):
        if request.data:
            self.get_logger().info('received takeoff command')
            response.success = True
            self.cmd_state = 1
        return response

    def get_drive(self, request, response):
        if request.data:
            self.get_logger().info('received drive command')
            response.success = True
            self.cmd_state = 2
        return response

    def get_drive_auto(self, request, response):
        if request.data:
            self.get_logger().info('received drive auto command')
            response.success = True
            self.cmd_state = 3
        return response

    def get_land(self, request, response):
        if request.data:
            self.get_logger().info('received land command')
            response.success = True
            self.cmd_state = 4
        return response



class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')                                                          
        self.client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff node : service not available, waiting again...')
        self.request = CommandTOL.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.future = False

    def send_request(self):
        self.future = self.client.call_async(self.request)
        if self.future:
            self.get_logger().info('takeoffing..')
        else:
            self.get_logger().info('takeoff request fail..')



class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')                                        
        self.client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm node : service not available, waiting again...')
        self.request = CommandBool.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.arming = False

    def send_request(self):
        self.request.value = self.arming
        self.future = self.client.call_async(self.request)


        
class LandNode(Node):
    def __init__(self):
        super().__init__('land_node')                                                             
        self.client = self.create_client(CommandTOL, '/mavros/cmd/land')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('land node : service not available, waiting again...')
        self.request = CommandTOL.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.future = False

    def send_request(self):
        self.future = self.client.call_async(self.request)
        if self.future:
            self.get_logger().info('landing..')
        else:
            self.get_logger().info('landing request fail..')       



class DriveSwitchNode(Node):
    def __init__(self):
        super().__init__('driveswitch_node')                                                             
        self.client = self.create_client(SetBool, 'vel_data/switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('drive switch node : service not available, waiting again...')
        self.request = SetBool.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.switch = False

    def send_request(self):
        future = self.client.call_async(self.request)
        if future:
            self.get_logger().info('switching..')
            self.switch = not self.switch
        else:
            self.get_logger().info('switching request fail..')



class StateListener(Node):
    def __init__(self):
        super().__init__('state_listener')
        self.state_subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.get_state,
            10)
        self.alt_subscription = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.get_alt,
            qos_profile)
        self.msg_armed = False
        self.msg_mode = 'AUTO.LOITER'
        self.alt = 0.

    def get_state(self, msg):
        self.msg_armed = msg.armed
        self.msg_mode = msg.mode

    def get_alt(self, msg):
        self.alt = msg.data


class ModeChanger(Node):
    def __init__(self):
        super().__init__('mode_changer')
        # `set_mode` 서비스 클라이언트 생성
        self.client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode service not available, waiting again...')
        self.req = SetMode.Request()
        self.future = False

    def send_mode_change_request(self, custom_mode_name):
        self.req.custom_mode = custom_mode_name
        self.future = self.client.call_async(self.req)
        if self.future:
            self.get_logger().info(f'mode change : {self.req.custom_mode}')


def main(args=None):
    '''
    while문 두고 takeoff, offboard, land 조건에 따라 실행
    각 실행 단위는 노드 단위로 구분할것
    - ready : 시작 상태.
    - takeoff : takeoff 노드로부터 신호를 받고 고도에 도달하면 완료 신호 날리기
    - drive : on 일 때 vel 메시지 기반으로 운전. off 일 때 vel = 0
    - land : 
    '''
    # init
    rclpy.init(args=args)
    main_server = MainServer()
    takeoff_node = TakeoffNode()
    arm_node = ArmNode()
    land_node = LandNode()
    driveswitch_node = DriveSwitchNode()
    state_listener = StateListener()
    mode_changer = ModeChanger()

    # state : loiter(0), takeoff(1), drive(2), land(3)
    while True:
        rclpy.spin_once(main_server)
        # takeoff
        if main_server.cmd_state == 1:
            while not takeoff_node.future:
                rclpy.spin_once(takeoff_node)
            takeoff_node.future = False
            while state_listener.alt < alt_threshold:
                arm_node.arming = True
                rclpy.spin_once(arm_node)
                rclpy.spin_once(state_listener)
            takeoff_node.get_logger().info('takeoff stablizing..')
            time.sleep(stablizing_time)
            takeoff_node.get_logger().info('takeoff done!\n')
        # drive
        elif main_server.cmd_state == 2:
            mode_changer.send_mode_change_request('OFFBOARD')
            mode_changer.get_logger().info('wait for state response..')
            time.sleep(mode_response_time)
            rclpy.spin_once(state_listener)
            # if loiter mode or no velocity topic, not switching to drive mode
            if state_listener.msg_mode == 'AUTO.LOITER':
                mode_changer.send_mode_change_request('OFFBOARD')
                mode_changer.get_logger().warn(f'no velocity topic!')
            elif not state_listener.msg_armed:
                state_listener.get_logger().warn(f'not armed!')
            # drive mode
            else:
                while not driveswitch_node.switch:
                    rclpy.spin_once(driveswitch_node)
                count = 0
                while True:
                    count += 1
                    state_listener.get_logger().info(f'moving count : {count}')
                    if count >= max_count:
                        while driveswitch_node.switch:
                            rclpy.spin_once(driveswitch_node)
                        break
                    time.sleep(0.1)
            time.sleep(stablizing_time)

            count = 0
            mode_changer.send_mode_change_request('AUTO.LOITER')
            while True:
                rclpy.spin_once(state_listener)
                if state_listener.msg_mode == 'AUTO.LOITER':
                    break
        # drive auto
        elif main_server.cmd_state == 3:
            mode_changer.send_mode_change_request('OFFBOARD')
            mode_changer.get_logger().info('wait for state response..')
            time.sleep(mode_response_time)
            rclpy.spin_once(state_listener)
            # if loiter mode or no velocity topic, not switching to drive mode
            if state_listener.msg_mode == 'AUTO.LOITER':
                mode_changer.send_mode_change_request('OFFBOARD')
                mode_changer.get_logger().warn(f'no velocity topic!')
            elif not state_listener.msg_armed:
                state_listener.get_logger().warn(f'not armed!')
            # drive mode
            else:
                while not driveswitch_node.switch:
                    rclpy.spin_once(driveswitch_node)
                count = 0
                while count < 800:
                    count += 1
                    time.sleep(0.1)
                while driveswitch_node.switch:
                    rclpy.spin_once(driveswitch_node)
            time.sleep(stablizing_time)
            mode_changer.send_mode_change_request('AUTO.LOITER')
            while True:
                rclpy.spin_once(state_listener)
                if state_listener.msg_mode == 'AUTO.LOITER':
                    break
        # land
        elif main_server.cmd_state == 4:
            while not land_node.future:
                rclpy.spin_once(land_node)
            land_node.future = False
            while state_listener.alt > land_threshold:
                arm_node.arming = True
                rclpy.spin_once(arm_node)
                rclpy.spin_once(state_listener)
            land_node.get_logger().info('landing stablizing..')
            time.sleep(stablizing_time)
            mode_changer.send_mode_change_request('AUTO.LOITER')
            land_node.get_logger().info('landing done!\n')

        main_server.cmd_state = 0

    # destroy
    main_server.destroy_node()
    takeoff_node.destroy_node()
    arm_node.destroy_node()
    land_node.destroy_node()
    driveswitch_node.destroy_node()
    state_listener.destroy_node()
    mode_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()