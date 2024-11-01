import rclpy
import time
import os
import datetime
import sys
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64, Bool
from mavros_msgs.msg import State
from mavros_msgs.srv import (
    CommandBool,
    CommandTOL,
    SetMode
)
BASE = os.getcwd()
sys.path.append(os.path.join(BASE))
from tools import *
from config import cfg
qos_profile = cfg.qos_profile
PERIOD = cfg.CONTROL.PERIOD
WIDTH, HEIGHT = cfg.WIDTH, cfg.HEIGHT
stablizing_time = cfg.CALIBRATION.stablizing_time
termination_time = cfg.CALIBRATION.termination_time
mode_response_time = cfg.CALIBRATION.mode_response_time
alt_threshold = cfg.CALIBRATION.alt_threshold
land_threshold = cfg.CALIBRATION.land_threshold

def get_current_time_string():
    now = datetime.datetime.now()
    time_str = now.strftime('%Y-%m-%d-%H%M%S')
    return time_str

def send_switch_signal(switch_node, signal):
    switch_node.set_data(signal)
    while True:
        switch_node.get_logger().info('send test')
        rclpy.spin_once(switch_node)
        if switch_node.success == True:
            break

class MainServer(Node):
    def __init__(self):
        super().__init__('main_server')
        self.takeoff_service = self.create_service(SetBool, '/server/takeoff', self.get_takeoff)
        self.drive_service = self.create_service(SetBool, '/server/drive', self.get_drive)
        self.drive_auto_service = self.create_service(SetBool, '/server/drive_auto', self.get_drive_auto)
        self.drive_termination_service = self.create_service(SetBool, '/server/drive_termination', self.get_drive_termination)
        self.land_service = self.create_service(SetBool, '/server/land', self.get_land)
        self.subscriber_finish = self.create_subscription(
            Bool, '/vel_data/finish', self.sub_finish, 10)
        self.finish = False
        self.cmd_state = 0

    def get_takeoff(self, request, response):
        if request.data:
            self.get_logger().info('received takeoff command')
            response.success = True
            self.cmd_state = 1
        return response

    def get_land(self, request, response):
        if request.data:
            self.get_logger().info('received land command')
            response.success = True
            self.cmd_state = 2
        return response

    def get_drive(self, request, response):
        if request.data:
            self.get_logger().info('received drive command')
            response.success = True
            self.cmd_state = 3
        return response

    def get_drive_auto(self, request, response):
        if request.data:
            self.get_logger().info('received drive auto command')
            response.success = True
            self.cmd_state = 4
        return response

    def get_drive_termination(self, request, response):
        if request.data:
            self.get_logger().info('received drive termination command')
            response.success = True
            self.cmd_state = -1
        return response
    
    def sub_finish(self, data):
        self.finish = data.data


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
        # self.request.altitude = alt_threshold
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


class ArmingSwitchNode(Node):
    def __init__(self):
        super().__init__('armswitch_node')                                                             
        self.client = self.create_client(SetBool, '/vel_data/arming_switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm switch node : service not available, waiting again...')
        self.request = SetBool.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.switch = False
        self.success = False

    def send_request(self):
        self.request.data = self.switch
        future = self.client.call_async(self.request)
        if future:
            self.get_logger().info('switching..')
            self.success = True
        else:
            self.get_logger().info('switching request fail..')

    def set_data(self, signal):
        self.switch = signal
        self.success = False

class DrivePreparationSwitch(Node):
    def __init__(self):
        super().__init__('driveprepswitch_node')                                                             
        self.client = self.create_client(SetBool, '/vel_data/preparation_switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('drive preparation switch node : service not available, waiting again...')
        self.request = SetBool.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.switch = False
        self.success = False

    def send_request(self):
        self.request.data = self.switch
        future = self.client.call_async(self.request)
        if future:
            self.get_logger().info('switching..')
            self.success = True
        else:
            self.get_logger().info('switching request fail..')
    
    def set_data(self, signal):
        self.switch = signal
        self.success = False


class DriveSwitchNode(Node):
    def __init__(self):
        super().__init__('driveswitch_node')                                                             
        self.client = self.create_client(SetBool, '/vel_data/drive_switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('drive switch node : service not available, waiting again...')
        self.request = SetBool.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.switch = False
        self.success = False

    def send_request(self):
        self.request.data = self.switch
        future = self.client.call_async(self.request)
        if future:
            self.get_logger().info('switching..')
            self.success = True
        else:
            self.get_logger().info('switching request fail..')
    
    def set_data(self, signal):
        self.switch = signal
        self.success = False


class VideoSwitchNode(Node):
    def __init__(self):
        super().__init__('videoswitch_node')                                                             
        self.client = self.create_client(SetBool, '/video/switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('video switch node : service not available, waiting again...')
        self.request = SetBool.Request()
        self.timer = self.create_timer(0.1, self.send_request)
        self.switch = False
        self.success = False

    def send_request(self):
        self.request.data = self.switch
        future = self.client.call_async(self.request)
        if future:
            self.get_logger().info('switching..')
            self.success = False
        else:
            self.get_logger().info('switching request fail..')
    
    def set_data(self, signal):
        self.switch = signal
        self.success = False


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
    armswitch_node = ArmingSwitchNode()
    driveprepswitch_node = DrivePreparationSwitch()
    driveswitch_node = DriveSwitchNode()
    videoswitch_node = VideoSwitchNode()
    state_listener = StateListener()
    mode_changer = ModeChanger()

    os.makedirs('drone_video', exist_ok=True)
    # state : loiter(0), takeoff(1), land(2), drive(etc)
    while True:
        rclpy.spin_once(main_server)
        # 0. no signal
        if main_server.cmd_state == 0:
            pass
        # 1. takeoff
        elif main_server.cmd_state == 1:
            while not takeoff_node.future:
                rclpy.spin_once(takeoff_node)
            takeoff_node.future = False
            while state_listener.alt < alt_threshold:
                arm_node.arming = True
                rclpy.spin_once(arm_node)
                rclpy.spin_once(state_listener)
            send_switch_signal(armswitch_node, True)
            time.sleep(stablizing_time)
            while state_listener.msg_mode != 'OFFBOARD':
                mode_changer.send_mode_change_request('OFFBOARD')
                mode_changer.get_logger().info('takeoff stablizing..')
                time.sleep(PERIOD)
                rclpy.spin_once(state_listener)
            # if loiter mode or no velocity topic, not switching to drive mode
            if state_listener.msg_mode == 'AUTO.LOITER':
                mode_changer.get_logger().warn(f'no velocity topic!')
                main_server.cmd_state = 0
                continue
            takeoff_node.get_logger().info('takeoff done!\n')
        # 2. land
        elif main_server.cmd_state == 2:
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
            send_switch_signal(armswitch_node, False)
            land_node.get_logger().info('landing done!\n')
        # etc. drive
        else:
            rclpy.spin_once(state_listener)
            # if loiter mode or no velocity topic, not switching to drive mode
            if state_listener.msg_mode == 'AUTO.LOITER':
                mode_changer.send_mode_change_request('OFFBOARD')
                time.sleep(mode_response_time)
                if state_listener.msg_mode == 'AUTO.LOITER':
                    mode_changer.get_logger().warn(f'no velocity topic!')
                    main_server.cmd_state = 0
                    continue
            if not state_listener.msg_armed:
                state_listener.get_logger().warn(f'not armed!')
                main_server.cmd_state = 0
                continue
            send_switch_signal(driveprepswitch_node, True)
            time.sleep(stablizing_time+3)
            # 3. select rulebase drive mode. record video
            if main_server.cmd_state == 3:
                # switch to drive mode
                send_switch_signal(videoswitch_node, True)
                send_switch_signal(driveswitch_node, True)
                # drive
                while not main_server.finish:
                    rclpy.spin_once(main_server)
                    # time.sleep(PERIOD)
                    if main_server.cmd_state == -1 or main_server.finish == True:
                        break
                # turn off switch
                send_switch_signal(videoswitch_node, False)
                send_switch_signal(driveswitch_node, False)
            # 4. select drive mode. not record video
            elif main_server.cmd_state == 4:
                # switch to drive mode
                send_switch_signal(driveswitch_node, True)
                # drive
                while not main_server.finish:
                    rclpy.spin_once(main_server)
                    # time.sleep(PERIOD)
                    if main_server.cmd_state == -1 or main_server.finish == True:
                        break
                # turn off switch
                send_switch_signal(driveswitch_node, False)
            send_switch_signal(driveprepswitch_node, False)
            if main_server.cmd_state >= 0:
                time.sleep(stablizing_time)
            else:
                time.sleep(termination_time)

        main_server.cmd_state = 0


    # destroy
    main_server.destroy_node()
    takeoff_node.destroy_node()
    arm_node.destroy_node()
    land_node.destroy_node()
    armswitch_node.destroy_node()
    driveprepswitch_node.destroy_node()
    driveswitch_node.destroy_node()
    videoswitch_node.destroy_node()
    state_listener.destroy_node()
    mode_changer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()