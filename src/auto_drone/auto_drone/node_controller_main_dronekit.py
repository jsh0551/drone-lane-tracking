import rclpy
import time
import cv2
import os
import datetime
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
TIMER = True
PERIOD = 0.05
WIDTH, HEIGHT = 480, 360
qos_profile = QoSProfile(
reliability=ReliabilityPolicy.BEST_EFFORT,
durability=DurabilityPolicy.VOLATILE,
history=HistoryPolicy.KEEP_LAST,
depth=1
)
stablizing_time = 5
termination_time = 0.1
mode_response_time = 10
alt_threshold = 1.6
land_threshold = 0.0

def get_current_time_string():
    now = datetime.datetime.now()
    time_str = now.strftime('%Y-%m-%d-%H%M%S')
    return time_str

class DroneControl:
    def __init__(self, address = '/dev/ttyUSB0:57600'):
        self.vehicle = connect(address, wait_ready=True)
        self.yaw = self.vehicle.attitude.yaw
        self.pitch = self.vehicle.attitude.pitch
        self.roll = self.vehicle.attitude.roll
        self.state = 0

    def mode_change(self, mode):
        while self.vehicle.mode.name != mode:
            self.vehicle.mode = VehicleMode(mode)
            time.sleep(PERIOD)

    def arm_and_takeoff(self, target_altitude):
        while not self.vehicle.is_armable:
            print(" 초기화를 기다리는 중...")
            time.sleep(1)

        # self.mode_change("GUIDED")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            # self.vehicle.armed = True
            print(" 아밍을 기다리는 중...")
            time.sleep(PERIOD)

        print("이륙!")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            print(f" 고도: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
                print("목표 고도에 도달!")
                break
            time.sleep(1)
        # self.mode_change("LOITER")
        self.vehicle.mode = VehicleMode("LOITER")
        time.sleep(stablizing_time)

    def land(self):
        # self.mode_change("LAND")
        self.vehicle.mode = VehicleMode("LAND")

        # 착륙 완료까지 대기
        while self.vehicle.armed:
            print(" 착륙 중...")
            time.sleep(1)

    def send_ned_velocity(self, vel_x, vel_y, vel_z, yaw_rate):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,       # 시간 부트 ms (시스템에서 무시됨)
                0, 0,    # 타겟 시스템, 컴포넌트 (0: 자신)
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # 좌표계
                0b010111000111,  # type_mask (위치 무시, 속도 사용, 가속도 무시, yaw 무시, yaw_rate 사용)
                0, 0, 0,  # x, y, z 위치 (무시됨)
                vel_x, vel_y, vel_z,  # x, y, z 속도 (NED)
                0, 0, 0,  # x, y, z 가속도 (무시됨)
                0, yaw_rate)
        self.vehicle.send_mavlink(msg)

class MainServer(Node):
    def __init__(self):
        super().__init__('main_server')
        self.takeoff_service = self.create_service(SetBool, 'server/takeoff', self.get_takeoff)
        self.drive_service = self.create_service(SetBool, 'server/drive', self.get_drive)
        self.drive_auto_service = self.create_service(SetBool, 'server/drive_auto', self.get_drive_auto)
        self.drive_termination_service = self.create_service(SetBool, 'server/drive_termination', self.get_drive_termination)
        self.land_service = self.create_service(SetBool, 'server/land', self.get_land)
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


class VelocityNode(Node):
    def __init__(self):
        super().__init__('velocity_node')
        self.subscriber_finish = self.create_subscription(
            TwistStamped, 'vel_data/velocity', self.sub_velocity, qos_profile)
        self.velx, self.vely, self.velz = 0.0, 0.0, 0.0
        self.wx, self.wy, self.wz = 0.0, 0.0, 0.0

    def sub_velocity(self, twist_stamped):
        self.velx = twist_stamped.twist.linear.x = 0.0 
        self.vely = twist_stamped.twist.linear.y = 0.0 
        self.velz = twist_stamped.twist.linear.z = 0.0 
        self.wx = twist_stamped.twist.angular.x = 0.0 
        self.wy = twist_stamped.twist.angular.y = 0.0 
        self.wz = twist_stamped.twist.angular.z = 0.0 


class FinishNode(Node):
    def __init__(self):
        super().__init__('finish_node')
        self.subscriber_finish = self.create_subscription(
            Bool, 'vel_data/finish', self.sub_finish, qos_profile)
        self.finish = False

    def sub_finish(self, data):
        self.finish = data.data  


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
            self.switch = True
        else:
            self.get_logger().info('switching request fail..')


class VideoSwitchNode(Node):
    def __init__(self):
        super().__init__('videoswitch_node')                                                             
        self.client = self.create_client(SetBool, 'video/switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('video switch node : service not available, waiting again...')
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
    dronecontrol = DroneControl()
    rclpy.init(args=args)
    main_server = MainServer()
    driveswitch_node = DriveSwitchNode()
    videoswitch_node = VideoSwitchNode()
    velocity_node = VelocityNode()
    finish_node = FinishNode()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    os.makedirs('drone_video', exist_ok=True)
    # state : loiter(0), takeoff(1), land(2), drive(etc)
    while True:
        rclpy.spin_once(main_server)
        # 0. no signal
        if main_server.cmd_state == 0:
            pass
        # 1. takeoff
        elif main_server.cmd_state == 1:
            dronecontrol.arm_and_takeoff(2)
            main_server.get_logger().info('takeoff done!\n')
        # 2. land
        elif main_server.cmd_state == 2:
            dronecontrol.land()
            main_server.get_logger().info('landing stablizing..')
            time.sleep(stablizing_time)
            main_server.get_logger().info('landing done!\n')
        # etc. drive
        else:
            # change to GUIDED, check arming.
            dronecontrol.vehicle.mode = "GUIDED"
            # receive velocity topic
            while not driveswitch_node.switch:
                rclpy.spin_once(driveswitch_node)
            while not videoswitch_node.switch:
                rclpy.spin_once(videoswitch_node)
            while True:
                altitude = dronecontrol.vehicle.location.global_relative_frame.alt
                velx = velocity_node.velx
                vely = velocity_node.vely
                velz = velocity_node.velz
                # velz = (2.0 - altitude)/2
                wz = velocity_node.wz
                dronecontrol.send_ned_velocity(velx, vely, velz, wz)
                if main_server.cmd_state == -1:
                    while not driveswitch_node.switch:
                        rclpy.spin_once(driveswitch_node)
                    while videoswitch_node.switch:
                        rclpy.spin_once(videoswitch_node)
                    break
                time.sleep(PERIOD)
            dronecontrol.vehicle.mode = "LOITER"
            time.sleep(stablizing_time)
            
        main_server.cmd_state = 0

    # destroy
    main_server.destroy_node()
    driveswitch_node.destroy_node()
    videoswitch_node.destroy_node()
    velocity_node.destroy_node()
    finish_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()