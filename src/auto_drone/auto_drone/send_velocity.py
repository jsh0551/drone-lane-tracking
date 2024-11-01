from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import numpy as np
import os
import sys
from tf_transformations import quaternion_from_euler, euler_from_quaternion
BASE = os.getcwd()
sys.path.append(os.path.join(BASE))
from tools import *
from config import cfg

period = cfg.CONTROL.PERIOD
count = cfg.CONTROL.STEP
vel = cfg.CONTROL.VELOCITY_RULE
qos_profile = cfg.qos_profile


class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.arm_service = self.create_service(SetBool, 'vel_data/arming_switch', self.arm_switch)
        self.prep_service = self.create_service(SetBool, 'vel_data/preparation_switch', self.driveprep_switch)
        self.service = self.create_service(SetBool, 'vel_data/drive_switch', self.drive_switch)
        self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.finish_publisher = self.create_publisher(Bool, 'vel_data/finish', 10)
        self.state_subscription = self.create_subscription(Altitude, '/mavros/altitude', self.get_altitude, qos_profile)
        self.pos_subscription = self.create_subscription(PoseStamped,'/mavros/local_position/pose', self.get_pos, qos_profile)
        self.subscriber_data = self.create_subscription(Imu, '/mavros/imu/data', self.get_data, qos_profile)
        self.state_subscription = self.create_subscription(
            State, '/mavros/state', self.get_state, qos_profile)
        self.timer = self.create_timer(period, self.timer_callback)
        self.pose = TwistStamped()
        self.local_pos = PoseStamped()
        self.finish = Bool()
        self.quaternion = Quaternion()
        self.step = 0
        self.flag = False
        self.prep_flag = False
        self.arm_flag = False
        self.get_logger().info(f'wait for start signal..')
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.angle = 0.0
        self.finish.data = False


    def arm_switch(self, request, response):
        self.arm_flag = not self.arm_flag
        self.get_logger().info(f'switch to drive preparation : {self.prep_flag}')
        response.success = True
        if not self.arm_flag:
            self.get_logger().info(f'wait for takeoff signal..')
        # initialize
        self.finish.data = False
        self.prev_error = 0
        self.moved = 0
        return response
    
    def driveprep_switch(self, request, response):
        self.prep_flag = not self.prep_flag
        self.get_logger().info(f'switch to drive preparation : {self.prep_flag}')
        response.success = True
        if not self.prep_flag:
            self.get_logger().warning(f'wait for preparation signal..')
        # initialize
        self.finish.data = False
        self.prev_error = 0
        self.moved = 0
        return response
    
    def drive_switch(self, request, response):
        self.flag = not self.flag
        self.get_logger().info(f'switch to drive : {self.flag}')
        response.success = True
        if not self.flag:
            self.get_logger().info(f'wait for start signal..')
        # initialize
        self.finish.data = False
        self.prev_error = 0
        self.moved = 0
        return response

    def get_altitude(self, msg):
        self.altitude = msg.relative
        self.local_alt = msg.local

    def get_pos(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    def get_state(self, msg):
        self.drone_mode = msg.mode

    def get_data(self, data):
        self.quaternion.x = data.orientation.x
        self.quaternion.y = data.orientation.y
        self.quaternion.z = data.orientation.z
        self.quaternion.w = data.orientation.w
        roll, pitch, yaw = euler_from_quaternion((self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w))
        self.roll = roll
        self.pitch = -pitch
        self.yaw = yaw

    def timer_callback(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = ''
        if not self.arm_flag and not self.prep_flag and not self.flag:
            self.local_pos.header.stamp = self.get_clock().now().to_msg()
            self.local_pos.header.frame_id = ''
            gap = self.local_alt - self.altitude
            self.local_pos.pose.position.x = self.x
            self.local_pos.pose.position.y = self.y
            self.local_pos.pose.position.z = gap + 2.5
            qx,qy,qz,qw = quaternion_from_euler(0, 0, self.yaw)
            self.local_pos.pose.orientation.x = qx
            self.local_pos.pose.orientation.y = qy
            self.local_pos.pose.orientation.z = qz
            self.local_pos.pose.orientation.w = qw
        # 1. armed. hovering
        elif self.arm_flag and not self.flag:
            self.finish.data = False
            self.pos_publisher.publish(self.local_pos)
        # 2. drive
        elif self.arm_flag and self.prep_flag and self.flag:
            self.get_logger().info(f'step : {self.step}')
            if self.step < count:
                self.pose.twist.linear.x = min(float(self.step)/4, vel) 
                self.pose.twist.linear.y = 0.0
                self.pose.twist.linear.z = (2.5-self.local_alt)/4
            else:
                self.pose.twist.linear.x = -min(float(count*2-self.step)/4, vel)  
                self.pose.twist.linear.y = 0.0
                self.pose.twist.linear.z = (2.5-self.local_alt)/4
            self.step += 1
            if self.step >= count*2:
                self.step = 0
            self.publisher.publish(self.pose)
        t = self.get_clock().now().to_msg().sec
        nt = str(self.get_clock().now().to_msg().nanosec)
        self.get_logger().info(f't = {t}.{nt[:6]}')
        self.get_logger().info(f'altitude : {self.altitude}, local altitude : {self.local_alt}')
        self.get_logger().info(f'roll {round(np.degrees(self.roll),4)}, pitch : {round(np.degrees(self.pitch),4)}, yaw : {round(np.degrees(self.yaw),4)}')
        self.get_logger().info(f'x : {self.x}, y : {self.y}, z : {self.z}')
        self.finish_publisher.publish(self.finish)

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
