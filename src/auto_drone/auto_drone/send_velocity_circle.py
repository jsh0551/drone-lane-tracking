from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, State
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import numpy as np
import os
import sys
from tf_transformations import quaternion_from_euler, euler_from_quaternion
BASE = os.getcwd()
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *


count = 80
vel = 1.0
kp = 0.6
ki = 0.1
kd = 0.05
period = 0.1
qos_profile = QoSProfile(
reliability=ReliabilityPolicy.BEST_EFFORT,
durability=DurabilityPolicy.VOLATILE,
history=HistoryPolicy.KEEP_LAST,
depth=1
)


class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.service = self.create_service(SetBool, 'vel_data/switch', self.drive_switch)
        self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.finish_publisher = self.create_publisher(Bool, 'vel_data/finish', 10)
        self.state_subscription = self.create_subscription(Altitude, '/mavros/altitude', self.get_altitude, qos_profile)
        self.pos_subscription = self.create_subscription(PoseStamped,'/mavros/local_position/pose', self.get_pos, qos_profile)
        self.subscriber_data = self.create_subscription(Imu, '/mavros/imu/data', self.get_data, qos_profile)
        self.state_subscription = self.create_subscription(
            State, '/mavros/state', self.get_state, qos_profile)
        self.timer = self.create_timer(period, self.timer_callback)
        self.pose = TwistStamped()
        self.finish = Bool()
        self.quaternion = Quaternion()
        self.step = 0
        self.flag = False
        self.get_logger().info(f'wait for start signal..')
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.prev_error = 0
        self.integral = 0
        self.angle = 0.0
        self.finish.data = False

    def drive_switch(self, request, response):
        self.flag = not self.flag
        self.step = 0
        self.get_logger().info(f'switch to drive : {self.flag}')
        response.success = True
        if not self.flag:
            self.get_logger().info(f'wait for start signal..')
        self.integral = 0
        self.prev_error = 0
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
        # self.get_logger().info(f'pid value : {pid_value}')
        if self.flag:
            dtheta = 2*np.pi/(count*2)
            velx = vel*np.cos(self.angle)
            vely = vel*np.sin(self.angle)
            self.pose.twist.linear.x = velx
            self.pose.twist.linear.y = vely
            self.pose.twist.linear.z = 0.0
            self.pose.twist.angular.x = 0.0
            self.pose.twist.angular.y = 0.0
            self.pose.twist.angular.z = dtheta*period
            self.angle += dtheta
            if self.drone_mode == 'AUTO.LOITER':
                self.flag = False         
        else:
            self.pose.twist.linear.x = 0.0 
            self.pose.twist.linear.y = 0.0
            self.pose.twist.linear.z = 0.0
            self.pose.twist.angular.x = 0.0
            self.pose.twist.angular.y = 0.0
            self.pose.twist.angular.z = 0.0
        self.get_logger().info(f'altitude : {self.altitude}, local altitude : {self.local_alt}')
        self.get_logger().info(f'roll {round(np.degrees(self.roll),4)}, pitch : {round(np.degrees(self.pitch),4)}, yaw : {round(np.degrees(self.yaw),4)}')
        self.get_logger().info(f'x : {self.x}, y : {self.y}, z : {self.z}')
        self.publisher.publish(self.pose)
        self.finish_publisher.publish(self.finish)

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
