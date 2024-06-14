import rclpy
import sys
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

import numpy as np

from mavros_msgs.msg import Altitude, PositionTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Quaternion
from sensor_msgs.msg import Imu
from custom_message.msg import LandmarkCloud, TargetPolyInfo
from tf_transformations import euler_from_quaternion

BASE = os.getcwd()
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *

count = 40
vel = 1.0
kp = 0.6
ki = 0.1
kd = 0.05
period = 0.1
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)


def calculate_angle(polyline):
    mi = len(polyline)//2
    bx,by = polyline[-1]
    mx,my = polyline[0] # TODO change this in the future
    theta = np.arctan((mx-bx)/(my-by))
    return theta

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('node_velocity_control')
        # command to pixhawk
        self.service = self.create_service(SetBool, 'vel_data/switch', self.drive_switch)
        self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.timer = self.create_timer(period, self.timer_callback)
        # Subscribing position info
        self.subscriber_state = self.create_subscription(
            Altitude, '/mavros/altitude', self.get_altitude, qos_profile)
        self.subscriber_position = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.get_position, qos_profile)
        self.subscriber_data = self.create_subscription(
            Imu, '/mavros/imu/data', self.get_data, qos_profile)
        # Subscribing target polyline info
        self.subscriber_targetpoly = self.create_subscription(
            TargetPolyInfo, '/targetpoly_info', self.get_targetpoly, 10)
        
        # init value
        # self.get_logger().info(f'wait for start signal..')
        self.twist_stamped = TwistStamped()
        self.local_pos = PoseStamped()
        self.quaternion = Quaternion()
        self.roll, self.pitch, self.yaw = 0.0, np.radians(-30), 0.0
        self.flag = False
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.angle = 0.0 ## TODO get from current drone quaternion
        self.targetpoly = []
        self.lidarinfo = []

        # Ziegler-Nichols
        ku_lx = 2.5
        ku_ly = 2.5
        ku_az = 2.5
        tu_lx = 1.2
        tu_ly = 1.2
        tu_az = 1.2

        # PID Controller
        self.prev_error = 0
        self.integral = 0

        kp_lx = 0.6 * ku_lx
        ki_lx = 2 * kp_lx / tu_lx
        kd_lx = kp_lx * tu_lx / 8

        kp_ly = 0.6 * ku_ly
        ki_ly = 2 * kp_ly / tu_ly
        kd_ly = kp_ly * tu_ly / 8

        kp_az = 0.6 * ku_az
        ki_az = 2 * kp_az / tu_az
        kd_az = kp_az * tu_az / 8

        # self.linear_x_pid = PIDController(kp_lx, ki_lx, kd_lx)
        # self.linear_y_pid = PIDController(kp_ly, ki_ly, kd_ly)
        # self.angular_z_pid = PIDController(kp_az, ki_az, kd_az)
        self.linear_x_pid = PIDController(0.1, 0.001, 0.1)
        self.linear_y_pid = PIDController(0.1, 0.001, 0.1)
        self.angular_z_pid = PIDController(0.1, 0.001, 0.1)


    def drive_switch(self, request, response):
        self.flag = not self.flag
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

    def get_position(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    def get_data(self, data):
        self.quaternion.x = data.orientation.x
        self.quaternion.y = data.orientation.y
        self.quaternion.z = data.orientation.z
        self.quaternion.w = data.orientation.w
        roll, pitch, yaw = euler_from_quaternion((self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w))
        self.roll = roll
        self.pitch = -pitch + np.radians(-30)
        self.yaw = yaw
        # self.get_logger().info(f'roll : {self.roll}, pitch : {self.pitch}, yaw : {self.yaw}')


    def get_hovering_state(self, pos, yaw=-8.0):
        pos.header.stamp = self.get_clock().now().to_msg()
        pos.header.frame_id = ''
        gap = self.local_alt - self.altitude
        x,y,z,w = calculate_quaternion(yaw, 0.0, 0.0)
        pos.pose.orientation.x = x
        pos.pose.orientation.y = y
        pos.pose.orientation.z = z
        pos.pose.orientation.w = w
        # tmp_pos.pose.position.x = self.x
        # tmp_pos.pose.position.y = self.y
        pos.pose.position.z = gap + 2.5
        return pos


    def get_targetpoly(self, targetpoly_info):
        ## All the codes below are written with assumption that
        ## lanes are properly detected
        if targetpoly_info.clouds.points:
            targetpoly = targetpoly_info.clouds.points
            targetpoly = np.array([(data.x, data.y) for data in targetpoly])
            height = targetpoly_info.height
            width = targetpoly_info.width
            bot_point = targetpoly[-1]
            slope = calculate_slope(bot_point, self.pitch, width, height)
            self.targetpoly = affine_transform(targetpoly, slope)


    def get_lidarInfo(self, lidarInfo):
        pass

    
    def timer_callback(self):
        if self.flag:
            if len(self.targetpoly):
                theta = calculate_angle(self.targetpoly)
                if abs(np.degrees(theta))<5:
                    theta = 0.0
                self.angle += theta*period
                velocity = 10 # TODO
                velx = velocity*np.cos(self.angle)
                vely = velocity*np.sin(self.angle)
                gap = self.local_alt - self.altitude
                self.twist_stamped.twist.linear.x = velx
                self.twist_stamped.twist.linear.y = vely
                self.twist_stamped.twist.linear.z = (2.5-self.altitude)/5
                self.twist_stamped.twist.angular.x = 0.0
                self.twist_stamped.twist.angular.y = 0.0
                self.twist_stamped.twist.angular.z = theta
                ## Vision-based control code
                self.publisher.publish(self.twist_stamped)
                self.get_logger().info(f"angle : {np.degrees(self.angle)}, pitch : {np.degrees(self.pitch)}")
                self.get_logger().info("Following track with linear.x: {:.2f} m/s, linear.y: {:.2f} m/s, angular.z: {:.2f} degree/s"\
                                    .format(self.twist_stamped.twist.linear.x, self.twist_stamped.twist.linear.y, np.degrees(theta)))
        else:
            self.local_pos = self.get_hovering_state(self.local_pos)
            self.pos_publisher.publish(self.local_pos)
 

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
