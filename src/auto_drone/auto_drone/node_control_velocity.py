import rclpy
import sys
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

import pandas as pd
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

count = 110
kp = 0.6
ki = 0.1
kd = 0.05
period = 0.05
R = 48
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
        self.subscriber_vel = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.get_velocity, qos_profile)
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
        self.connect_flag = False
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.angle = 0.0 ## TODO get from current drone quaternion
        self.angle_flag = 0.0
        self.curve_angle = 0.0
        self.theta = 0.0
        self.h_error = 0.0
        self.integrated_error = 0.0
        self.track_num = 0
        self.targetpoly = []
        self.raw_targetpoly = []
        self.lidarinfo = []
        self.drone_data = dict()
        self.remain = R*np.pi
        self.count = 0
        self.ideal_angle = 0.0
        self.velocity = 0.0
        self.distance = 0.0

        # Ziegler-Nichols
        # PID Controller
        self.prev_error = 0
        self.integral = 0

        kp_dv, ki_dv, kd_dv = 4.0, 0.002, 4.0
        self.dvel_pid = PIDController(kp_dv, ki_dv, kd_dv)

        kp_dth, ki_dth, kd_dth = 2.0, 0.01, 0.1
        self.dtheta_pid = PIDController(kp_dth, ki_dth, kd_dth)


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

    def get_velocity(self, msg):
        x = msg.twist.linear.x 
        y = msg.twist.linear.y 
        self.distance += np.sqrt(x**2 + y**2)/4

    def get_data(self, data):
        # self.connect_flag = True
        self.quaternion.x = data.orientation.x
        self.quaternion.y = data.orientation.y
        self.quaternion.z = data.orientation.z
        self.quaternion.w = data.orientation.w
        roll, pitch, yaw = euler_from_quaternion((self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w))
        self.roll = roll
        self.pitch = -pitch + np.radians(-30)
        self.yaw = yaw
        # self.get_logger().info(f'roll : {self.roll}, pitch : {self.pitch}, yaw : {self.yaw}')


    def get_hovering_state(self, pos, yaw=-7.5):
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
            self.raw_targetpoly = targetpoly
            self.height = targetpoly_info.height
            self.width = targetpoly_info.width
            bot_point = targetpoly[-1]
            # error = gap * sin(pitch) * cos(roll)?
            prev_error = self.h_error
            # self.h_error = ((width/2 - bot_point[0])/width) / (2.5*np.sin(-self.pitch+np.radians(45))) # TODO : change altitude and FOV
            self.h_error = (self.width/2 - bot_point[0])/self.width
            self.get_logger().info(f'error:{self.h_error}, prev_error:{prev_error}, integrated : {self.integrated_error}, track num : {self.track_num}')
            if abs(self.h_error) < 0.02: # TODO adjust orin
                self.h_error = 0
            elif prev_error * self.h_error < 0:
                if prev_error > 0:
                    self.track_num += 1
                elif prev_error < 0:
                    self.track_num -= 1
                self.integrated_error = self.integrated_error + prev_error - self.h_error
                if self.track_num == 0:
                    self.integrated_error = 0.0
            slope = calculate_slope(bot_point, self.pitch, self.width, self.height)
            self.targetpoly = affine_transform(targetpoly, slope)


    def get_lidarInfo(self, lidarInfo):
        pass


    def save_droneInfo(self, k1, k2):
        if self.flag:
            if not self.drone_data:
                self.drone_data['x'] = [self.x]
                self.drone_data['y'] = [self.y]
                self.drone_data['z'] = [self.z]      
                self.drone_data['roll'] = [self.roll]
                self.drone_data['pitch'] = [self.pitch + np.radians(30)]
                self.drone_data['yaw'] = [self.yaw]
                self.drone_data['velx'] = [self.twist_stamped.twist.linear.x]
                self.drone_data['vely'] = [self.twist_stamped.twist.linear.y]
                self.drone_data['velz'] = [self.twist_stamped.twist.linear.z]
                self.drone_data['omega'] = [self.theta]
                self.drone_data['h_error'] = [self.h_error]
            else:
                self.drone_data['x'].append(self.x)
                self.drone_data['y'].append(self.y)
                self.drone_data['z'].append(self.z)      
                self.drone_data['roll'].append(self.roll)
                self.drone_data['pitch'].append(self.pitch + np.radians(30))
                self.drone_data['yaw'].append(self.yaw)
                self.drone_data['velx'].append(self.twist_stamped.twist.linear.x)
                self.drone_data['vely'].append(self.twist_stamped.twist.linear.y)
                self.drone_data['velz'].append(self.twist_stamped.twist.linear.z)
                self.drone_data['omega'].append(self.theta)
                self.drone_data['h_error'].append(self.h_error)
        else:
            if self.drone_data:
                os.makedirs('drone_data', exist_ok=True)
                df = pd.DataFrame(self.drone_data)
                file_name = f'drone_data/drone_data_modi_{k1}_{k2}.csv' # TODO
                file_name = get_new_filename(file_name)
                df.to_csv(file_name, index=False)
                self.drone_data = dict()


    def timer_callback(self):
        k1 = 1.5
        k2 = 3.0
        velocity = 8.0 # TODO
        # calculate error
        # if len(self.targetpoly):
        #     bot_point = self.raw_targetpoly[-1]
        #     # error = gap * sin(pitch) * cos(roll)?
        #     prev_error = self.h_error
        #     # self.h_error = ((width/2 - bot_point[0])/width) / (2.5*np.sin(-self.pitch+np.radians(45))) # TODO : change altitude and FOV
        #     self.h_error = (self.width/2 - bot_point[0])/self.width
        #     self.get_logger().info(f'error:{self.h_error}, prev_error:{prev_error}, integrated : {self.integrated_error}')
        #     if abs(self.h_error) < 0.02:
        #         self.h_error = 0
        #     elif prev_error * self.h_error < 0:
        #         if prev_error > 0:
        #             self.track_num += 1
        #         elif prev_error < 0:
        #             self.track_num -= 1
        #         self.integrated_error = self.integrated_error + prev_error - self.h_error
        #         if self.track_num == 0:
        #             self.integrated_error = 0.0

        if self.flag:
            if len(self.targetpoly):
                theta = calculate_angle(self.targetpoly)
                roll = self.roll
                if abs(np.degrees(roll)) < 0.1:
                    roll = 0.0
                theta += roll
                # apply horizontal error to theta
                error = self.integrated_error + self.h_error
                # dvel = k1*(error) # y-axis movement
                # dtheta = k2*theta*(error) # additional theta by error .. is needed?
                dvel = self.dvel_pid.compute(error)
                # dvel = dvel if abs(dvel) >= 0.02 else (0.02 if dvel >= 0 else -0.02)
                if abs(np.degrees(theta)) < 0.5 or self.angle_flag >= np.pi:
                    theta = 0.0
                dtheta = self.dtheta_pid.compute(error)*theta
                theta += dtheta
                self.theta = theta
                # calculate velocity
                # theta += dtheta/period
                self.angle += theta*period
                # self.angle_flag += theta*period
                omega = theta
                dx = -dvel*np.sin(self.angle)
                dy = dvel*np.cos(self.angle)
                velx = velocity*np.cos(self.angle)
                vely = velocity*np.sin(self.angle)
                self.twist_stamped.twist.linear.x = velx + dx
                self.twist_stamped.twist.linear.y = vely + dy
                self.twist_stamped.twist.linear.z = (2.5-self.local_alt)/5
                self.twist_stamped.twist.angular.x = 0.0
                self.twist_stamped.twist.angular.y = 0.0
                self.twist_stamped.twist.angular.z = omega
                ## Vision-based control code
                self.publisher.publish(self.twist_stamped)
                self.get_logger().info(f"distance : {self.distance},theta : {np.degrees(theta)}, angle : {np.degrees(self.angle)}")
                # self.get_logger().info(f"dvel : {dvel}")
                self.get_logger().info("Following track with linear.x: {:.2f} m/s, linear.y: {:.2f} m/s, angular.z: {:.2f} degree/s"\
                                    .format(self.twist_stamped.twist.linear.x, self.twist_stamped.twist.linear.y, np.degrees(theta)))
        else:
            if len(self.targetpoly):
                theta = calculate_angle(self.targetpoly)
                self.theta = theta
                self.track_num = 0
                self.integrated_error = 0.0
                if abs(np.degrees(theta)) < 0.1:
                    theta = 0.0
                vely = 0.04 if abs(self.h_error) >= 0.0005 else 0.0
                self.twist_stamped.twist.linear.x = 0.0
                self.twist_stamped.twist.linear.y = vely
                self.twist_stamped.twist.linear.z = 0.0
                self.twist_stamped.twist.angular.x = 0.0
                self.twist_stamped.twist.angular.y = 0.0
                self.twist_stamped.twist.angular.z = theta
                self.angle = self.yaw
                self.get_logger().info(f'x : {self.x}, y : {self.y}, z : {self.z}')
                # self.local_pos = self.get_hovering_state(self.local_pos)
                # self.pos_publisher.publish(self.local_pos)
                self.publisher.publish(self.twist_stamped)
        self.get_logger().info(f'roll {round(np.degrees(self.roll),5)},  pitch : {round(np.degrees(self.pitch)+30.0,5)},  yaw : {round(np.degrees(self.yaw),5)}')
        self.get_logger().info(f'horizontal error : {self.h_error}, integral error : {self.integrated_error}, track_num : {self.track_num}')
        self.save_droneInfo(k1,k2)

def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
