import rclpy
import sys
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

import pandas as pd
import numpy as np

from std_msgs.msg import Bool
from mavros_msgs.msg import Altitude, State
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from sensor_msgs.msg import Imu
from custom_message.msg import TargetPolyInfo
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64MultiArray
from rclpy.executors import MultiThreadedExecutor

BASE = os.getcwd()
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
PERIOD = 0.05
DEFAULT = [0.0, 0.0, -0.1, -0.1]
ROT_OFFSET = 10
TRACK_R = [37.898, 39.118, 40.338, 41.5,58, 42.778, 43.998, 45.218, 46.438]
TRACK_ANGLE = [0.0, 10.29766794, 20.85159044, 30.77199376, 40.11425805, 48.92749134, 57.25539304, 65.13697863]
TRACK_DISTANCE = [100, 200, 400, 800]
DRONE_NUM = 5
RUNNER_NUM = 0
EVENT = 0 # 0: 100m, 1:200m, 2:400m, 3: over 400m
HFOV = 110
ASPECT_RATIO = 4 / 3
CAM_TILT = -45.0

class DroneStateCollector(Node):
    def __init__(self):
        super().__init__('statecollector')
        # Subscribing position info
        self.subscriber_state = self.create_subscription(
            Altitude, '/mavros/altitude', self.get_altitude, qos_profile)
        self.subscriber_position = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.get_position, qos_profile)
        self.subscriber_vel = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.get_velocity, qos_profile)
        self.subscriber_data = self.create_subscription(
            Imu, '/mavros/imu/data', self.get_data, qos_profile)
        # Subscribing target polyline info
        self.subscriber_targetpoly = self.create_subscription(
            TargetPolyInfo, '/targetpoly_info', self.get_targetpoly, 10)
        
        self.collector_publisher = self.create_publisher(Float64MultiArray, 'vel_data/collected', 10)

        self.quaternion = Quaternion()
        self.collected = Float64MultiArray()
        self.vfov = 2 * np.arctan(np.tan(np.radians(HFOV / 2)) / ASPECT_RATIO)
        # init value
        self.roll, self.pitch, self.yaw = 0.0, np.radians(CAM_TILT), 0.0
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.error = 0.0
        self.accumulatedError = DEFAULT[EVENT]
        self.currentTrack = 0.
        self.targetpoly = []
        self.lidarinfo = []
        self.velocity = 0.0
        self.moved = 0.0
        self.numPoly = 0.
        self.polySlope = 0.0
        self.ox, self.oy, self.oz = 0.0, 0.0, 0.0


    def get_altitude(self, msg):
        self.altitude = msg.relative
        self.local_alt = msg.local

    def get_position(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z

    def get_velocity(self, msg):
        x = msg.twist.linear.x 
        y = msg.twist.linear.y 
        self.moved += np.sqrt(x**2 + y**2)/4

    def get_data(self, data):
        self.quaternion.x = data.orientation.x
        self.quaternion.y = data.orientation.y
        self.quaternion.z = data.orientation.z
        self.quaternion.w = data.orientation.w
        roll, pitch, yaw = euler_from_quaternion((self.quaternion.x,self.quaternion.y,self.quaternion.z,self.quaternion.w))
        self.roll = roll
        self.pitch = -pitch + np.radians(CAM_TILT)
        self.yaw = yaw if yaw >= 0 else 2*np.pi + yaw
        # self.yaw = yaw

    def get_lidarInfo(self, lidarInfo):
        pass

    def get_targetpoly(self, targetpoly_info):
        ## All the codes below are written with assumption that
        ## lanes are detected at least one
        if targetpoly_info.polyline.points:
            # get information
            targetpoly = targetpoly_info.polyline.points
            rpoly = targetpoly_info.rpoly.points
            lpoly = targetpoly_info.lpoly.points
            self.height = targetpoly_info.height
            self.width = targetpoly_info.width
            # preprocess
            targetpoly = np.array([(data.x, data.y) for data in targetpoly])
            rpoly = np.array([(data.x, data.y) for data in rpoly])
            lpoly = np.array([(data.x, data.y) for data in lpoly])
            bot_point = targetpoly[-1]
            bot_point_r = rpoly[-1]
            bot_point_l = lpoly[-1]
            # calcuate error
            factor = bot_point_r[0] - bot_point_l[0]
            prev_error = self.error
            self.error = (self.width/2 - bot_point[0])/factor
            # change track number if meet some conditions
            prev_gap = 1 - (0.5 + prev_error) if prev_error >= 0 else (0.5 + prev_error)
            h_gap = 1 - (0.5 + self.error) if self.error >= 0 else (0.5 + self.error)
            if (prev_error*self.error < 0) and (abs(self.error - prev_error) > 0.3) and ((prev_gap < 0.25) or (h_gap < 0.25) or True):
                if self.error < prev_error:
                    self.currentTrack += 1.0
                    self.accumulatedError += 1.
                elif self.error > prev_error:
                    self.currentTrack -= 1.0
                    self.accumulatedError += -1.
                if self.currentTrack == 0:
                    self.accumulatedError = DEFAULT[EVENT]
            self.get_logger().info(f'--prev_gap : {prev_gap}, h_gap : {h_gap}')
            self.get_logger().info(f'--prev_error : {prev_error:5f}, error : {self.error:5f}, integrated error : {self.accumulatedError:5f}')
            self.get_logger().info(f'--total error : {self.accumulatedError + self.error:5f}, track num : {self.currentTrack}')
            slope = calculate_slope(bot_point, self.pitch, self.width, self.height, fov=HFOV)
            self.targetpoly = affine_transform(targetpoly, slope)
            self.numPoly = len(self.targetpoly)
            self.polySlope = calculate_angle(self.targetpoly, -self.pitch, self.vfov)
        # roll, pitch, yaw,
        # altitude, local_alt, x, y, z,
        # error, accumError, curTrack, theta
        # velocity, moved, numPoly
        data = [self.roll, self.pitch, self.yaw, self.altitude, self.local_alt, self.x, self.y, self.z,\
                          self.error, self.accumulatedError, self.currentTrack, self.polySlope, self.velocity, self.moved, self.numPoly]
        data = np.array(data).astype(np.float64)
        self.collected.data = list(data)
        self.collector_publisher.publish(self.collected)
        self.get_logger().info(f'Received and published collected state')


class PositionPublisher(Node):
    def __init__(self):
        super().__init__('node_velocity_control')
        # command to pixhawk
        self.arm_service = self.create_service(SetBool, 'vel_data/arming_switch', self.arm_switch)
        self.prep_service = self.create_service(SetBool, 'vel_data/preparation_switch', self.driveprep_switch)
        self.service = self.create_service(SetBool, 'vel_data/drive_switch', self.drive_switch)
        self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.finish_publisher = self.create_publisher(Bool, 'vel_data/finish', 10)
        self.timer = self.create_timer(PERIOD, self.timer_callback)
        self.collected_subscription = self.create_subscription(Float64MultiArray, 'vel_data/collected', self.get_collected, 10)
        # topic message
        self.twist_stamped = TwistStamped()
        self.local_pos = PoseStamped()
        self.quaternion = Quaternion()
        self.finish = Bool()
        # init value
        self.roll, self.pitch, self.yaw = 0.0, np.radians(CAM_TILT), 0.0
        self.flag = False
        self.prep_flag = False
        self.arm_flag = False
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.globalAngle = 0.0 ## TODO get from current drone quaternion
        self.currentAngle = np.radians(ROT_OFFSET) + np.radians(TRACK_ANGLE[RUNNER_NUM]) # TODO
        self.theta = 0.0
        self.error = 0.0
        self.accumulatedError = DEFAULT[EVENT]
        self.currentTrack = 0
        self.targetpoly = []
        self.lidarinfo = []
        self.drone_data = dict()
        self.count = 0
        self.velocity = 0.0
        self.moved = 0.0
        self.dvel = 0.0
        self.is_curve = True
        self.finish.data = False
        self.polySlope = 0.0
        # theta coefficient
        self.vfov = 2 * np.arctan(np.tan(np.radians(HFOV / 2)) / ASPECT_RATIO)
        self.distance = TRACK_DISTANCE[EVENT]

        # Ziegler-Nichols
        # PID Controller
        self.prev_error = 0

        dvel_k_matrix = [[0.9, 0.005, 0.005], # 100m
                         [0.9, 0.005, 0.005], # 200m
                         [0,0,0], # 400m
                         [1.2, 0.2, 0.1]] # over 400m
        
        dtheta_k_matrix = [[0.03, 0.0001, 0.0001], # 100m
                         [0.015, 0.0005, 0.0005], # 200m
                         [0,0,0], # 400m
                         [0.02, 0.0001, 0.0001]] # over 400m

        self.kp_dv, self.ki_dv, self.kd_dv = dvel_k_matrix[EVENT]
        self.dvel_pid = PIDController(self.kp_dv, self.ki_dv, self.kd_dv, PERIOD)

        self.kp_dth, self.ki_dth, self.kd_dth = dtheta_k_matrix[EVENT]
        self.dtheta_pid = PIDController(self.kp_dth, self.ki_dth, self.kd_dth, PERIOD)

    def arm_switch(self, request, response):
        self.arm_flag = not self.arm_flag
        self.get_logger().info(f'switch to drive preparation : {self.prep_flag}')
        response.success = True
        if not self.flag:
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
        if not self.flag:
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

    def get_collected(self, msg):
        self.roll, self.pitch, self.yaw, self.altitude, self.local_alt, self.x, self.y, self.z,\
                          self.error, self.accumulatedError, self.currentTrack, self.polySlope, self.velocity, self.moved, self.numPoly = msg.data
        self.get_logger().info(f'Processed and published velocity')


    def save_droneInfo(self):
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
                self.drone_data['h_error'] = [self.error]
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
                self.drone_data['h_error'].append(self.error)
        else:
            if self.drone_data:
                os.makedirs('drone_data', exist_ok=True)
                df = pd.DataFrame(self.drone_data)
                file_name = f'drone_data/drone_data_event-{EVENT}_vel{self.velocity}_{self.kp_dv}-{self.ki_dv}-{self.kd_dv}_{self.kp_dth}-{self.ki_dth}-{self.kd_dth}.csv' # TODO
                file_name = get_new_filename(file_name)
                df.to_csv(file_name, index=False)
                self.drone_data = dict()


    def timer_callback(self):
        # 0. unarmed
        if not self.arm_flag and not self.prep_flag and not self.flag:
            self.local_pos.header.stamp = self.get_clock().now().to_msg()
            self.local_pos.header.frame_id = ''
            gap = self.local_alt - self.altitude
            self.local_pos.pose.position.x = self.x
            self.local_pos.pose.position.y = self.y
            self.local_pos.pose.position.z = gap + 2.5
            # qx,qy,qz,qw = calculate_quaternion(self.yaw, 0.0, 0.0)
            qx,qy,qz,qw = quaternion_from_euler(0, 0, self.yaw)
            self.local_pos.pose.orientation.x = qx
            self.local_pos.pose.orientation.y = qy
            self.local_pos.pose.orientation.z = qz
            self.local_pos.pose.orientation.w = qw
            # self.pos_publisher.publish(self.local_pos)
        # 1. armed. hovering
        elif self.arm_flag and not self.prep_flag and not self.flag:
            self.globalAngle = self.yaw
            self.currentAngle = self.yaw - np.radians(ROT_OFFSET) # TODO
            self.accumulatedError = DEFAULT[EVENT]
            self.prev_error = 0.0
            self.currentTrack = 0.0
            self.finish.data = False
            self.pos_publisher.publish(self.local_pos)
        # 2. prepare to drive
        elif self.arm_flag and self.prep_flag and not self.flag:
            if self.numPoly:
                theta = self.polySlope
                self.theta = theta
                self.currentTrack = 0
                omega = np.radians(2.) if theta >= np.radians(1.0) else (np.radians(-2.0) if theta <= np.radians(-1.0) else 0.0)
                vely = -0.05 if self.error >= 0.05 else (0.05 if self.error <= -0.05 else 0.0)
                self.twist_stamped.twist.linear.x = vely*np.sin(self.yaw)
                self.twist_stamped.twist.linear.y = -vely*np.cos(self.yaw)
                self.twist_stamped.twist.linear.z = (2.0-self.local_alt)/4
                self.twist_stamped.twist.angular.x = 0.0
                self.twist_stamped.twist.angular.y = 0.0
                self.twist_stamped.twist.angular.z = omega
                self.globalAngle = self.yaw
                self.currentAngle = self.yaw - np.radians(ROT_OFFSET) # TODO
                self.accumulatedError = DEFAULT[EVENT]
                self.prev_error = 0.0
                self.currentTrack = 0.0
                self.finish.data = False
                self.publisher.publish(self.twist_stamped)
        # 3. drive
        elif self.arm_flag and self.prep_flag and self.flag and self.moved <= self.distance + 10:
            # TODO from lidar
            self.velocity = 11.0
            tmp_vel = 11.0
            self.count += 2.0*PERIOD
            velocity = min(tmp_vel, self.count)
            if self.numPoly:
                # calculate theta
                theta = self.polySlope*(velocity/3)
                roll = self.roll
                theta += roll
                if abs(np.degrees(theta)) < 0.2:
                    theta = 0.0
                # error
                error = (self.accumulatedError + self.error)
                # 100m
                if EVENT == 0:
                    error = self.error
                    dvel = self.dvel_pid.compute(error)
                    dvel = np.clip(dvel,-0.3, 0.3)
                    self.dvel = dvel
                    dtheta = self.dtheta_pid.compute(error)
                    theta = dtheta
                    theta = np.clip(theta, np.radians(-2), np.radians(2))
                    self.globalAngle += theta*PERIOD
                    self.theta = theta
                # 200m
                elif EVENT == 1:
                    if (self.yaw - self.currentAngle >= np.pi or self.yaw - self.currentAngle <= -np.pi) and self.globalAngle >= np.pi/2:
                        self.is_curve = False
                        self.currentAngle = self.yaw
                        theta = 0.0
                        self.dtheta_pid.reset()
                        self.dvel_pid.reset()
                        self.accumulatedError = DEFAULT[EVENT]
                        error = (self.accumulatedError + self.error)
                        self.currentTrack = 0
                    dvel = self.dvel_pid.compute(error)
                    dvel = np.clip(dvel,-0.4, 0.4)
                    self.dvel = dvel
                    dtheta = self.dtheta_pid.compute(error) # TODO another PID controller
                    if self.is_curve:
                        theta = velocity/TRACK_R[DRONE_NUM]
                        theta += dtheta
                    else:
                        theta += dtheta
                        theta = np.clip(theta, np.radians(-4), np.radians(4))
                    self.globalAngle += theta*PERIOD
                    self.theta = theta
                 # 400m
                elif EVENT == 2:
                    pass
                 # over 400m. velocity must be under 8m/s
                else:
                    if self.currentAngle >= np.pi:
                        self.currentAngle -= np.pi
                        theta = 0.0
                        self.dtheta_pid.reset()
                        self.dvel_pid.reset()
                    dvel = self.dvel_pid.compute(error)
                    dtheta = self.dtheta_pid.compute(error)
                    self.dvel = dvel
                    theta += dtheta
                    theta = theta
                    self.currentAngle += theta*PERIOD
                    self.globalAngle += theta*PERIOD
                    self.theta = theta

                ## publish topic
                omega = theta
                dx = -dvel*np.sin(self.globalAngle)
                dy = dvel*np.cos(self.globalAngle)
                velx = velocity*np.cos(self.globalAngle)
                vely = velocity*np.sin(self.globalAngle)
                self.twist_stamped.twist.linear.x = velx + dx
                self.twist_stamped.twist.linear.y = vely + dy
                self.twist_stamped.twist.linear.z = (2.0-self.local_alt)/2
                self.twist_stamped.twist.angular.x = 0.0
                self.twist_stamped.twist.angular.y = 0.0
                self.twist_stamped.twist.angular.z = omega
                self.publisher.publish(self.twist_stamped)
                self.get_logger().info(f"angle : {np.degrees(self.globalAngle):5f}(deg), angle_flag : {np.degrees(self.currentAngle):5f}(deg), theta : {np.degrees(theta):5f}(deg), dtheta : {np.degrees(dtheta):5f}(deg)")
                self.get_logger().info("x : {:.2f}(m/s), y : {:.2f}(m/s), omega : {:.2f}(deg/s)".format(self.twist_stamped.twist.linear.x, self.twist_stamped.twist.linear.y, np.degrees(theta)))
                self.get_logger().info("moved : {:5f}(m), vel : {:5f}(m/s) dvel : {:5f}(m/s)".format(self.moved, velocity, dvel))

        elif self.arm_flag and self.prep_flag and self.flag and self.moved > self.distance + 10:
            self.finish.data = True
            self.twist_stamped.twist.linear.x = 0.0
            self.twist_stamped.twist.linear.y = 0.0
            self.twist_stamped.twist.linear.z = (2.0-self.local_alt)/2
            self.publisher.publish(self.twist_stamped)
            self.get_logger().warning(f'finishing.. {self.moved} : {self.finish.data}, flag : {self.flag}')
            if self.drone_mode == 'AUTO.LOITER':
                self.flag = False
                self.moved = 0.0
        self.get_logger().info(f'roll : {np.degrees(self.roll):5f} deg,  pitch : {np.degrees(self.pitch):5f} deg,  yaw : {np.degrees(self.yaw):5f} deg')
        self.save_droneInfo()
        self.finish_publisher.publish(self.finish)

def main(args=None):
    rclpy.init(args=args)
    statecollector = DroneStateCollector()
    position_publisher = PositionPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(statecollector)
    executor.add_node(position_publisher)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        statecollector.destroy_node()
        position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
