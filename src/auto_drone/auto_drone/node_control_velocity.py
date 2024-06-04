import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_srvs.srv import SetBool

import numpy as np
from tf_transformations import quaternion_from_euler

from mavros_msgs.msg import Altitude, PositionTarget
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, Quaternion

from custom_message.msg import LandmarkCloud


PI = np.pi
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
    depth=1
)


def radians(degrees):
    return degrees * np.pi / 180


def calculate_quaternion(yaw_deg, pitch_deg, roll_deg):
    """Calculate quaternion from yaw, pitch, and roll in degrees."""
    yaw = radians(yaw_deg)
    pitch = radians(pitch_deg)
    roll = radians(roll_deg)
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion



class PIDController():
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
    
    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('node_velocity_control')
        # Publishing position and velocity command to pixhawk
        self.service = self.create_service(SetBool, 'vel_data/switch', self.drive_switch)
        # self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.publisher = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)

        # Subscribing position information from pixhawk
        self.subscriber_state = self.create_subscription(
            Altitude, '/mavros/altitude', self.get_altitude, qos_profile)
        self.subscriber_position = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.get_position, qos_profile)
        self.subscriber_velocity = self.create_subscription(
            Odometry, '/mavros/local_position/velocity_local', self.get_velocity, 10)
        
        # Subscribing results from detection node
        self.subscriber_landmark = self.create_subscription(
            LandmarkCloud, '/landmark_points', self.get_landmark, 10)
        
        self.timer = self.create_timer(period, self.timer_callback)
        self.pose = TwistStamped()
        self.local_pos = PoseStamped()
        self.step = 0
        self.flag = False
        self.get_logger().info(f'wait for start signal..')
        self.altitude, self.local_alt = 2.5, 2.5
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.prev_error = 0
        self.integral = 0
        self.angle = 0.0

        # Ziegler-Nichols tuning
        ku_lx = 2.5
        ku_ly = 2.5
        ku_az = 2.5
        tu_lx = 1.2
        tu_ly = 1.2
        tu_az = 1.2

        # PID Parameter tuning
        kp_lx = 0.6 * ku_lx
        ki_lx = 2 * kp_lx / tu_lx
        kd_lx = kp_lx * tu_lx / 8

        kp_ly = 0.6 * ku_ly
        ki_ly = 2 * kp_ly / tu_ly
        kd_ly = kp_ly * tu_ly / 8

        kp_az = 0.6 * ku_az
        ki_az = 2 * kp_az / tu_az
        kd_az = kp_az * tu_az / 8

        # PID Controller setting
        # self.linear_x_pid = PIDController(kp_lx, ki_lx, kd_lx)
        # self.linear_y_pid = PIDController(kp_ly, ki_ly, kd_ly)
        # self.angular_z_pid = PIDController(kp_az, ki_az, kd_az)
        self.linear_x_pid = PIDController(0.1, 0.001, 0.1)
        self.linear_y_pid = PIDController(0.1, 0.001, 0.1)
        self.angular_z_pid = PIDController(0.1, 0.001, 0.1)

        # Current position of drone
        self.current_pose = None
        self.current_velocity = None

        # Target position
        self.target_velocity_x = 320
        self.target_point_x = 240

        # Calculate velocity of drone
        self.twist = Twist()


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


    def get_position(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z


    def get_hovering_state(self, tmp_pos, yaw=180.0):
        tmp_pos.header.stamp = self.get_clock().now().to_msg()
        tmp_pos.header.frame_id = ''
        gap = self.local_alt - self.altitude
        x,y,z,w = calculate_quaternion(yaw, 0.0, 0.0)
        tmp_pos.pose.orientation.x = x
        tmp_pos.pose.orientation.y = y
        tmp_pos.pose.orientation.z = z
        tmp_pos.pose.orientation.w = w
        # tmp_pos.pose.position.x = self.x
        # tmp_pos.pose.position.y = self.y
        tmp_pos.pose.position.z = gap + 2.5
        return tmp_pos


    # def cal_pid(self, target=2.5):
    #     error = target - self.altitude
    #     self.integral += error * period
    #     derivative = (error - self.prev_error) / period
    #     pid_value = kp * error + ki * self.integral + kd * derivative
    #     self.prev_error = error 
    #     return pid_value


    def timer_callback(self):
        ## Vision-based control code
        self.publisher.publish(self.twist)
        self.get_logger().info("Following track with linear.x: {:.2f} m/s, linear.y: {:.2f} m/s, angular.z: {:.2f} rad/s"\
                               .format(self.twist.linear.x, self.twist.linear.y, self.twist.angular.z))
        
        ## Scenario-based control code
        # self.pose.header.stamp = self.get_clock().now().to_msg()
        # self.pose.header.frame_id = ''
        # pid_value = self.cal_pid()
        # self.get_logger().info(f'pid value : {pid_value}')
        # if self.flag:
        #     # move drone with time steps
        #     if self.step < count:
        #         self.pose.twist.linear.x = -min(float(self.step)/4, vel) 
        #         self.pose.twist.linear.y = 0.0
        #         self.pose.twist.linear.z = pid_value
        #     elif self.step < count*2:
        #         theta = PI*(self.step-count)/count
        #         velx = vel*np.cos(theta)
        #         vely = vel*np.sin(theta)
        #         dangle = theta-self.angle
        #         self.angle = theta
        #         self.pose.twist.linear.x = -velx
        #         self.pose.twist.linear.y = -vely
        #         self.pose.twist.linear.z = pid_value
        #         self.pose.twist.angular.x = 0.0
        #         self.pose.twist.angular.y = 0.0
        #         self.pose.twist.angular.z = dangle/period
        #     else:
        #         self.pose.twist.linear.x = min(float(count*3-self.step)/4, vel)  
        #         self.pose.twist.linear.y = 0.0
        #         self.pose.twist.linear.z = pid_value
        #         self.pose.twist.angular.x = 0.0
        #         self.pose.twist.angular.y = 0.0
        #         self.pose.twist.angular.z = 0.0
        #     self.step += 1
        #     if self.step >= count*3:
        #         self.step = 0
                
        #     self.publisher.publish(self.pose)
        #     self.get_logger().info(f'step : {self.step}, vel : {self.pose.twist.linear.x}')
        # else:
        #     self.local_pos = self.get_hovering_state(self.local_pos)
        #     self.pos_publisher.publish(self.local_pos)
        # # self.get_logger().info(f'{x},{y},{z},{w}')
    

    def get_velocity(self, msg):
        self.current_velocity = msg.twist.twist
    

    def get_landmark(self, landmarkData):
        ## All the codes below are written with assumption that
        ## lanes are properly detected
        self.get_logger().info('Receiving landmark data')
        
        if landmarkData:
            lines = []
            for cloud in landmarkData.clouds:
                # print("New Cloud:")
                cloud_coord = {'x':[], 'y':[]}
                for point in cloud.points:
                    cloud_coord['x'].append(point.x)
                    cloud_coord['y'].append(point.y)
                lines.append(cloud_coord)
            if lines:
                # If lines are detected
                mean_lines = {'x':[], 'y':[]}
                for line in lines:
                    x_mean = sum(line['x'])/len(line['x'])
                    y_mean = sum(line['y'])/len(line['y'])
                    mean_lines['x'].append(x_mean)
                    mean_lines['y'].append(y_mean)
                x_center = self.frame_width/2
                x_mean_calib = [x - x_center for x in mean_lines['x']]

                max_neg = float('-inf')
                min_pos = float('inf')
                num_left_lane = -1
                num_right_lane = -1

                for i, num in enumerate(x_mean_calib):
                    if num < 0 and num > max_neg:
                        max_neg = num
                        num_left_lane = i
                    elif num > 0 and num < min_pos:
                        min_pos = num
                        num_right_lane = i

                if num_left_lane and num_right_lane:
                    left = lines[num_left_lane]
                    right = lines[num_right_lane]
                    print("left : ", left)
                    print("right : ", right)
                    line_center_zero = [(left['x'][0]+right['x'][0])/2, (left['y'][0]+right['y'][0])/2]
                    line_center_three = [(left['x'][3]+right['x'][3])/2, (left['y'][3]+right['y'][3])/2]
                    center_points = [line_center_zero, line_center_three]
                    self.control_drone(center_points)


    def control_drone(self, center_points):
        ## Calculate pitching (Human following)
        ## self.twist.linear.x = self.linear_x_pid.compute()
        ## Calculate rolling
        bias_y = self.frame_width/2 - center_points[0][0]
        distance_y = bias_y * 1.2/290
        self.twist.linear.y = self.linear_y_pid.compute(0, distance_y)
        ## Calculate yawing
        self.twist.angular.z = self.angular_z_pid.compute(0, self.calculate_angle(center_points))


    def calculate_angle(self, center_points):
        # y1 - y0
        bottom_triangle = center_points[0][1] - center_points[1][1]
        # x1 - x0 (anti-clockwise direction is positive)
        height_triangle = -(center_points[1][0] - center_points[0][0])
        # tangent
        tangent = height_triangle/bottom_triangle
        # radian
        radian = np.arctan(tangent)
        return radian

 
def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
