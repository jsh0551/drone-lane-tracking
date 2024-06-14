import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from custom_message.msg import LandmarkCloud
import numpy as np


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


class DirectionNodePID(Node):
    def __init__(self):
        super().__init__("node_control_direction")
        self.subscriber_landmark = self.create_subscription(
            LandmarkCloud, '/landmark_points', self.listener_callback, 10)
        self.subscriber_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10)
        self.subscriber_velocity = self.create_subscription(
            Odometry, '/mavros/local_position/velocity_local', self.velocity_callback, 10)
        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.roll_command = 0.1
        self.frame_width = 640

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


    

    def pose_callback(self, msg):
        self.current_pose = msg.pose
    

    def velocity_callback(self, msg):
        self.current_velocity = msg.twist.twist


    def listener_callback(self, landmarkData):
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
    

    def timer_callback(self):
        self.publisher.publish(self.twist)
        # self.get_logger().info("Following track with linear.x: {:.2f} m/s, linear.y: {:.2f} m/s, angular.z: {:.2f} rad/s"\
        #                        .format(self.twist.linear.x, self.twist.linear.y, self.twist.angular.z))





# class DirectionNode(Node):
#     def __init__(self):
#         super().__init__("control_direction_node")
#         self.sub = self.create_subscription(
#             LandmarkCloud, '/landmark_points', self.listener_callback, 10)
#         self.pub = self.create_publisher(
#             Twist, '/cmd_vel', 10)
#         timer_period = 0.1
#         self.timer = self.create_timer(timer_period, self.timer_callback)
#         self.roll_command = 0.1

#         self.linear_x_pid = PIDController(1.0, 0.0, 0.1)
#         self.linear_y_pid = PIDController(1.0, 0.0, 0.1)
#         self.angular_z_pid = PIDController(1.0, 0.0, 0.1)

#         self.target_velocity_x = 320
#         self.target_point_x = 240


#     def check_declines(list, tolerance):
#         increasing = True
#         decreasing = True
#         constant = True

#         for i in range(1,5):
#             diff = list[i] - list[i-1]

#             if abs(diff) > tolerance:
#                 constant = False
#                 if diff > 0:
#                     decreasing = False
#                 elif diff < 0:
#                     increasing = False
        
#         if constant:
#             return "Straight"
#         else:
#             return "Cureved"
            

#     def listener_callback(self, landmarkData):
#         '''
#         Parameters)
#         - landmarkData : Data of received message '/landmark_points'

#         Description)
#         - Callback function of subscribtion node
#         - Separate flying position into 4 cases
#           : 1) Straight lane
#               - Fixed position for going straight
#           : 2) Transition point (straight -> curve)
#               - Turn position for turning
#           : 3) Curved lane
#               - Fixed position for turning
#           : 4) Transition point (curve -> straight)
#               - Turn position for turning
#         '''

#         self.get_logger().info('Receiving landmark data')
        
#         if landmarkData:
#             lines = []
#             for cloud in landmarkData.clouds:
#                 # print("New Cloud:")
#                 cloud_coord = {'x':[], 'y':[]}
#                 for point in cloud.points:
#                     # print(f"Point coordinates: x={point.x}, y={point.y}")
#                     cloud_coord['x'].append(point.x)
#                     cloud_coord['y'].append(point.y)
#                 lines.append(cloud_coord)
#             if lines:
#                 # If lines are detected
#                 mean_lines = {'x':[], 'y':[]}
#                 for line in lines:
#                     x_mean = sum(line['x'])/len(line['x'])
#                     y_mean = sum(line['y'])/len(line['y'])
#                     mean_lines['x'].append(x_mean)
#                     mean_lines['y'].append(y_mean)
#                 x_center = FRAME_WIDTH/2
#                 x_mean_calib = [x - x_center for x in mean_lines['x']]

#                 # Get the element number of main line(left, right) among detected lines
#                 number_left_lane = len([x for x in x_mean_calib if x < 0]) - 1
#                 number_right_lane = number_left_lane + 1
#                 if (number_left_lane >= 0) and (number_right_lane <= len(lines) - 1):
#                     # When both left and right lanes are detected

#                     ## 1) Calculate the declines of landmarks => Straight or Curved
#                     declines_left = []
#                     for i in range(1,5) :
#                         width = lines['number_left_lane']['x'][i] - lines['number_left_lane']['x'][i-1]
#                         height = lines['number_left_lane']['y'][i] - lines['number_left_lane']['y'][i-1]
#                         declines_left[i] = height/width
#                     ## Decide the declines are constant or changing
#                     line_result = self.check_declines(declines_left, 0.1)

#                     ## 2) Calculate bias from center line
#                     left = lines[number_left_lane]
#                     right = lines[number_right_lane]
#                     line_center_zero = [(left[0]['x']+right[0]['x'])/2, (left[0]['y']+right[0]['y'])/2]
#                     bias = line_center_zero - x_center
#                     rolling = 0
#                     if bias > 10:
#                         rolling -= 0.1
#                     elif bias < -10:
#                         rolling += 0.1
                    
#                     ## 3) Calculate twist from the ahead
#                     ### Use first two landmarks from the bottom (They are close to straight)
#                     line_center_one = [(left[1]['x']+right[1]['x'])/2, (left[1]['y']+right[1]['y'])/2]
#                     bias_zero = line_center_zero[0] - x_center
#                     bias_one = line_center_one[0] - x_center

#                     yawing = 0
#                     if bias_zero > bias_one:
#                         yawing -= 0.1
#                     elif bias_zero < bias_one:
#                         yawing += 0.1


#                     twist = Twist()
#                     ## Go straight if lines
#                     twist.linear.x = 10.0
#                     ## Roll to match the center line
#                     twist.linear.y += rolling
#                     ## Yaw to match the straight
#                     twist.angular.z += yawing




    
    # def check_declines(declines_list, tolerance):
    #     increasing = True
    #     decreasing = True
    #     constant = True

    #     for i in range(1,5):
    #         diff = declines_list[i] - declines_list[i-1]

    #         if abs(diff) > tolerance:
    #             constant = False
    #             if diff > 0:
    #                 decreasing = False
    #             elif diff < 0:
    #                 increasing = False
        
    #     if constant:
    #         return "Straight"
    #     else:
    #         return "Curved"


    # def timer_callback(self):
    #     msg = TwistStamped()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.twist.angular.x = 0.0
    #     msg.twist.angular.y = 0.0
    #     msg.twist.angular.z = self.roll_command
    #     self.publisher.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.twist.angular.z)



def main(args=None):
    rclpy.init(args=args)
    # node = DirectionNode()
    node = DirectionNodePID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()