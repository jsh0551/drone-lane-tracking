from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, PositionTarget
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import numpy as np
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Header

PI = np.pi
count = 60
vel = 2.0
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

class PositionPublisher(Node):
    def __init__(self):
        super().__init__('position_publisher')
        self.service = self.create_service(SetBool, 'vel_data/switch', self.drive_switch)
        self.publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        self.pos_publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.state_subscription = self.create_subscription(
            Altitude,
            '/mavros/altitude',
            self.get_altitude,
            qos_profile)
        self.pos_subscription = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.get_pos,
            qos_profile)

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

        self.set_pose = PoseStamped()
        
        # Set the header
        self.set_pose.header = Header()
        self.set_pose.header.stamp = self.get_clock().now().to_msg()
        self.set_pose.header.frame_id = 'map'
        
        # Set the pose (assuming the current position is the origin)
        self.set_pose.pose.position.x = 0.0
        self.set_pose.pose.position.y = 0.0
        # self.pose.pose.orientation.x = 0.0
        # self.pose.pose.orientation.y = 0.0
        # self.pose.pose.orientation.z = 0.0
        # self.pose.pose.orientation.w = 1.0

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

    def cal_pid(self, target=2.5):
        error = target - self.altitude
        self.integral += error * period
        derivative = (error - self.prev_error) / period
        pid_value = kp * error + ki * self.integral + kd * derivative
        self.prev_error = error 
        return pid_value

    def timer_callback(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = ''
        if self.flag:
            if self.step == 0:
                self.pose.header.stamp = self.get_clock().now().to_msg()
                self.pos_publisher.publish(self.set_pose)
            theta = 2*PI*self.step / (count*4)
            velx = vel*np.cos(theta)
            vely = vel*np.sin(theta)
            dangle = 2*PI / (count*4)
            self.angle = theta
            self.pose.twist.linear.x = -velx
            self.pose.twist.linear.y = -vely
            # self.pose.twist.linear.z = 0.0
            self.pose.twist.angular.x = 0.0
            self.pose.twist.angular.y = 0.0
            self.pose.twist.angular.z = dangle/period

            self.step += 1
            if self.step >= count*4:
                self.step = 0
                
            self.publisher.publish(self.pose)
            self.get_logger().info(f'step : {self.step}, theta : {theta}, vel : {self.pose.twist.linear.x}')
        else:
            self.local_pos = self.get_hovering_state(self.local_pos)
            self.pos_publisher.publish(self.local_pos)
        # self.get_logger().info(f'{x},{y},{z},{w}')
def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
