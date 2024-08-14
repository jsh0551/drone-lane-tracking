from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import numpy as np
import os
import sys
BASE = os.getcwd()
sys.path.append(os.path.join(BASE, "src","utils"))
from tools import *

count = 80
vel = 1.0
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
        self.publisher = self.create_publisher(TwistStamped, 'vel_data/velocity', 10)
        self.timer = self.create_timer(period, self.timer_callback)
        self.pose = TwistStamped()
        self.finish = Bool()
        self.step = 0
        self.flag = False
        self.finish.data = False

    def drive_switch(self, request, response):
        self.flag = not self.flag
        self.step = 0
        self.get_logger().info(f'switch to drive : {self.flag}')
        response.success = True
        if not self.flag:
            self.get_logger().info(f'wait for start signal..')
        return response


    def timer_callback(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = ''

        if self.flag:
            self.get_logger().info(f'step : {self.step}')
            if self.step < count:
                self.pose.twist.linear.x = min(float(self.step)/4, vel) 
                self.pose.twist.linear.y = 0.0
                self.pose.twist.linear.z = 0.0
            else:
                self.pose.twist.linear.x = -min(float(count*2-self.step)/4, vel)  
                self.pose.twist.linear.y = 0.0
                self.pose.twist.linear.z = 0.0
            self.step += 1
            if self.step >= count*2:
                self.step = 0
        else:
            self.pose.twist.linear.x = 0.0 
            self.pose.twist.linear.y = 0.0
            self.pose.twist.linear.z = 0.0
            self.pose.twist.angular.x = 0.0
            self.pose.twist.angular.y = 0.0
            self.pose.twist.angular.z = 0.0
        self.publisher.publish(self.pose)


def main(args=None):
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    position_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
