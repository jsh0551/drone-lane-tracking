import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np
import math


class PointNode(Node):
    def __init__(self):
        super().__init__('point_node')

        self.publisher = self.create_publisher(Point, '/point', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1.0

        self.xyz_data = Point()
        self.xyz_data.x = 0.0
        self.xyz_data.y = 0.0
        self.xyz_data.z = 0.0

    def timer_callback(self):
        
        self.i += 1.0
        if self.i > 10.1:
            self.i = 1.0
                           
        self.xyz_data.x = self.i
        self.xyz_data.y = 5.0     
        # self.xyz_data.y = self.i + 2.0     
        
        # print(self.xyz_data.x)

        self.publisher.publish(self.xyz_data)


def main(args=None):
    rclpy.init(args=args)
    point_node = PointNode()
    rclpy.spin(point_node)
    point_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
