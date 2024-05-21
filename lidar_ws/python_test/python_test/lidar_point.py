import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import numpy as np
import math


class ScanProcessorNode(Node):
    def __init__(self):
        super().__init__('scan_processor_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
                
    def scan_callback(self,scan_data):
        self.get_logger().info('----- scan -----\t')


def main(args=None):
    rclpy.init(args=args)
    scan_processor_node = ScanProcessorNode()
    rclpy.spin(scan_processor_node)
    scan_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
