import rclpy
from rclpy.node import Node
from datmo_msg.msg import TrackArray
from std_msgs.msg import String
import numpy as np
import math
from scipy.spatial import distance

class PointNode(Node):
    def __init__(self):
        super().__init__('topic_listener')
        
        self.subscription = self.create_subscription(
            TrackArray,
            '/Datmo',
            self.topic_callback,
            10
        )

    def topic_callback(self, data):
        lentopic = len(data.tracks)
        
 


def main(args=None):
    rclpy.init(args=args)
    point_node = PointNode()
    rclpy.spin(point_node)
    point_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

