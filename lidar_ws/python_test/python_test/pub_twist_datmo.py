import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from datmo_msg.msg import TrackArray
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

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.zerodist = (0,0)

    def topic_callback(self, data):
        lentopic = len(data.tracks)
        shortdist = float('inf')
        
        twist_msg = Twist()
        
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        x = 0.0
        y = 0.0

        for i in range(lentopic):
            
            dist = (data.tracks[i].odom.pose.pose.position.x,data.tracks[i].odom.pose.pose.position.y)
            eucldist = distance.euclidean(self.zerodist,dist)

            # if eucldist < shortdist and eucldist > 0.9:
            if eucldist < shortdist:

                shortdist = eucldist
                point = i
                
                twist_msg.linear.x = data.tracks[i].odom.twist.twist.linear.x
                twist_msg.linear.y = data.tracks[i].odom.twist.twist.linear.y
                twist_msg.angular.z = data.tracks[i].odom.twist.twist.angular.z
                
                x = data.tracks[i].odom.pose.pose.position.x
                y = data.tracks[i].odom.pose.pose.position.y
                
            # print("----------------")
            # print(eucldist)


            # # print(data.tracks[i].odom.pose.pose.position.x)
            # # print(data.tracks[i].odom.pose.pose.position.y)
            # print("----------------")
        
        print("shortdist")
        print(shortdist)
        print(point)
        print("x: ",x, "y: ",y,"\t")
        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    point_node = PointNode()
    rclpy.spin(point_node)
    point_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

