import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class ScanProcessorNode(Node):
    def __init__(self):
        super().__init__('scan_processor_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.publisher = self.create_publisher(Point, '/range_angle', 10)
        self.msg_data = Point()

    def scan_callback(self, msg):
        # 가장 가까운 거리와 각도를 찾는 로직
        min_range = min(msg.ranges)
        min_range_index = msg.ranges.index(min_range)
        min_range_angle = msg.angle_min + min_range_index * msg.angle_increment

        # 찾은 값들을 퍼블리시
        self.msg_data.x = min_range
        self.msg_data.y = min_range_angle
        self.msg_data.z = 0.0
        self.publisher.publish(self.msg_data)

def main(args=None):
    rclpy.init(args=args)
    scan_processor_node = ScanProcessorNode()
    rclpy.spin(scan_processor_node)
    scan_processor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
