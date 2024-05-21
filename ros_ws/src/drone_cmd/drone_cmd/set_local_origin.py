import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool

class SetLocalPositionOrigin(Node):
    def __init__(self):
        super().__init__('set_local_position_origin')
        self.publisher = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose = PoseStamped()
        self.service = self.create_service(SetBool, 'vel_data/switch', self.drive_switch)
        
        # Set the header
        self.pose.header = Header()
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = 'map'
        
        # Set the pose (assuming the current position is the origin)
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 1.0
        self.step = 0
        self.flag = False
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

    def timer_callback(self):
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.pose)
        self.get_logger().info('Publishing local position setpoint as origin')

def main(args=None):
    rclpy.init(args=args)
    node = SetLocalPositionOrigin()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
