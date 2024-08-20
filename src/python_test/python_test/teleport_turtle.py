import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute

class TeleportTurtle(Node):
    def __init__(self):
        super().__init__('teleport_turtle')
        self.teleport_absolute_client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        self.point_subscription = self.create_subscription(Point, '/point', self.pose_callback, 10)
        # self.point_subscription = self.create_subscription(Point, '/point', self.pose_callback, 10)

        while not self.teleport_absolute_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, waiting again...')

    def pose_callback(self, msg):
        # Callback function to handle incoming Pose messages
        self.get_logger().info('Received turtle pose: x={}, y={}'.format(msg.x, msg.y))
        self.send_teleport_request(x=msg.x, y=msg.y, theta=0.0)
        # self.send_teleport_request(x=msg.x*2, y=5.0, theta=0.0)

    def send_teleport_request(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = self.teleport_absolute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Turtle teleported to x={}, y={}, theta={}'.format(x, y, theta))
        else:
            self.get_logger().error('Failed to teleport turtle')

def main(args=None):
    rclpy.init(args=args)

    try:
        teleport_turtle_node = TeleportTurtle()
        rclpy.spin(teleport_turtle_node)
    except Exception as e:
        print(f'Error: {e}')
    finally:
        teleport_turtle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
