import rclpy
from turtlesim.srv import Spawn

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('spawn_turtle2_node')

    client = node.create_client(Spawn, 'spawn')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service not available, waiting again...')

    request = Spawn.Request()
    request.x = 5.0  # Set the X-coordinate for turtle2 spawn
    request.y = 5.0 # Set the Y-coordinate for turtle2 spawn
    request.theta = 0.0  # Set the initial orientation for turtle2 spawn
    request.name = 'turtle2'  # Set the name for turtle2

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Turtle2 spawned successfully')
    else:
        node.get_logger().error('Failed to spawn turtle2')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()