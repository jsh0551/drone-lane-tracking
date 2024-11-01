import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class TakeoffClient(Node):
    def __init__(self):
        super().__init__('takeoff_client')
        self.client = self.create_client(SetBool, '/server/takeoff')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    takeoff_client = TakeoffClient()
    takeoff_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(takeoff_client)
        if takeoff_client.future.done():
            try:
                response = takeoff_client.future.result()
                takeoff_client.get_logger().info(
                    f'Result of set_bool: {response.success}, {response.message}')
            except Exception as e:
                takeoff_client.get_logger().error('Service call failed %r' % (e,))
            break
    takeoff_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
