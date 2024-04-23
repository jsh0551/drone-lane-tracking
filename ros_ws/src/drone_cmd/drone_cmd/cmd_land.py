import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class LandClient(Node):
    def __init__(self):
        super().__init__('land_client')
        self.client = self.create_client(SetBool, 'server/land')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('land service not available')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    land_client = LandClient()
    land_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(land_client)
        if land_client.future.done():
            try:
                response = land_client.future.result()
                land_client.get_logger().info(
                    f'Result of set_bool: {response.success}, {response.message}')
            except Exception as e:
                land_client.get_logger().error('Service call failed %r' % (e,))
            break
    land_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
