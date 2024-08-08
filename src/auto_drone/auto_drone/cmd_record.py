import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class RecordClient(Node):
    def __init__(self):
        super().__init__('record_client')
        self.client = self.create_client(SetBool, 'video/switch')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('takeoff service not available')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    record_client = RecordClient()
    record_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(record_client)
        if record_client.future.done():
            try:
                response = record_client.future.result()
                record_client.get_logger().info(
                    f'Result of set_bool: {response.success}, {response.message}')
            except Exception as e:
                record_client.get_logger().error('Service call failed %r' % (e,))
            break
    record_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
