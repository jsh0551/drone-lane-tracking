import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class DriveClient(Node):
    def __init__(self):
        super().__init__('drive_client')
        self.client = self.create_client(SetBool, 'server/drive_auto')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('drive service not available')
        self.req = SetBool.Request()

    def send_request(self):
        self.req.data = True
        self.future = self.client.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    drive_client = DriveClient()
    drive_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(drive_client)
        if drive_client.future.done():
            try:
                response = drive_client.future.result()
                drive_client.get_logger().info(
                    f'Result of set_bool: {response.success}, {response.message}')
            except Exception as e:
                drive_client.get_logger().error('Service call failed %r' % (e,))
            break
    drive_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
