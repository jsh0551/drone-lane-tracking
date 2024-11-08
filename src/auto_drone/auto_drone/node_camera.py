import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class CameraNode(Node):
    def __init__(self, camera_name, namespace, device_path):
        super().__init__(camera_name, namespace=namespace)
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer_period = 1/20
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device_path,cv2.CAP_V4L2)

    def timer_callback(self):
        ret, frame = self.cap.read()
        t = self.get_clock().now().to_msg().sec
        nt = str(self.get_clock().now().to_msg().nanosec)

        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing video frame from {self.get_name()}, t = {t}.{nt[:6]}")
        else:
            self.get_logger().warn(f"Failed to capture video frame from {self.get_name()}, t = {t}.{nt[:6]}")

def main(args=None):
    rclpy.init(args=args)
    camera_node1 = CameraNode('node_camera_1', 'camera1', '/dev/video0')
    camera_node2 = CameraNode('node_camera_2', 'camera2', '/dev/video2')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera_node1)
    executor.add_node(camera_node2)
    try:
        executor.spin()
    finally:
        camera_node1.destroy_node()
        camera_node2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()