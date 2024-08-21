import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2
import datetime

PERIOD = 0.05  # 프레임 간격 (초)
WIDTH = 480
HEIGHT = 360

def get_current_time_string():
    now = datetime.datetime.now()
    time_str = now.strftime('%Y-%m-%d-%H%M%S')
    return time_str

class VideoCaptureNode(Node):
    def __init__(self):
        super().__init__('video_capture_node')
        self.subscription = self.create_subscription(
            Image, '/camera1/video_frames', self.image_callback, 10)
        self.subscription2 = self.create_subscription(
            Image, '/camera2/video_frames', self.image_callback2, 10)
        self.srv = self.create_service(SetBool, 'video/switch', self.service_callback)
        self.cv_bridge = CvBridge()
        self.video_writer = None
        self.video_writer2 = None
        self.is_recording = False
        self.switch = False

    def image_callback(self, msg):
        if self.is_recording:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.video_writer.write(cv_image)

    def image_callback2(self, msg):
        if self.is_recording:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.video_writer2.write(cv_image)

    def service_callback(self, request, response):
        self.switch = not self.switch
        if self.switch and not self.is_recording:
            current_time = get_current_time_string()
            self.video_writer = cv2.VideoWriter(f'drone_video/c1_{current_time}.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 1/PERIOD, (WIDTH, HEIGHT))
            self.video_writer2 = cv2.VideoWriter(f'drone_video/c2_{current_time}.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 1/PERIOD, (WIDTH, HEIGHT))
            self.is_recording = True
            response.success = True
            self.get_logger().info('Video recording started')
        elif not self.switch and self.is_recording:
            self.video_writer.release()
            self.video_writer2.release()
            cv2.destroyAllWindows()
            self.is_recording = False
            response.success = True
            self.get_logger().info('Video recording stopped')
        else:
            response.success = False
        return response

def main():
    rclpy.init()
    node = VideoCaptureNode()

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()