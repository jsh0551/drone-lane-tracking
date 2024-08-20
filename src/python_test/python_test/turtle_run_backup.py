import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import math

class TurtleController:
    def __init__(self):
        self.node = rclpy.create_node('turtle_controller')
        self.publisher = self.node.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscriber = self.node.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.goal_pose = Pose()

    def pose_callback(self, msg):
        # 현재 터틀의 위치 업데이트
        current_pose = msg
        print(self.goal_pose.x)
        print("-----------------------")
        print(current_pose.x)

        # 목표 위치까지의 거리와 방향 계산
        distance = math.sqrt((self.goal_pose.x - current_pose.x)**2 + (self.goal_pose.y - current_pose.y)**2)
        angle = math.atan2(self.goal_pose.y - current_pose.y, self.goal_pose.x - current_pose.x)

        # 이동 명령 생성
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = min(distance, 1.0)  # 최대 선속도를 1.0으로 제한
        cmd_vel_msg.angular.z = 2.0 * math.sin(angle - current_pose.theta)  # 회전 속도 계산

        # 터틀 이동 명령을 발행
        self.publisher.publish(cmd_vel_msg)

    def set_goal_pose(self, x, y):
        # 목표 위치 설정
        self.goal_pose.x = x
        self.goal_pose.y = y

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleController()

    # 원하는 목표 위치를 설정 (예: x=5.0, y=5.0)
    goal_x = 2.0
    goal_y = 2.0
    controller.set_goal_pose(goal_x, goal_y)

    rclpy.spin(controller.node)
    controller.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
