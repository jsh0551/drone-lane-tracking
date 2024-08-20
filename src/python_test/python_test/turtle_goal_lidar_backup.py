import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from geometry_msgs.msg import Point


import math

class TurtleFollower:
    def __init__(self, turtle_name):
        self.node = rclpy.create_node(f'{turtle_name}_follower')
        self.publisher = self.node.create_publisher(Twist, f'/{turtle_name}/cmd_vel', 10)
        self.subscriber = self.node.create_subscription(
            Point,
            '/range_angle',
            self.point_callback,
            10
        )
        
        self.subscriber = self.node.create_subscription(
            Pose,
            f'/{turtle_name}/pose',
            self.sub_pose_callback,
            10
        )        
        
        self.turtle_pose = Pose()
        
    def sub_pose_callback(self,msg):
        self.turtle_pose.x = msg.x
        self.turtle_pose.y = msg.y

    def point_callback(self, msg):
        # 목표 위치 업데이트
        # self.goal_pose = msg    
        target_pose = Pose()
        
        Range = msg.x*2
        Angle = msg.y
        
        target_pose.x = Range*math.cos(Angle)
        target_pose.y = Range*math.sin(Angle)
        
        print(target_pose.x,'\n',target_pose.y )
        print("==============================")
        
        target_pose.x += 5
        target_pose.y += 5
        # self.get_logger().info('Turtle teleported to x={}, y={}'.format(target_pose.x, target_pose.y))

        # turtle_pose_x = self.turtle_pose.x
        # turtle_pose_y = self.turtle_pose.y

        # 현재 터틀의 위치와 목표 위치를 고려하여 이동 명령 생성
        cmd_vel_msg = Twist()
        # distance = math.sqrt((self.goal_pose.x)**2 + (self.goal_pose.y)**2)
        # angle = math.atan2(self.goal_pose.y, self.goal_pose.x)
        
        # distance = math.sqrt((self.goal_pose.x - current_pose.x)**2 + (self.goal_pose.y - current_pose.y)**2)
        # angle = math.atan2(self.goal_pose.y - current_pose.y, self.goal_pose.x - current_pose.x)   
        if target_pose.x > self.turtle_pose.x:
            x_distance =  abs(target_pose.x - self.turtle_pose.x)
        elif target_pose.x < self.turtle_pose.x:
            x_distance = -abs(target_pose.x - self.turtle_pose.x)
        elif target_pose.x == self.turtle_pose.x:
            x_distance = 0.0            
            
        if target_pose.y > self.turtle_pose.y:
            y_distance =  abs(target_pose.y - self.turtle_pose.y)
        elif target_pose.y < self.turtle_pose.y:
            y_distance = -abs(target_pose.y - self.turtle_pose.y)
        elif target_pose.y == self.turtle_pose.y:
            y_distance = 0.0       
        
        cmd_vel_msg.linear.x = min(x_distance, 1.0)  # 최대 선속도를 1.0으로 제한
        cmd_vel_msg.linear.y = min(y_distance, 1.0)  # 최대 선속도를 1.0으로 제한
        # cmd_vel_msg.linear.y = 2.0 * math.sin(angle - current_pose.theta)  # 회전 속도 계산

        # 이동 명령 발행
        self.publisher.publish(cmd_vel_msg)
        
        if target_pose.x == self.turtle_pose.x and target_pose.y == self.turtle_pose.y:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    # turtle1의 이름은 'turtle1', turtle2의 이름은 'turtle2'로 가정
    turtle_follower = TurtleFollower( 'turtle1')

    rclpy.spin(turtle_follower.node)
    turtle_follower.node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
