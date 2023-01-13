#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates


message = Pose()
message.position.x = 0.0
message.position.y = 0.0
message.position.z = 0.0
message.orientation.x = 0.0
message.orientation.y = 0.0
message.orientation.z = 0.0
message.orientation.w = 0.0


class BotPose(Node):

    def __init__(self):
        super().__init__('botPose')
        self.publisher_ = self.create_publisher(Pose, '/position_robot', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.subscription = self.create_subscription(LinkStates,'/link_states', self.listener_callback, 10)


    def timer_callback(self):
        self.publisher_.publish(message)


    def listener_callback(self, msg):
        print(msg.name)
        global message
        name = msg.name
        for i in range(len(name)):
            if name[i] == 'sam_bot::base_link':
                indice = i

        message = msg.Pose[indice]


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = BotPose()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()