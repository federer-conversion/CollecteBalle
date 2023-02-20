#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np
import math

# Variable
x_target, y_target = None, None
x, y, yaw = None, None, None


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class controlSimple(Node):
    def __init__(self):
        super().__init__("controlSimple")

        self.subscription = self.create_subscription(
            PoseStamped, "robot_position", self.robot_position_callback, 10)
        self.subscription

        self.subscription2 = self.create_subscription(
            Pose, "target", self.target_callback, 10)
        self.subscription2

        self.publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.publisher

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

    def process(self):

        if x is None or x_target is None:
            return 1

        msg = Twist()
        k = 1
        k_linear = 1

        theta_voulu = np.arctan2(y_target - y, x_target - x)
        delta_theta = -theta_voulu - yaw
        e = 2 * np.arctan(np.tan(delta_theta / 2))

        msg.angular.z = k * e

        dx = x - x_target
        dy = y - y_target
        distance = np.sqrt(dx * dx + dy * dy)
        # msg.linear.x = k_linear * distance
        self.publisher.publish(msg)

    def robot_position_callback(self, msg):
        global x, y, yaw
        x = msg.pose.position.x
        y = msg.pose.position.y
        quat = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(quat)

    def target_callback(self, msg):
        global x_target, y_target
        x_target = msg.position.x
        y_target = msg.position.y


def main(args=None):
    rclpy.init(args=args)
    my_py_node = controlSimple()

    rclpy.spin(my_py_node)

    controlSimple.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
