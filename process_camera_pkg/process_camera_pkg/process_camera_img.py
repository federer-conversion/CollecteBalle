import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import matplotlib.pyplot as plt
import math

from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def ros_quaternion_from_euler(ai, aj, ak):
    ros_quaternion = Quaternion()
    ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w = quaternion_from_euler(
        ai, aj, ak)
    return ros_quaternion


class ImageParser(Node):

    def __init__(self):
        # Create node
        super().__init__('image_parser')
        self.declare_parameter('display_mode', False)
        self.debug_mode = self.get_parameter(
            'display_mode').get_parameter_value().bool_value

        # Create camera subscriber
        self.subscription = self.create_subscription(
            Image, '/zenith_camera/image_raw', self.get_image_callback, 10)
        self.subscription  # Avoid warning unused variable

        # Create ball positions publisher
        self.ball_publisher = self.create_publisher(
            UInt16MultiArray, 'ball_positions', 10)
        self.ball_publisher

        # Create safezone positions publisher
        self.safezone_publisher = self.create_publisher(
            UInt16MultiArray, 'safezone_positions', 10)
        self.safezone_publisher

        # Create robot position publisher
        self.robot_publisher = self.create_publisher(
            PoseStamped, 'robot_position', 10)
        self.robot_publisher

        # Create variable to store image
        self.image = np.zeros((240, 240, 3))

        # Create variable to save position of ball, safezone_positions, robot_position
        self.ball_positions = []
        self.safezone_positions = []
        self.robot_position = []
        self.pince_position = []

        # Create variable to parse HSV frame
        self.ball_low_HSV = (29, 0, 0)
        self.ball_high_HSV = (31, 255, 255)
        self.safezone_low_HSV = (13, 0, 0)
        self.safezone_high_HSV = (16, 255, 255)
        self.robot_low_HSV = (59, 0, 0)
        self.robot_high_HSV = (60.5, 255, 255)
        self.pince_low_HSV = (0, 10, 98)
        self.pince_high_HSV = (1, 255, 255)
        self.pince_low_HSV_bis = (170, 10, 10)
        self.pince_high_HSV_bis = (180, 255, 255)

    def get_image_callback(self, img_msg):
        # Note: get encoding but for our case its rbg8
        height, width, encoding = img_msg.height, img_msg.width, img_msg.encoding

        # Store image and convert it in BGR
        image_rgb = np.array(img_msg.data).reshape((height, width, 3))
        self.image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

        # Call the method to process the image
        self.detect_ball_in_image()

    def detect_ball_in_image(self):
        # Clear the position lists
        self.ball_positions.clear()
        self.safezone_positions.clear()
        self.robot_position.clear()
        self.pince_position.clear()

        # Convert image to HSV
        image_HSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # Get binary image with balls isolated
        image_ball = cv2.inRange(
            image_HSV, self.ball_low_HSV, self.ball_high_HSV)
        # Use opening morphology to avoid false detection
        kernel = np.ones((3, 3), np.uint8)
        image_ball = cv2.morphologyEx(image_ball, cv2.MORPH_OPEN, kernel)

        # Get binary image with safe zones isolated
        image_safezone = cv2.inRange(
            image_HSV, self.safezone_low_HSV, self.safezone_high_HSV)
        # Use opening morphology to avoid false detection
        kernel = np.ones((3, 3), np.uint8)
        image_safezone = cv2.morphologyEx(
            image_safezone, cv2.MORPH_OPEN, kernel)

        # Get binary image with robot isolated
        image_robot = cv2.inRange(
            image_HSV, self.robot_low_HSV, self.robot_high_HSV)
        image_pince = cv2.inRange(
            image_HSV, self.pince_low_HSV, self.pince_high_HSV)+cv2.inRange(
            image_HSV, self.pince_low_HSV_bis, self.pince_high_HSV_bis)
        tab_inter=np.argwhere(image_pince>0)
        try:
            minlig,maxlig=np.min(tab_inter[:,0]),np.max(tab_inter[:,0])
            mincol,maxcol=np.min(tab_inter[:,1]),np.max(tab_inter[:,1])
            for i in range(minlig,maxlig):
                for j in range (mincol,maxcol):
                    image_pince[i,j]=1
        except Exception as e:
            print(e)
        image_ball_int=image_ball.copy()
        image_ball[image_pince==1]=0
        image_ball_int[image_pince==0]=0
        # image_ball[image_pince]=1
        # Classified and localized balls
        cnts = cv2.findContours(
            image_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cnts_int = cv2.findContours(
            image_ball_int, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_int = cnts_int[0] if len(cnts_int) == 2 else cnts_int[1]
        print(cnts_int)
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            self.ball_positions.append(int(x+(w/2)))
            self.ball_positions.append(int(y+(h/2)))
            if self.debug_mode:
                cv2.rectangle(self.image, (x, y),
                              (x + w, y + h), (0, 0, 255), 2)

        # Classified and localized safezones
        cnts = cv2.findContours(
            image_safezone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            self.safezone_positions.append(int(x+(w/2)))
            self.safezone_positions.append(int(y+(h/2)))
            if self.debug_mode:
                cv2.rectangle(self.image, (x, y),
                              (x + w, y + h), (0, 255, 0), 2)

        # Classified and localized robot
        cnts = cv2.findContours(
            image_robot, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            if (w > 10 and h > 15):
                self.robot_position.append(x+(w/2))
                self.robot_position.append(y+(h/2))
                if self.debug_mode:
                    cv2.rectangle(self.image, (x, y),
                                  (x + w, y + h), (255, 0, 0), 2)
            else:
                if self.debug_mode:
                    print("erreur de détection de robot")

        # Classified and localized pince
        cnts = cv2.findContours(
            image_pince, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            # if(w>20 and h>20):
            self.pince_position.append(x+(w/2))
            self.pince_position.append(y+(h/2))
            if self.debug_mode:
                cv2.rectangle(self.image, (x, y),
                              (x + w, y + h), (255, 0, 0), 2)

        # robot heading
        if (len(self.robot_position) > 0 and len(self.pince_position) > 1):
            self.robot_position[0] = (
                self.robot_position[0] + self.pince_position[0])/2
            self.robot_position[1] = (
                self.robot_position[1] + self.pince_position[1])/2
            orientation = math.atan2(
                self.robot_position[1]-self.pince_position[1], self.pince_position[0]-self.robot_position[0])

        # Publish the balls positions
        ball_positions_msg = UInt16MultiArray()
        ball_positions_msg.data = self.ball_positions
        self.ball_publisher.publish(ball_positions_msg)

        # Publish the safezone positions
        safezone_positions_msg = UInt16MultiArray()
        safezone_positions_msg.data = self.safezone_positions
        self.safezone_publisher.publish(safezone_positions_msg)

        # Publish the robot position
        robot_position_msg = PoseStamped()
        if (len(self.robot_position) > 0 and len(self.pince_position) > 1):
            robot_position_msg.pose.position.x = self.robot_position[0]
            robot_position_msg.pose.position.y = self.robot_position[1]
            robot_position_msg.pose.orientation = ros_quaternion_from_euler(
                0.0, 0.0, orientation)
            self.robot_publisher.publish(robot_position_msg)

        # Display to the user
        if self.debug_mode:
            cv2.imshow("Balle(s)", image_ball)
            cv2.imshow("Safe zones", image_safezone)
            image_robot = image_robot+image_pince
            cv2.imshow("Robot", image_robot)
            cv2.imshow("Image", self.image)
            cv2.waitKey(1)


def main(args=None):
    # Init
    rclpy.init(args=args)

    # Create the image parser
    image_parser = ImageParser()

    # Loop
    rclpy.spin(image_parser)

    # Kill properly the programs
    cv2.destroyAllWindows()
    image_parser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
