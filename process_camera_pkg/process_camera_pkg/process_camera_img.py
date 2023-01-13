import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image
from std_msgs.msg import UInt16MultiArray


class ImageParser(Node):

    def __init__(self):
        # Create node
        super().__init__('image_parser')
        self.declare_parameter('display_mode',False)
        self.debug_mode = self.get_parameter('display_mode').get_parameter_value().bool_value
        
        # Create camera subscriber
        self.subscription = self.create_subscription(Image,'/zenith_camera/image_raw',self.get_image_callback,10)
        self.subscription # Avoid warning unused variable
        
        # Create ball positions publisher
        self.ball_publisher = self.create_publisher(UInt16MultiArray, 'ball_positions', 10)
        self.ball_publisher

        # Create safezone positions publisher
        self.safezone_publisher = self.create_publisher(UInt16MultiArray, 'safezone_positions', 10)
        self.safezone_publisher

        # Create variable to store image
        self.image = np.zeros((240,240,3))
        
        # Create variable to save position of ball
        self.ball_positions = []
        self.safezone_positions = []
        
        # Create variable to parse HSV frame
        self.ball_low_HSV = (29,0,0)
        self.ball_high_HSV = (31,255,255)
        self.safezone_low_HSV = (13,0,0)
        self.safezone_high_HSV = (16,255,255)

    def get_image_callback(self, img_msg):
        # Note: get encoding but for our case its rbg8
        height, width, encoding = img_msg.height, img_msg.width, img_msg.encoding
        
        # Store image and convert it in BGR
        image_rgb = np.array(img_msg.data).reshape((height,width,3))
        self.image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # Call the method to process the image
        self.detect_ball_in_image()
    
    def detect_ball_in_image(self):
        # Clear the position lists
        self.ball_positions.clear()
        self.safezone_positions.clear()

        # Convert image to HSV
        image_HSV = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # Get binary image with balls isolated
        image_ball = cv2.inRange(image_HSV, self.ball_low_HSV, self.ball_high_HSV)
        # Get binary image with safe zones isolated
        image_safezone = cv2.inRange(image_HSV, self.safezone_low_HSV, self.safezone_high_HSV)

        # Classified and localized balls
        cnts = cv2.findContours(image_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            self.ball_positions.append(int(x+(w/2)))
            self.ball_positions.append(int(y+(h/2)))
            if self.debug_mode:
                cv2.rectangle(self.image, (x, y), (x + w, y + h), (0,0,255), 2)

        # Classified and localized safezones
        cnts = cv2.findContours(image_safezone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            self.safezone_positions.append(int(x+(w/2)))
            self.safezone_positions.append(int(y+(h/2)))
            if self.debug_mode:
                cv2.rectangle(self.image, (x, y), (x + w, y + h), (0,255,0), 2)

        # Publish the balls positions
        ball_positions_msg = UInt16MultiArray()
        ball_positions_msg.data = self.ball_positions
        self.ball_publisher.publish(ball_positions_msg)

        # Publish the safezone positions
        safezone_positions_msg = UInt16MultiArray()
        safezone_positions_msg.data = self.safezone_positions
        self.safezone_publisher.publish(safezone_positions_msg)

        # Display to the user
        if self.debug_mode:
            cv2.imshow("Balle(s)",image_ball)
            cv2.imshow("Safe zones",image_safezone)
            cv2.imshow("Image",self.image)
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
