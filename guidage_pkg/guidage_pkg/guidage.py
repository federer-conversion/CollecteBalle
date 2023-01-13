import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Pose

class Guidage(Node):

    def __init__(self):
        # Create node
        super().__init__('guidage')
        
        # Balls position subscriber
        self.subscription_balls = self.create_subscription(UInt16MultiArray,'/ball_positions',self.sub_balls_callback,10)
        self.subscription_balls # Avoid warning unused variable
        # Save balls position
        self.ball_positions_matrix = np.array([])

        # Safe zone position subscriber
        self.subscription_safezones = self.create_subscription(UInt16MultiArray,'/safezone_positions',self.sub_safezones_callback,10)
        self.subscription_safezones # Avoid warning unused variable
        # Save Safe zone position
        self.safezones_positions_matrix = np.array([])
        
        # Create ball positions publisher
        self.target_publisher = self.create_publisher(Pose, 'target', 10)

    def sub_balls_callback(self, array_msg):
        self.ball_positions_matrix = np.array(array_msg.data).reshape((-1,2))
        self.publish_target()

    def sub_safezones_callback(self, array_msg):
        self.safezones_positions_matrix = np.array(array_msg.data).reshape((-1,2))

    def publish_target(self):
        try:
            x,y = self.ball_positions_matrix[0]
            pose_msg = Pose()
            pose_msg.position.x = float(x)
            pose_msg.position.y = float(y)
            self.target_publisher.publish(pose_msg)
        except:
            pass



def main(args=None):
    # Init
    rclpy.init(args=args)
    
    # Create the image parser
    guidage = Guidage()
    
    # Loop
    rclpy.spin(guidage)
    
    # Kill properly the programs
    guidage.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
