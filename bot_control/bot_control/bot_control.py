import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np
import math

from std_msgs.msg import Int64

from enum import Enum

class Drone_State(Enum):
    start = 1
    change_zone = 2
    Go_to_ball = 3
    Go_to_safeZone = 4
    Go_out_safeZone = 5



# Variable
x_target, y_target = None, None
x, y, yaw = None, None, None

marche_arriere = 0.0

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
        super().__init__("control_simple")
        self.subscription = self.create_subscription(
            PoseStamped, "robot_position", self.robot_position_callback, 10)
        self.subscription

        self.subscription2 = self.create_subscription(
            Pose, "target", self.target_callback, 10)
        self.subscription2

        self.subs_marche_arriere = self.create_subscription(
            Int64, "fsm_state", self.fsm_state_callback, 10)
        self.subs_marche_arriere

        self.publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.publisher

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

        self.X_past = np.array([math.nan,math.nan])
        self.u_lin_past = math.nan
        self.robot_state = Drone_State.start


    def process(self):

        msg2 = Twist()
        
        k_theta = 2.6
        k_lin = 0.003    
        b_lin = 0.5
        

        if yaw != None and x_target!=None:
            ## Detection du blocage

            vit = 0.
            diff_vit_cmd_max = 10
            if not math.isnan(self.X_past[0]):
                timer_period = 0.1
                vit = np.linalg.norm(X_robot - self.X_past)*timer_period

                print("vit = ", vit, " self.u_lin_past = ", self.u_lin_past)
                if abs(self.u_lin_past-vit) > diff_vit_cmd_max:
                    print("BLOCAGE")
   
            ## Calculer de l'erreur
            X_target = np.array([x_target,y_target])
            X_robot = np.array([x,y])
            X_err = X_target-X_robot

            

            ## Calcul de la commande
            u_lin = 0.
            u = 0.
            err_theta_start = 10*np.pi/180
            dist_stop = 18

            # err angulaire
            def sawtooth(x):
                return (x+np.pi)%(2*np.pi)-np.pi
            w = -1*np.arctan2(X_err[1],X_err[0])
            err_theta = sawtooth(w - yaw)
            
            # err linéaire
            dist = np.linalg.norm(X_err)
    
            # commande
            # print("dist to goal = ", dist)
            if abs(err_theta) >= err_theta_start: #
                print("Turn to aim", " err_theta = ", err_theta*180/np.pi)
                # print("vitesse lin = ", vit, " X_actu = ", X_robot, " X_past = ", self.X_past)
                u_lin = 100*(-vit) # Essayer de tourner sur place
                u = 2.6*err_theta
                # print("u_lin =", u_lin, " u =", u)

            if dist <= dist_stop:
                print("Goad Reached")
                u_lin = 0.
                u = 0.
            else:
                u_lin = k_lin*dist + b_lin
                u = k_theta*err_theta

            if robot_state==5:
                u_lin = -1000000.
                u = 0.
                print("marche_arriere")
    
            # Memoire
            self.X_past = X_robot
            self.u_lin_past = u_lin
            # print(u,u_lin)
            msg2.linear.x = u_lin
            msg2.angular.z = u

        self.publisher.publish(msg2)

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

    def fsm_state_callback(self, msg):
        self.robot_state = msg.data


def main(args=None):
    rclpy.init(args=args)
    my_py_node = controlSimple()
    rclpy.spin(my_py_node)
    my_py_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
