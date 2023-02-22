import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose
import numpy as np
import math

from std_msgs.msg import Int64, Bool

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

        self.pub_is_blocker = self.create_publisher(Bool, '/is_blocked', 10)
        self.pub_is_blocker

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.process)
        self.get_logger().info(self.get_name() + " is launched")

        self.X_past = np.array([math.nan,math.nan])
        self.u_lin_past = math.nan
        self.robot_state = Drone_State.start
        self.comtpeur_bloc=0
        self.yaw_past = math.nan

        self.cnt_arriere = 0


    def process(self):
        print("etat",self.robot_state)
        msg2 = Twist()
        msg_blocked = Bool()
        
        k_theta = 2.6
        k_lin = 0.005    
        b_lin = 0.5
        
        if yaw != None and x_target!=None:
            ## Detection du blocage
            X_target = np.array([x_target,y_target])
            X_robot = np.array([x,y])

            vit = 0.
            diff_vit_cmd_max = 2.75
            if not math.isnan(self.X_past[0]):
                timer_period = 0.1
                vit = np.linalg.norm(X_robot - self.X_past)*timer_period

                print("vit = ", vit, " self.u_lin_past = ", self.u_lin_past)
                if vit<0.11 and self.robot_state != 6:
                    self.comtpeur_bloc+=1
                else:
                    self.comtpeur_bloc=0
                
                if self.comtpeur_bloc > 60 and self.robot_state != 1:
                    msg_blocked.data = True

                elif (abs(self.u_lin_past-vit) > diff_vit_cmd_max and self.robot_state != 1):
                    msg_blocked.data = True
                    # print("BLOCAGE")
                else:
                    msg_blocked.data = False
                

            ## Calculer de l'erreur
            X_err = X_target-X_robot

            
            ## Calcul de la commande
            u_lin = 0.
            u = 0.
            err_theta_start = 10*np.pi/180
            dist_stop=18
            if self.robot_state==3:
                dist_stop = 1
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
                u_lin = 10*(-vit) # Essayer de tourner sur place
                u = 2.6*err_theta
                # print("u_lin =", u_lin, " u =", u)
            print(self.robot_state)

            if dist <= dist_stop:
                print("Goal Reached")
                u_lin = 0.
                u = 0.
                msg_blocked.data = False
            else:
                u_lin = k_lin*dist + b_lin
                u = k_theta*err_theta


            if self.robot_state==5 or self.robot_state==6 : # Etat marche arriere
                if self.cnt_arriere < 4:
                    u_lin = -2.
                    u = 0.
                    self.cnt_arriere += 1
                elif self.cnt_arriere >= 4:
                    u_lin = 2.
                    u = 0.
                if not math.isnan(self.yaw_past):
                    u = 2.5*sawtooth(self.yaw_past - yaw)
                print("marche_arriere")
            if self.robot_state==1: # Etat start
                u_lin = -0.1 * self.u_lin_past
                u = 0.
                print("IDLE")

            # Memoire
            self.X_past = X_robot
            self.u_lin_past = u_lin
            self.yaw_past = yaw
            # print(u,u_lin)
            msg2.linear.x = u_lin
            msg2.angular.z = u

        self.publisher.publish(msg2)
        self.pub_is_blocker.publish(msg_blocked)

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
