import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Pose

from .Balle import *

def nettoyer(tab_de_balles):
    nouv_tab=[]
    for i in range (len(tab_de_balles)):
        if i ==0:
            nouv_tab.append(tab_de_balles[i])
        else:
            ajout=True
            for ele in nouv_tab:
                if ele==tab_de_balles[i]:
                    ajout=False
            if ajout:
                nouv_tab.append(tab_de_balles[i])
    return nouv_tab

class Guidage(Node):

    def __init__(self):
        # Create node
        super().__init__('guidage')
        
        # Balls position subscriber
        self.subscription_balls = self.create_subscription(UInt16MultiArray,'/ball_positions',self.sub_balls_callback,10)
        self.subscription_balls # Avoid warning unused variable
        # Save balls position
        self.balles_pres = []

        # Safe zone position subscriber
        self.subscription_safezones = self.create_subscription(UInt16MultiArray,'/safezone_positions',self.sub_safezones_callback,10)
        self.subscription_safezones # Avoid warning unused variable
        # Save Safe zone position
        self.safezones_positions = np.array([])
        
        # Create ball positions publisher
        self.target_publisher = self.create_publisher(Pose, 'target', 10)



    def sub_balls_callback(self, array_msg):
        balles=np.array(array_msg.data).reshape((-1,2))
        balle_vue_ce_tour=[False]*len(self.balles_pres)
        for ele in balles:
            nouvelle_balle=Balle(ele[0],ele[1])
            ajout=True
            for i in range (len(self.balles_pres)):
                if self.balles_pres[i]==nouvelle_balle:
                    self.balles_pres[i].set_pose(nouvelle_balle.get_pose()[0],nouvelle_balle.get_pose()[1])
                    balle_vue_ce_tour[i]=True
                    ajout=False
            if ajout:
                balle_vue_ce_tour.append(True)
                self.balles_pres.append(nouvelle_balle)
        for i in range (len(self.balles_pres)):
            if balle_vue_ce_tour[i]:
                self.balles_pres[i].vieillir()
                self.balles_pres[i].tour_pas_vue=0
            else:
                self.balles_pres[i].tour_pas_vue+=1
        
        balle_a_garder=[]
        for ele in self.balles_pres:
            if ele.tour_pas_vue>3:
                balle_a_garder.append(False)
            else:
                balle_a_garder.append(True)
        balle_nouv=[]
        for i in range (len(balle_a_garder)):
            if balle_a_garder[i]:
                balle_nouv.append(self.balles_pres[i])
                print(self.balles_pres[i])
        self.balles_pres=balle_nouv
        self.balles_pres=nettoyer(self.balles_pres)
        print(len(self.balles_pres))
        self.publish_target()

    def sub_safezones_callback(self, array_msg):
        self.safezones_positions_matrix = np.array(array_msg.data).reshape((-1,2))



    def publish_target(self):
        try:
            balle_pose= self.balles_pres[0].get_pose()
            pose_msg = Pose()
            pose_msg.position.x = float(balle_pose[0])
            pose_msg.position.y = float(balle_pose[1])
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
