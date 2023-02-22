import rclpy
from rclpy.node import Node

import numpy as np

from std_msgs.msg import UInt16MultiArray
from geometry_msgs.msg import Pose, PoseStamped
from enum import Enum

from .Balle import *

# path_H = np.array([[531.,531.92587191,532.92877696,534.00714496,535.15940572, 536.38398902,537.67932468,539.04384248,540.47597224,541.97414374,543.5367868,545.16233121,546.84920678,548.59584329,550.40067056,552.26211839,554.17861657,556.1485949,558.17048319,560.24271123,562.36370883,564.53190578,566.7457319,569.00361697,571.30399079,573.64528318,576.02592392,578.44434282,580.89896968,583.3882343,585.91056647,588.46439601,591.04815271,593.66026637,596.29916679,598.96328378,601.65104712,604.36088663,607.0912321,609.84051333,612.60716013,615.38960229,618.18626961,620.9955919,623.81599896,626.64592058,629.48378656,632.32802672,635.17707084,638.02934872,640.88329018,643.737325,646.58988299,649.43939395,652.28428767,655.12299397,657.95394264,660.77556347,663.58628628,666.38454086,669.16875701,671.93736453,674.68879323,677.42147289,680.13383333,682.82430434,685.49131573,688.13329729,690.74867882,693.33589013,695.89336102,698.41952128,700.91280072,703.37162913,705.79443632,708.17965209,710.52570623,712.83102855,715.09404886,717.31319694,719.48690259,721.61359563,723.69170585,725.71966305,727.69589703,729.61883759,731.48691453,733.29855766,735.05219676,736.74626165,738.37918213,739.94938798,741.45530902,742.89537504,744.26801585,745.57166125,746.80474102,747.96568499,749.05292294,750.06488468,751.], [151.,147.41615278,143.90546524,140.46791417,137.10347634,133.81212852,130.59384749,127.44861001,124.37639286,121.37717282,118.45092665,115.59763113,112.81726304,110.10979915,107.47521623,104.91349105,102.42460039,100.00852102,97.66522972,95.39470325,93.1969184,91.07185193,89.01948062,87.03978124,85.13273056,83.29830537,81.53648242,79.8472385,78.23055038,76.68639482,75.21474862,73.81558853,72.48889133,71.2346338,70.05279271,68.94334483,67.90626693,66.9415358,66.04912819,65.22902089,64.48119067,63.8056143,63.20226856,62.67113021,62.21217604,61.82538281,61.5107273,61.26818629,61.09773653,60.99935482,60.97301792,61.01870261,61.13638565,61.32604383,61.58765391,61.92119268,62.32663689,62.80396333,63.35314877,63.97416998,64.66700374,65.43162682,66.26801599,67.17614802,68.1559997,69.20754779,70.33076906,71.5256403,72.79213826,74.13023974,75.53992149,77.0211603,78.57393293,80.19821616,81.89398677,83.66122152,85.49989719,87.40999055,89.39147839,91.44433746,93.56854454,95.76407641,98.03090984,100.36902161,102.77838848,105.25898723,107.81079463,110.43378747,113.1279425,115.8932365,118.72964625,121.63714853,124.61572009,127.66533773,130.7859782,133.97761829,137.24023476,140.5738044,143.97830397,147.45371024,151.]])
# path_B = np.array([[531.,531.92587191,532.92877696,534.00714496,535.15940572,536.38398902,537.67932468,539.04384248,540.47597224,541.97414374,543.5367868,545.16233121,546.84920678,548.59584329,550.40067056,552.26211839,554.17861657,556.1485949,558.17048319,560.24271123,562.36370883,564.53190578,566.7457319,569.00361697,571.30399079,573.64528318,576.02592392,578.44434282,580.89896968,583.3882343,585.91056647,588.46439601,591.04815271,593.66026637,596.29916679,598.96328378,601.65104712,604.36088663,607.0912321,609.84051333,612.60716013,615.38960229,618.18626961,620.9955919,623.81599896,626.64592058,629.48378656,632.32802672,635.17707084,638.02934872,640.88329018,643.737325,646.58988299,649.43939395,652.28428767,655.12299397,657.95394264,660.77556347,663.58628628,666.38454086,669.16875701,671.93736453,674.68879323,677.42147289,680.13383333,682.82430434,685.49131573,688.13329729,690.74867882,693.33589013,695.89336102,698.41952128,700.91280072,703.37162913,705.79443632,708.17965209,710.52570623,712.83102855,715.09404886,717.31319694,719.48690259,721.61359563,723.69170585,725.71966305,727.69589703,729.61883759,731.48691453,733.29855766,735.05219676,736.74626165,738.37918213,739.94938798,741.45530902,742.89537504,744.26801585,745.57166125,746.80474102,747.96568499,749.05292294,750.06488468,751.],[575.,578.58384722,582.09453476,585.53208583,588.89652366,592.18787148,595.40615251,598.55138999,601.62360714,604.62282718,607.54907335,610.40236887,613.18273696,615.89020085,618.52478377,621.08650895,623.57539961,625.99147898,628.33477028,630.60529675,632.8030816,634.92814807,636.98051938,638.96021876,640.86726944,642.70169463,644.46351758,646.1527615,647.76944962,649.31360518,650.78525138,652.18441147,653.51110867,654.7653662,655.94720729,657.05665517,658.09373307,659.0584642,659.95087181,660.77097911,661.51880933,662.1943857,662.79773144,663.32886979,663.78782396,664.17461719,664.4892727,664.73181371,664.90226347,665.00064518,665.02698208,664.98129739,664.86361435,664.67395617,664.41234609,664.07880732,663.67336311,663.19603667,662.64685123,662.02583002,661.33299626,660.56837318,659.73198401,658.82385198,657.8440003,656.79245221,655.66923094,654.4743597,653.20786174,651.86976026,650.46007851,648.9788397,647.42606707,645.80178384,644.10601323,642.33877848,640.50010281,638.59000945,636.60852161,634.55566254,632.43145546,630.23592359,627.96909016,625.63097839,623.22161152,620.74101277,618.18920537,615.56621253,612.8720575,610.1067635,607.27035375,604.36285147,601.38427991,598.33466227,595.2140218,592.02238171,588.75976524,585.4261956,582.02169603,578.54628976,575.]])
path_H = [[531, 151],[641, 61],[751, 151]]
path_B = [[531, 575],[641, 665],[751, 575]]
indice_suivi = 0

class Drone_State(Enum):
    start = 1
    change_zone = 2
    Go_to_ball = 3
    Go_to_safeZone = 4

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


def nettoyer(tab_de_balles):
    nouv_tab = []
    for i in range(len(tab_de_balles)):
        if i == 0:
            nouv_tab.append(tab_de_balles[i])
        else:
            ajout = True
            for ele in nouv_tab:
                if ele == tab_de_balles[i]:
                    ajout = False
            if ajout:
                nouv_tab.append(tab_de_balles[i])
    return nouv_tab


def cost_fnct(pos_robot, zone, balle, K_d=1, K_a=4, K_z=1):
    balle_pose = balle.get_pose()
    cost_dist = ((balle_pose[0]*1.0-pos_robot[0])**2 +
                 (balle_pose[1]*1.0-pos_robot[1])**2)**0.5
    cost_age = balle.age
    cost_zone = ((pos_robot[0]*1.0-zone[0])**2 +
                 (balle_pose[1]*1.0-zone[1])**2)**0.5
    total_cos = K_d*cost_dist+K_a*cost_age + K_z*cost_zone
    return max(total_cos/100., 0.)


class Guidage(Node):

    def __init__(self):
        # Create node
        super().__init__('guidage')
        self.declare_parameter('display_mode', False)
        self.debug_mode = self.get_parameter(
            'display_mode').get_parameter_value().bool_value
        
        # Variable
        self.x, self.y, self.yaw = None, None, None
        self.target_ball = [None, None]
        self.target = [None, None]
        self.robot_state = Drone_State.start

        self.change_zone = True
        self.capture_ball = False
        self.in_safezone = False

        # Balls position subscriber
        self.subscription_balls = self.create_subscription(
            UInt16MultiArray, '/ball_positions', self.sub_balls_callback, 10)
        self.subscription_balls  # Avoid warning unused variable
        # Save balls position
        self.balles_pres = []
        self.safezones_positions_matrix = np.array([[0, 0], [0, 0]])

        # Safe zone position subscriber
        self.subscription_safezones = self.create_subscription(
            UInt16MultiArray, '/safezone_positions', self.sub_safezones_callback, 10)
        self.subscription_safezones  # Avoid warning unused variable
        # Save Safe zone position
        self.safezones_positions = np.array([])

        # Robot position subscriber
        self.subscription_robot_pos = self.create_subscription(
            PoseStamped, "robot_position", self.robot_position_callback, 10)
        self.subscription_robot_pos

        # Create ball positions publisher
        self.target_publisher = self.create_publisher(Pose, 'target', 10)
    
    def robot_position_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        quat = msg.pose.orientation
        roll, pitch, self.yaw = euler_from_quaternion(quat)

    def sub_balls_callback(self, array_msg):
        global target_ball
        balles = np.array(array_msg.data).reshape((-1, 2))
        balle_vue_ce_tour = [False]*len(self.balles_pres)
        for ele in balles:
            nouvelle_balle = Balle(ele[0], ele[1])
            ajout = True
            for i in range(len(self.balles_pres)):
                if self.balles_pres[i] == nouvelle_balle:
                    self.balles_pres[i].set_pose(nouvelle_balle.get_pose()[
                                                 0], nouvelle_balle.get_pose()[1])
                    balle_vue_ce_tour[i] = True
                    ajout = False
            if ajout:
                balle_vue_ce_tour.append(True)
                self.balles_pres.append(nouvelle_balle)
        for i in range(len(self.balles_pres)):
            if balle_vue_ce_tour[i]:
                self.balles_pres[i].vieillir()
                self.balles_pres[i].tour_pas_vue = 0
            else:
                self.balles_pres[i].tour_pas_vue += 1

        balle_a_garder = []
        for ele in self.balles_pres:
            if ele.tour_pas_vue > 3:
                balle_a_garder.append(False)
            else:
                balle_a_garder.append(True)
        balle_nouv = []
        for i in range(len(balle_a_garder)):
            if balle_a_garder[i]:
                balle_nouv.append(self.balles_pres[i])
                if self.debug_mode:
                    print(self.balles_pres[i])
        self.balles_pres = balle_nouv
        self.balles_pres = nettoyer(self.balles_pres)
        if self.debug_mode:
            print("en tout", len(self.balles_pres), "balles")
        ind_min, val_min = -1, 1000000000
        for ind, balle_pres in enumerate(self.balles_pres):
            if self.debug_mode:
                print(balle_pres.age)
            if balle_pres.age > 10:
                cost = min(cost_fnct((100, 100), self.safezones_positions_matrix[0], balle_pres), cost_fnct(
                    (100, 100), self.safezones_positions_matrix[1], balle_pres))
                if self.debug_mode:
                    print(cost)
                if cost < val_min:
                    val_min, ind_min = cost, ind

        if ind_min == -1:
            self.target_ball = [0, 0]
        else:
            self.target_ball = self.balles_pres[ind_min].get_pose()
        
        self.target_ball = [1200, 300]

        self.publish_target()

    def sub_safezones_callback(self, array_msg):
        self.safezones_positions_matrix = np.array(
            array_msg.data).reshape((-1, 2))

    def publish_target(self):
        if self.x is not None:
            print(self.robot_state)
            print(self.x, self.target_ball[0])
            self.action_state()
            self.update_state()

        if self.target[0] is not None:
            pose_msg = Pose()
            pose_msg.position.x = float(self.target[0])
            pose_msg.position.y = float(self.target[1])
            self.target_publisher.publish(pose_msg)
        

    def update_state(self):
        global indice_suivi
        if self.robot_state == Drone_State.start:
            self.robot_state = Drone_State.change_zone
            indice_suivi = 0
        elif (self.robot_state == Drone_State.change_zone and not self.change_zone):
            self.change_zone = True
            self.robot_state = Drone_State.Go_to_ball
        elif (self.robot_state == Drone_State.Go_to_ball and self.capture_ball):
            self.robot_state = Drone_State.Go_to_safeZone
        elif (self.robot_state == Drone_State.Go_to_safeZone and self.in_safezone):
            self.robot_state = Drone_State.start

    def action_state(self):
        if self.robot_state == Drone_State.start:
            if (self.x < 641 and self.target_ball[0] < 641) or (self.x > 641 and self.target_ball[0] > 641):
                self.change_zone = False
            elif (self.x > 641 and self.target_ball[0] < 641) or (self.x < 641 and self.target_ball[0] > 641):
                self.change_zone = True
            else:
                self.change_zone = True
                print("erreur cas imprévu")
        
        elif self.robot_state == Drone_State.change_zone:
            global indice_suivi
            if self.x < 641 and self.target_ball[0] > 641:
                if self.y < 360 : 
                    if(self.x > path_H[0][0] - 54):
                        if(self.y < path_H[0][1] + 54): 
                            indice_suivi = 1
                    if(self.x > path_H[1][0] - 54):
                        if(self.y < path_H[1][1]+20):
                            indice_suivi = 2
                    self.target = path_H[indice_suivi]
                else:
                    if(self.x > path_B[0][0] - 54):
                        if(self.y < path_B[0][1] + 54): 
                            indice_suivi = 1
                    if(self.x > path_B[1][0] - 54):
                        if(self.y < path_B[1][1]+20):
                            indice_suivi = 2
                    self.target = path_B[indice_suivi]
            elif self.x > 641 and self.target_ball[0] < 641:
                if self.y < 360:
                    print("suivre path_H (sens oppo)")
                    print(self.target_ball, path_H[-1])
                else:
                    print("suivre path_B (sens oppo)")
                    print(self.target_ball, path_B[-1])
            else:
                self.change_zone = False
        
        elif self.robot_state == Drone_State.Go_to_ball:
            self.target = self.target_ball
        


def main(args=None):
    pos_robot = (100, 100)
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
