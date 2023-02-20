[![tennis-court-foxy](https://github.com/nathan-teaching/CollecteBalle/actions/workflows/tennis_court-foxy.yaml/badge.svg)](https://github.com/nathan-teaching/CollecteBalle/actions/workflows/tennis_court-foxy.yaml)
# Tennis Ball Collector

Ceci est un template de dépôt Git pour le cours d'ingénierie système et modélisation robotique à l'ENSTA Bretagne en 2023.

## Lancer la simulation

### Dépendences

Install ROS2 foxy :  
https://docs.ros.org/en/foxy/Installation.html

Install Gazebo :
```bash
sudo apt install ros-foxy-gazebo-ros
# sudo apt install ros-foxy-gazebo-* # if the previous line is not enough
```

Install ros package :
```bash
sudo apt-get install ros-foxy-rqt-robot-steering
sudo apt-get install ros-foxy-robot-localization
sudo apt-get install ros-foxy-rqt-robot-xacro
```

Install Python3, OpenCV and other required package :
```bash
sudo apt install python3
pip install cv2
pip install evdev
```

Install ros2_control :

```bash
sudo apt-get install ros-foxy-controller-manager
sudo apt-get install ros-foxy-gazebo-ros2-control
sudo apt-get install ros-foxy-ros2-control
```

### Clone the repository

In the `src/` directory of your ROS2 workspace clone the git repository :
```bash
git clone https://github.com/federer-conversion/CollecteBalle.git
```

### Build

In the root of your ROS2 workspace run :
```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select tennis_court robot_description process_camera_pkg guidage_pkg remote_controller_pkg
# ou colcon build
source install/setup.bash
```

### Launch

In a terminal :
Be sure you have already sourced, if not :
```bash
source /opt/ros/foxy/setup.bash # if your foxy is not sourced
source install/setup.bash # ifyour workspace is not sourced and if your are in the root of your worksapce
```

The launch the code :
```bash
ros2 launch robot_description simulation.launch.py
```

In another terminal, if you want to control the robot with a controller, run in the root of your ROS2 workspace:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run remote_controller_pkg remote_control_robot
```

Now enjoy the tennis court with the robot.

If you want to the the output of the image processing, in another terminal run in the root of your ROS2 workspace:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run process_camera_pkg process_camera_img --ros-args -p display_mode:=True
```

If you want to actuate grasper to retract use this command :
```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "data:
- <value>"
# Change value for example 0.1 in rad/s
```

If you want to the the output of the guidance, in another terminal run in the root of your ROS2 workspace:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run guidage_pkg guidage --ros-args -p display_mode:=True
```

### launch if `ros2 launch robot_description simulation.launch.py` is not working : 
In a first terminal, run in the root of your ROS2 workspace:
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch robot_description display.launch.py
```

In a second terminal, run in the root of your ROS2 workspace:
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch tennis_court tennis_court.launch.py
```
<details>



## Groupe

### Membres

* Damien Esnault
* Mirado Rajaomarosata
* Nicolas Defour
* Maël Godard
* Hugo Yverneau 

### Data sheet

Vous trouverez les documentations de communication entre client et équipes ici :
- Design du robot : [robot_design](data_sheet/robot_design.jpg)
- Chiffage prix : [chiffrage](data_sheet/prix.txt)
- Documentation technique : [Doc tech](data_sheet/doc_tec.txt)



### Gestion de projet

https://tree.taiga.io/project/d_snlt_work_account-federer-reconversions/

## Structure du dépôt

Ce dépôt doit être cloné dans le dossier `src` d'un workspace ROS 2.

### Package `tennis_court`

Le dossier `tennis_court` est un package ROS contenant le monde dans lequel le robot ramasseur de balle devra évoluer ainsi qu'un script permettant de faire apparaître des balles dans la simulation.
Ce package ne doit pas être modifié.
Consulter le [README](tennis_court/README.md) du package pour plus d'informations.


### Documents

Le dossier `docs` contient tous les documents utiles au projet:
- Des [instructions pour utiliser Git](docs/GitWorkflow_fork.md)
- Un [Mémo pour ROS 2 et Gazebo](docs/Memo_ROS2.pdf)
- Les [slides de la présentation Git](docs/GitPresentation.pdf)

Pour faire bouger des pièces entre elles : [gazebo_ros2_control](https://github.com/ros-controls/gazebo_ros2_control)


### Rapports

Le dossier `reports` doit être rempli avec les rapports d'[objectifs](../reports/GoalsTemplate.md) et de [rétrospectives](../reports/DebriefTemplate.md) en suivant les deux templates mis à disposition. Ces deux rapports doivent être rédigés respectivement au début et à la fin de chaque sprint.
