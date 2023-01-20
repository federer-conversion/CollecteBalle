import launch

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import launch_ros.actions

from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    tennis_court_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/tennis_court.launch.py']),
    )
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/display.launch.py']),
    )
    
    camera_node = launch_ros.actions.Node(
            package='process_camera_pkg',
            executable='process_camera_img',
            name='process_camera_img')
    
    guidage_node = launch_ros.actions.Node(
            package='guidage_pkg',
            executable='guidage',
            name='guidage')
     
    return launch.LaunchDescription([
        display_launch,
        tennis_court_launch,
        camera_node,
        guidage_node
    ])
