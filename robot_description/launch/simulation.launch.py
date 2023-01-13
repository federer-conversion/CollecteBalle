import launch

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    tennis_court_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/tennis_court.launch.py']),
    )
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/display.launch.py']),
    )
    return launch.LaunchDescription([
        display_launch,
        tennis_court_launch,
    ])
