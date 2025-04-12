import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the directory of the Nav2 Bringup package and the custom configuration
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    robot_config_dir = get_package_share_directory('robot_config')

    return LaunchDescription([
        # Nav2 Navigation Stack (planner, regulated pure pursuit controller, costmaps)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'params_file': os.path.join(robot_config_dir, 'robot_config', 'nav2_params.yaml'), 'slam': 'True'  # Custom params file
            }.items(),
        ),
    ])
