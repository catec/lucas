import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    ros_config_file = os.path.join(
        get_package_share_directory('cascade_pid_controller'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='cascade_pid_controller',
            executable='cascade_pid_controller_node',
            name='cascade_pid_controller_node',
            parameters=[ros_config_file],
            output='screen'
        )
    ])