from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    package_path = get_package_share_directory('catec_control_manager')
    
    config_file = os.path.join(package_path, 'config', 'params.yaml')
    general_params_file = os.path.join(package_path, 'config', 'general_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to the configuration file'
        ),

        Node(
            package='catec_control_manager',
            executable='control_manager_node',
            name='catec_control_manager',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {'parameters_file_path': general_params_file}
            ]
        ),
    ])