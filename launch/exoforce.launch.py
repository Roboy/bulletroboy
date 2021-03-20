import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(  'cage_conf',
                                default_value='',
                                description='Path to the cage configuration XML.'),
        Node(
            package='bulletroboy',
            executable='state_mapper',
            parameters = [os.path.join(
                get_package_share_directory('bulletroboy'),
                'config',
                'conf.yaml')]
        ),
        Node(
            package='bulletroboy',
            executable='operator',
            parameters = [os.path.join(
                get_package_share_directory('bulletroboy'),
                'config',
                'conf.yaml')]
        ),
        Node(
            package='bulletroboy',
            executable='exoforce',
            arguments = [LaunchConfiguration('cage_conf')],
            parameters = [os.path.join(
                get_package_share_directory('bulletroboy'),
                'config',
                'conf.yaml')]
        )
    ])
