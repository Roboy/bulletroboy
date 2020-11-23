import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(  'roboy_urdf',
                                default_value='',
                                description='Path to custom roboy URDF file.'),
        DeclareLaunchArgument(  'operator_urdf',
                                default_value='',
                                description='Path custom operator URDF file.'),
        DeclareLaunchArgument(  'cage_conf',
                                default_value='',
                                description='Path to the cage configuration XML.'),
        DeclareLaunchArgument(  'cage_mode',
                                default_value='debug',
                                description='- debug: uses pybullet debug force\n\t- tendon: uses tendon forces\n\t- forces: uses link forces'),
        Node(
            package='bulletroboy',
            executable='roboy_sim',
            arguments = [LaunchConfiguration('roboy_urdf')]
        ),
        Node(
            package='bulletroboy',
            executable='cage_sim',
            arguments = [LaunchConfiguration('operator_urdf'),
                        LaunchConfiguration('cage_conf'),
                        LaunchConfiguration('cage_mode')]
        ),
        Node(
            package='bulletroboy',
            executable='state_mapper',
            parameters = [os.path.join(
                get_package_share_directory('bulletroboy'),
                'config',
                'state_mapper.yaml'
        )]
        )
    ])
