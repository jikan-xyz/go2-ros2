from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('go2_image_transport'),
                    'launch',
                    'image_republish_launch.py'
                )
            ])
        ),

        Node(
            package='go2_control',
            executable='teleop',
            name='teleop',
            output='screen'
        ),

        Node(
            package='go2_control',
            executable='move',
            name='move',
            output='screen'
        ),
    ])
