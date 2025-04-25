from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # === Déclarations des arguments de lancement ===
    use_rviz = LaunchConfiguration('use_rviz')

    # === Chemins des fichiers partagés ===
    go2_bringup_share = get_package_share_directory('go2_bringup')
    rviz_config_path = os.path.join(go2_bringup_share, 'rviz', 'go2_default.rviz')

    return LaunchDescription([

        # === Arguments de lancement ===
        DeclareLaunchArgument('use_rviz', default_value='1', description='Lancer RViz (1 ou 0)'),

        # === Joystick ===
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # === Transport d'image (image_republish) ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('go2_image_transport'),
                    'launch',
                    'image_republish_launch.py'
                )
            ])
        ),

        # === Contrôle du robot ===
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

        # === TF & URDF ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('go2_description'),
                    'launch',
                    'display_robot.launch.py'
                )
            ])
        ),

        # === Localisation (tf_odom + lidar_fixer) ===
        Node(
            package='go2_description',
            executable='tf_odom',
            name='tf_odom',
            output='screen'
        ),

        Node(
            package='go2_description',
            executable='lidar_fixer',
            name='lidar_fixer',
            output='screen'
        ),

        # === RViz (optionnel via use_rviz) ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(use_rviz)
        )
    ])
