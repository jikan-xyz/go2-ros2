import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Déclaration de l'argument image_transport
        DeclareLaunchArgument('image_transport', default_value='go2', description='Image transport type'),

        # Node pour republisher l'image, en utilisant 'go2' comme type d'entrée et 'raw' comme sortie
        Node(
            package='image_transport',
            executable='republish',
            name='image_republish_node',
            arguments=['go2'],  # Utilisation du transporteur personnalisé go2_sub
            output='screen',
            remappings=[
                ('in/go2', 'frontvideostream'),  # Topic d'entrée avec images compressées
                ('out', 'go2/camera')    # Publication des images brutes sur /go2/camera
            ]
        ),

        # LogInfo pour signaler que le lancement s'est bien déroulé
        LogInfo(
            msg="Image republish node started successfully"
        )
    ])
