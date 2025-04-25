from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Définir le chemin du fichier URDF pour la description du robot
    urdf_file = os.path.join(
        get_package_share_directory('go2_description'), 'urdf', 'go2_description.urdf'
    )

    # Lire le contenu du fichier URDF
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()

    # Passer le contenu du fichier URDF directement comme paramètre robot_description
    rsp_params = {'robot_description': urdf_content}

    return LaunchDescription([
        # Lancer robot_state_publisher pour publier la description du robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[rsp_params],  # Publier robot_description directement à partir du contenu URDF
        ),
    ])
