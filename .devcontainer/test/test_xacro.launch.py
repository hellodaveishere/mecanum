from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import subprocess

def generate_launch_description():
    # Percorso assoluto al file di test
    urdf_file = os.path.abspath("test.urdf.xacro")
    print(f"[DEBUG] urdf_file = {urdf_file}")

    # Esegui xacro in Python e cattura l'output URDF
    urdf_text = subprocess.check_output(
        ["/opt/ros/jazzy/bin/xacro", urdf_file]
    ).decode('utf-8')

    # Nodo minimale: robot_state_publisher con URDF già generato
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': urdf_text
        }],
        output='screen'
    )

    return LaunchDescription([rsp_node])
