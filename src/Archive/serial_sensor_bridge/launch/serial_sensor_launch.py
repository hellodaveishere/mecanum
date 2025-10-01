import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('serial_sensor_bridge')
    config = os.path.join(pkg_share, 'config', 'sensors_params.yaml')

    return LaunchDescription([
        Node(
            package='serial_sensor_bridge',
            executable='serial_sensor_node',
            name='serial_sensor_node',
            output='screen',
            parameters=[config]
        )
    ])
