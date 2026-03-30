from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('mecanum_base')

    slam_params = PathJoinSubstitution([
        pkg_share,
        'config',
        'slam_toolbox.yaml'
    ])

    # Nodo SLAM Toolbox in modalità mapping
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params]
    )

    return LaunchDescription([
        slam_node
    ])
