from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare('mecanum_base')

    slam_params = PathJoinSubstitution([
        pkg_share,
        'config',
        'slam_toolbox_continue.yaml'
    ])

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
        remappings=[
            ('/scan', '/scan')
        ]
    )

    return LaunchDescription([
        slam_node
    ])
