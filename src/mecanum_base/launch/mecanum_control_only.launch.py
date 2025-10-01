from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Percorsi
    pkg_share = FindPackageShare('mecanum_base')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mecanum_robot.xacro'])
    controllers_path = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])

    # xacro → URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # Opzione A: robot_state_publisher (consigliata)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Nodo controller_manager
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            # Opzione B: se non vuoi usare robot_state_publisher, decommenta la riga sotto
            # {'use_robot_description_topic': False},
            controllers_path
        ],
        output='screen',
    )

    # Spawner controller
    spawner_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    spawner_mecanum = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_velocity_controller', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    # Ritarda gli spawner per dare tempo a ros2_control_node di avviarsi
    delayed_spawners = TimerAction(period=3.0, actions=[spawner_joint_state, spawner_mecanum])

    return LaunchDescription([
        robot_state_publisher_node,  # Opzione A
        ros2_control_node,
        delayed_spawners,
    ])
