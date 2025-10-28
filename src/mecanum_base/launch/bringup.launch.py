from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # =========================
    # 🔧 Argomenti configurabili
    # =========================
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Abilita o disabilita RViz alla partenza'
    )
    rviz_enabled = LaunchConfiguration('rviz')

    # =========================
    # 📂 Percorsi ai file del robot mecanum
    # =========================
    pkg_share = FindPackageShare('mecanum_base')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mecanum_robot.xacro'])
    controllers_path = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    rviz_path = PathJoinSubstitution([pkg_share, 'rviz', 'mecanum.rviz'])
    webserver_path = os.path.join(get_package_share_directory('mecanum_base'), 'webserver')

    # =========================
    # 🔄 Conversione Xacro → URDF
    # =========================
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),  # ⚠️ spazio dopo "xacro" necessario
        value_type=str
    )

    # =========================
    # 📡 Nodo per pubblicare lo stato del robot
    # =========================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # =========================
    # ⚙️ Nodo principale ros2_control
    # =========================
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers_path],
        output='screen',
    )

    # =========================
    # ⚙️ Nodo rosbridge_server per comunicazione WebSocket
    # =========================
    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'max_message_size': 100000000},
            {'port': 9091}
        ]
    )

    # =========================
    # 🌐 Web server Node.js per interfaccia web
    # =========================
    webserver_node = ExecuteProcess(
        cmd=['node', 'server.js'],
        cwd=webserver_path,
        output='screen'
    )

    # =========================
    # 🎛️ Spawner dei controller e broadcaster
    # =========================
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

    spawner_imu = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    spawner_ir_front_left = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ir_front_left_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    spawner_ir_front_center = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ir_front_center_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    spawner_ir_front_right = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ir_front_right_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    spawner_servo_position = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['servo_position_controller', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    spawner_battery = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['battery_state_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    # ⏱️ Ritardo per dare tempo a ros2_control_node di inizializzarsi
    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            spawner_joint_state,
            spawner_mecanum,
            spawner_imu,
            spawner_ir_front_left,
            spawner_ir_front_center,
            spawner_ir_front_right,
            spawner_servo_position,
            spawner_battery  # ✅ Avvio sincronizzato del broadcaster batteria
        ]
    )

    # =========================
    # 🚗 Nodi applicativi custom del robot mecanum
    # =========================
    mecanum_cmd_node = Node(
        package='mecanum_base',
        executable='mecanum_cmd_node',
        name='mecanum_cmd_node',
        output='screen',
        parameters=[controllers_path],
    )

    mecanum_odom_node = Node(
        package='mecanum_base',
        executable='mecanum_odom_node',
        name='mecanum_odom_node',
        output='screen',
        parameters=[controllers_path],
    )

    # =========================
    # 📡 EKF (filtro di localizzazione)
    # =========================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # =========================
    # 👀 RViz opzionale
    # =========================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        condition=IfCondition(rviz_enabled),
    )

    # =========================
    # 🌀 Inclusione del launch file del LiDAR SLLidar (Slamtec A1)
    # =========================
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'view_sllidar_a1_launch.py'
            ])
        ])
    )

    # =========================
    # 🚀 Ritorno della LaunchDescription
    # =========================
    return LaunchDescription([
        declare_rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        delayed_spawners,  # ✅ Tutti gli spawner avviati dopo ros2_control_node
        mecanum_cmd_node,
        mecanum_odom_node,
        ekf_node,
        rviz_node,
        # sllidar_launch,  # 👉 decommenta se vuoi avviare anche il LiDAR
        rosbridge_server_node,
        webserver_node
    ])
