from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ============================================================
    # 1) ARGOMENTI DI LAUNCH
    # ============================================================
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Abilita o disabilita RViz alla partenza'
    )
    rviz_enabled = LaunchConfiguration('rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    params_file = LaunchConfiguration(
        'params_file',
        default='config/nav2_params.yaml'  # relativo al package mecanum_base
    )

    # ============================================================
    # 2) PERCORSI FILE E RISORSE
    # ============================================================
    pkg_share = FindPackageShare('mecanum_base')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mecanum_robot.xacro'])
    controllers_path = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    ekf_config = PathJoinSubstitution([pkg_share, 'config', 'ekf.yaml'])
    rviz_path = PathJoinSubstitution([pkg_share, 'rviz', 'mecanum.rviz'])

    webserver_path = os.path.join(
        get_package_share_directory('mecanum_base'),
        'webserver'
    )

    # ============================================================
    # 3) ROBOT DESCRIPTION (XACRO → URDF)
    # ============================================================
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),  # spazio dopo "xacro" voluto
        value_type=str
    )

    # ============================================================
    # 4) NODI DI BASE DEL ROBOT
    #    - robot_state_publisher
    #    - ros2_control_node
    #    - spawner controller
    # ============================================================

    # Pubblica TF dal modello URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Nodo principale ros2_control
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_path
        ],
        output='screen',
    )

    # Spawner dei controller e broadcaster
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

    spawner_estop = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['estop_state_broadcaster', '--controller-manager-timeout', '10.0'],
        output='screen',
    )

    # Ritardo per dare tempo a ros2_control_node di inizializzarsi
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
            spawner_battery,
            spawner_estop,
        ]
    )

    # ============================================================
    # 5) NODI APPLICATIVI DEL ROBOT
    # ============================================================

    mecanum_cmd_node = Node(
        package='mecanum_base',
        executable='mecanum_cmd_node',
        name='mecanum_cmd_node',
        output='screen',
        # niente parameters=[controllers_path] se il nodo non li usa davvero
    )

    mecanum_odom_node = Node(
        package='mecanum_base',
        executable='mecanum_odom_node',
        name='mecanum_odom_node',
        output='screen',
    )

    calibration_node = Node(
        package='mecanum_base',
        executable='calibration_node',
        name='calibration_node',
        output='screen',
    )

    estop_manager_node = Node(
        package='mecanum_base',
        executable='estop_manager_node',
        name='estop_manager_node',
        output='screen',
    )

    rosout_reply_node = Node(
        package='mecanum_base',
        executable='rosout_relay_node',
        name='rosout_relay',
        output='screen',
    )

    # ============================================================
    # 6) EKF (robot_localization)
    # ============================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # ============================================================
    # 7) LiDAR (SLLidar A1)
    # ============================================================
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sllidar_ros2'),
                'launch',
                'view_sllidar_a1_launch.py'
            ])
        ])
    )

    # ============================================================
    # 8) NAV2 - NODI DI NAVIGAZIONE
    #    Coerenti con nav2_params.yaml che hai definito
    # ============================================================

    # map_server (solo se usi AMCL + mappa statica)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # AMCL (localizzazione su mappa statica)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file]
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file]
    )

    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[params_file]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'smoother_server',
                'behavior_server',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
                'bt_navigator'
            ]
        }]
    )

    # Client Nav2 (il tuo)
    my_nav_client = Node(
        package='mecanum_base',   # CORRETTO: è nel tuo pacchetto
        executable='my_nav_client',
        name='my_nav_client',
        output='screen'
    )

    # ============================================================
    # 9) RVIZ (opzionale)
    # ============================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen',
        condition=IfCondition(rviz_enabled),
    )

    # ============================================================
    # 10) ROSBRIDGE + WEBSERVER + CAMERA + THROTTLE
    # ============================================================

    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'max_message_size': 100000000},
            {'port': 9091},
            {'rosout_enable': True}
        ]
    )

    webserver_node = ExecuteProcess(
        cmd=['node', 'server.js'],
        cwd=webserver_path,
        output='screen'
    )

    throttle_node_for_battery_status = Node(
        package='topic_tools',
        executable='throttle',
        name='throttle_battery_state',
        arguments=[
            'messages',
            '/battery_state_broadcaster/battery_state',
            '0.2',
            '/battery_state_broadcaster/battery_state_throttled'
        ],
        output='screen'
    )

    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[
            {'video_device': '/dev/video0'},
            {'image_size': [640, 480]},
            {'frame_rate': 5.0},
            {'output_encoding': 'rgb8'},
            {'use_ros_timestamp': True}
        ],
        remappings=[
            ('image_raw', '/camera/image_raw')
        ]
    )

    # ============================================================
    # 11) COMPOSIZIONE FINALE DELLA LAUNCH DESCRIPTION
    # ============================================================
    return LaunchDescription([
        # Argomenti
        declare_rviz_arg,

        # Robot base
        robot_state_publisher_node,
        ros2_control_node,
        delayed_spawners,

        # Nodi robot custom
        mecanum_cmd_node,
        mecanum_odom_node,
        calibration_node,
        estop_manager_node,
        rosout_reply_node,

        # EKF + LiDAR
        ekf_node,
        sllidar_launch,

        # Nav2
        map_server,
        amcl,
        planner_server,
        controller_server,
        smoother_server,
        behavior_server,
        waypoint_follower,
        velocity_smoother,
        collision_monitor,
        bt_navigator,
        lifecycle_manager,
        my_nav_client,

        # Visualizzazione
        rviz_node,

        # Web / bridge / camera
        rosbridge_server_node,
        webserver_node,
        throttle_node_for_battery_status,
        v4l2_camera_node,
    ])
