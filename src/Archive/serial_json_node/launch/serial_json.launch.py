import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 1) Ensure we’re running from source, not an installed share
here = os.path.abspath(os.path.dirname(__file__))
#if '/install/' in here or '/share/' in here:
#    sys.exit(f"""
#[Launch] ❌ ERROR: Detected installed launch file at:
#  {here}

#Use the one in your workspace’s src folder:
#  ros2 launch ./src/serial_json_node/launch/serial_json_node.launch.py
#""")

# 2) Compute config path in source tree
config_dir   = os.path.normpath(os.path.join(here, '..', 'config'))
default_yaml = os.path.join(config_dir, 'sensors_params.yaml')

def _validate_and_print(context, *args, **kwargs):
    raw = LaunchConfiguration('param_file').perform(context)
    path = os.path.abspath(raw)

    #if not os.path.isfile(path):
    #    print(f"[Launch] ❌ ERROR: parameters file not found at {path}")
    #else:
    #    print(f"[Launch] ✅ Loading parameters from source: {path}")

    print(f"[Launch] ✅ Loading parameters from source: {path}")
    
    context.launch_configurations['param_file'] = path
    return []

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=default_yaml,
            description='absolute path to source/config/sensors_params.yaml'
        ),

        OpaqueFunction(function=_validate_and_print),

        Node(
            package='serial_json_node',
            executable='serial_json_node',
            name='serial_json_node',
            output='screen',
            parameters=[LaunchConfiguration('param_file')],
        ),
    ])

