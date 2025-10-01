Create a ros2 node in python that receives a json dictionary via UART like {"enc": [1234, 1200, 1190, 1210], "imu": [0.02, -0.01, 9.81], "sonar": [150, 160, 170]} ended by "/n" and publish them in the related topics (encoder, imu, sonar). There is the option to add static paramenters for serial and imu, encoder, sonar to the related topics in yaml config file.
Provide all needed files.


import time
import json
from typing import Any, Optional
import serial
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu, Range


class SerialSensorNode(Node):
    # ... (rest of the class unchanged)
    # [Class code omitted for brevity, unchanged from your original]

def main(args=None):
    """
    Node entrypoint: init, spin, and shutdown.
    Uses MultiThreadedExecutor for concurrency.
    """
    rclpy.init(args=args)
    node = SerialSensorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()