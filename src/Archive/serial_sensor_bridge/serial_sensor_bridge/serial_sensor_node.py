"""
# make sure you’re in the workspace root
cd ~/ros2_ws

# clean out old builds (optional but recommended)
rm -rf build/ install/ log/

# build ONLY the Python package (no CMakeLists needed):
# colcon build --symlink-install --packages-select serial_sensor_bridge

# finally, source the install space
source install/setup.bash

# launch the node
ros2 launch serial_sensor_bridge serial_sensor_launch.py

"""

import time
import json
from typing import Any, Optional
import serial
import rclpy
from rclpy.node import Node
from diagnostic_updater import Updater, FunctionDiagnosticTask
from diagnostic_msgs.msg import DiagnosticStatus

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu, Range


class SerialSensorNode(Node):
    """
    A ROS 2 node that reads JSON-formatted sensor data from a serial port,
    publishes encoder, IMU, and sonar topics, and provides diagnostics.
    """

    def __init__(self):
        super().__init__("serial_sensor_node")

        # --- Declare and read parameters ---
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("loop_hz", 10.0)
        self.declare_parameter("serial_reset_timeout", 5.0)

        self.declare_parameter(
            "wheel_power_front_left_topic", "/wheel_power_front_left"
        )
        self.declare_parameter(
            "wheel_power_front_right_topic", "/wheel_power_front_right"
        )
        self.declare_parameter(
            "wheel_power_back_left_topic", "/wheel_power_back_left"
        )
        self.declare_parameter(
            "wheel_power_back_right_topic", "/wheel_power_back_right"
        )
        self.declare_parameter("servo_pan_topic", "/servo_pan")
        self.declare_parameter("servo_tilt_topic", "/servo_tilt")

        self.declare_parameter("encoder_topic_front_left", "/encoder_front_left")
        self.declare_parameter("encoder_topic_front_right", "/encoder_front_right")
        self.declare_parameter("encoder_topic_back_left", "/encoder_back_left")
        self.declare_parameter("encoder_topic_back_right", "/encoder_back_right")

        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("sonar_topic", "/sonar")
        self.declare_parameter("sonar_frames", ["front", "right", "left"])
        self.declare_parameter("encoder_frame_id", "wheel_encoder")
        self.declare_parameter("imu_frame_id", "base_imu")

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        loop_hz = self.get_parameter("loop_hz").value
        self.reset_timeout = self.get_parameter("serial_reset_timeout").value

        self.wheel_power_front_left = self.get_parameter(
            "wheel_power_front_left_topic"
        ).value
        self.wheel_power_front_right = self.get_parameter(
            "wheel_power_front_right_topic"
        ).value
        self.wheel_power_back_left = self.get_parameter(
            "wheel_power_back_left_topic"
        ).value
        self.wheel_power_back_right = self.get_parameter(
            "wheel_power_back_right_topic"
        ).value
        self.servo_pan = self.get_parameter("servo_pan_topic").value
        self.servo_tilt = self.get_parameter("servo_tilt_topic").value

        # self.encoder_topic = self.get_parameter('encoder_topic').value
        self.encoder_topic_front_left = self.get_parameter(
            "encoder_topic_front_left"
        ).value
        self.encoder_topic_front_right = self.get_parameter(
            "encoder_topic_front_right"
        ).value
        self.encoder_topic_back_left = self.get_parameter(
            "encoder_topic_back_left"
        ).value
        self.encoder_topic_back_right = self.get_parameter(
            "encoder_topic_back_right"
        ).value

        self.imu_topic = self.get_parameter("imu_topic").value
        self.sonar_topic = self.get_parameter("sonar_topic").value
        self.sonar_frames = self.get_parameter("sonar_frames").value
        self.encoder_frame = self.get_parameter("encoder_frame_id").value
        self.imu_frame = self.get_parameter("imu_frame_id").value

        # --- Track last read time and open serial with retries ---
        self.last_read_time = self.get_clock().now()
        self._wait_and_open_serial(port, baud)

        # Call this in __init__ after opening serial
        self._setup_subscribers()


        # --- Publishers ---
        # self.enc_pub = self.create_publisher(Int32MultiArray, self.encoder_topic, 10)
        # Publisher per ogni encoder
        self.enc_front_left_pub = self.create_publisher(
            Int32, self.encoder_topic_front_left, 10
        )
        self.enc_front_right_pub = self.create_publisher(
            Int32, self.encoder_topic_front_right, 10
        )
        self.enc_back_left_pub = self.create_publisher(
            Int32, self.encoder_topic_back_left, 10
        )
        self.enc_back_right_pub = self.create_publisher(
            Int32, self.encoder_topic_back_right, 10
        )

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.sonar_pubs = []
        for frame in self.sonar_frames:
            topic = f"{self.sonar_topic}/{frame}"
            self.sonar_pubs.append(self.create_publisher(Range, topic, 10))

        # --- Diagnostics updater ---
        self.updater = Updater(self)
        self.updater.setHardwareID("serial_sensor_node")
        self.updater.add(
            FunctionDiagnosticTask("Serial Sensor Status", self._diagnostics_callback)
        )

        # --- Timer for periodic read & publish ---
        timer_period = 1.0 / loop_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # --- Subscribe to wheel power and servo commands ---
    def _setup_subscribers(self):
        self.create_subscription(
            Float32,
            str(self.wheel_power_front_left),
            lambda msg: self.wheel_power_callback(msg, self.wheel_power_front_left),
            10,
        )
        self.create_subscription(
            Float32,
            str(self.wheel_power_front_right),
            lambda msg: self.wheel_power_callback(msg, self.wheel_power_front_right),
            10,
        )
        self.create_subscription(
            Float32,
            str(self.wheel_power_back_left),
            lambda msg: self.wheel_power_callback(msg, self.wheel_power_back_left),
            10,
        )
        self.create_subscription(
            Float32,
            str(self.wheel_power_back_right),
            lambda msg: self.wheel_power_callback(msg, self.wheel_power_back_right),
            10,
        )
        self.create_subscription(
            Float32,
            str(self.servo_pan),
            lambda msg: self.servo_callback(msg, "servo_pan"),
            10,
        )
        self.create_subscription(
            Float32,
            str(self.servo_tilt),
            lambda msg: self.servo_callback(msg, "servo_tilt"),
            10,
        )

    def _wait_and_open_serial(self, port: str, baud: int):
        """
        Attempt to open the serial port repeatedly until successful.
        Logs a warning and retries once per second if the port is unavailable.
        """
        while rclpy.ok():
            try:
                self.ser = serial.Serial(port, baud, timeout=1.0)
                self.get_logger().info(f"Opened serial port {port} @ {baud} bps")
                return
            except serial.SerialException:
                self.get_logger().warn(
                    f"Serial port {port} not available; retrying in 1 second"
                )
                time.sleep(1.0)

    def _open_serial(self, port: str, baud: int):
        """
        Immediate open for resets; raises on failure.
        """
        try:
            self.ser = serial.Serial(port, baud, timeout=1.0)
            self.get_logger().info(f"Opened serial port {port} @ {baud} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise

    def _diagnostics_callback(self, stat):
        """
        Check elapsed time since last read; report ERROR if hung.
        """
        now = self.get_clock().now()
        elapsed = (now - self.last_read_time).nanoseconds * 1e-9

        if elapsed > self.reset_timeout:
            stat.summary(DiagnosticStatus.ERROR, "No data; serial hung")
        else:
            stat.summary(DiagnosticStatus.OK, "Serial port healthy")

        stat.add("seconds since last read", f"{elapsed:.2f}")
        return stat

    def _send_command(self,
                      topic: str,
                      payload: Optional[Any] = None,
                      newline: str = "\n",
                      retries: int = 3,
                      retry_delay: float = 0.2,
                      ) -> bool:
        
            """
            Send a JSON command over a pyserial Serial interface, retrying on failures.
            """
            packet = {"topic": topic}
            if payload is not None:
                packet["payload"] = f'{{"data":{payload.data}}}' if hasattr(payload, 'data') else payload

            packet_str = json.dumps(packet) + newline

            for attempt in range(1, retries + 1):
                try:
                    self.ser.write(packet_str.encode("utf-8"))
                    self.ser.flush()
                except Exception:
                    self.get_logger().warning(
                        "Attempt {}/{} failed sending: {}".format(
                            attempt,
                            retries,
                            packet_str.strip()))
                    time.sleep(retry_delay)
                else:
                    self.get_logger().info("Sent command: {}".format(packet_str.strip()))
                    return True

            self.get_logger().error("All {} attempts failed for command: {}".format(retries, packet_str.strip()))
            return False
      
            """
            Send a command as JSON over UART, ending with '/n'.
    
            #payload = {"topic": topic_name, "payload": {"payload": value}}
            packet = {"topic": topic_name}
            if payload is not None:
                packet["payload"] = payload
                
            packet_str = json.dumps(packet) + newline
            try:
                msg = json.dumps(payload) + "/n"
                self.ser.write(msg.encode("utf-8"))
            except Exception as e:
                self.get_logger().warn(f"Failed to send command: {e}")
            """

    def wheel_power_callback(self, msg, topic_name):
        self.get_logger().info(f"Received wheel power command on {topic_name}: {msg.data}")
        self._send_command(topic=topic_name, payload=msg)

    def servo_callback(self, msg, topic_name):
        self._send_command(topic_name, msg)

    def timer_callback(self):
        """
        Periodic callback: handles port resets, reads a full line,
        parses JSON, publishes data, and updates diagnostics.
        """
        stamp = self.get_clock().now().to_msg()
        now_time = self.get_clock().now()

        # Reset port if no data arrived within timeout
        elapsed = (now_time - self.last_read_time).nanoseconds * 1e-9
        if elapsed > self.reset_timeout:
            self.get_logger().warn("Serial timeout; reopening port")
            try:
                self.ser.close()
            except Exception:
                pass

            port = self.get_parameter("serial_port").value
            baud = self.get_parameter("baud_rate").value
            self._open_serial(port, baud)
            self.last_read_time = now_time
            return

        # Read one full line (blocking until '\n' or timeout)
        try:
            raw_line = self.ser.readline()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
            self.updater.force_update()
            return

        if not raw_line:
            return

        # Clean up stray '/n' literal and whitespace
        clean_bytes = raw_line.replace(b"/n", b"").strip()
        if not clean_bytes:
            return

        # Validate JSON object structure
        if not (clean_bytes.startswith(b"{") and clean_bytes.endswith(b"}")):
            self.get_logger().warn(f"Non-JSON received (skipping): {clean_bytes!r}")
            return

        # Decode and parse JSON
        try:
            line_str = clean_bytes.decode("utf-8", errors="ignore")
            data = json.loads(line_str)
            self.last_read_time = now_time
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            self.get_logger().warn(f"JSON parse error: {e}. Raw bytes: {clean_bytes!r}")
            return

        # Publish encoder data
        # if 'enc' in data:
        if "enc" not in data or not isinstance(data["enc"], dict):
            self.get_logger().warning(
                f"Encoder data mancante o malformata: {data.get('enc')}"
            )
            return
        #self.encoder_topic
        publishers = {
            "front_left": self.enc_front_left_pub,
            "front_right": self.enc_front_right_pub,
            "back_left": self.enc_back_left_pub,
            "back_right": self.enc_back_right_pub,
        }

        for key, pub in publishers.items():
            try:
                raw = data["enc"][key]
                pub.publish(Int32(data=int(raw)))

            except KeyError:
                self.get_logger().warning(f"Chiave mancante per encoder '{key}'")

            except (ValueError, TypeError):
                self.get_logger().warning(f"Valore non valido per '{key}': {raw}")
            # try:
            #    arr = [int(x) for x in data['enc']]
            #    self.enc_pub.publish(Int32MultiArray(data=arr))
            # except (ValueError, TypeError, KeyError) as e:
            #    self.get_logger().warn(f'Bad encoder data {data.get("enc")}: {e}')

        # Publish IMU data
        if "imu" in data:
            try:
                vals = data["imu"]
                imu_msg = Imu()
                imu_msg.header.stamp = stamp
                imu_msg.header.frame_id = self.imu_frame
                imu_msg.linear_acceleration.x = float(vals[0])
                imu_msg.linear_acceleration.y = float(vals[1])
                imu_msg.linear_acceleration.z = float(vals[2])
                self.imu_pub.publish(imu_msg)
            except (ValueError, TypeError, IndexError) as e:
                self.get_logger().warn(f'Bad IMU data {data.get("imu")}: {e}')

        # Publish sonar data
        if "sonar" in data:
            try:
                for i, val in enumerate(data["sonar"]):
                    if i >= len(self.sonar_pubs):
                        break
                    rng = Range()
                    rng.header.stamp = stamp
                    rng.header.frame_id = self.sonar_frames[i]
                    rng.radiation_type = Range.ULTRASOUND
                    rng.field_of_view = 0.5
                    rng.min_range = 0.02
                    rng.max_range = 4.0
                    rng.range = float(val) / 1000.0
                    self.sonar_pubs[i].publish(rng)
            except (ValueError, TypeError, IndexError) as e:
                self.get_logger().warn(f'Bad sonar data {data.get("sonar")}: {e}')

        # Update diagnostics
        self.updater.force_update()

    def destroy_node(self):
        """
        Ensure serial port is closed cleanly on shutdown.
        """
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    """
    Node entrypoint: init, spin, and shutdown.
    """
    rclpy.init(args=args)
    node = SerialSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
