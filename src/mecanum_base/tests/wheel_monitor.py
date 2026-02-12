#!/usr/bin/env python3
"""
wheel_monitor.py
----------------

Misura la VELOCITÀ MEDIA delle quattro ruote leggendo /joint_states
per tutta la durata scelta.

Esempi d'uso (da terminale):

  python3 wheel_monitor.py --duration 3 --target 5 0 0 0 --motor FL
  python3 wheel_monitor.py --duration 2 --target 0 0 0 8 --motor RR

Argomenti:
  --duration N        durata in secondi della misurazione
  --target a b c d    velocità target [FL FR RL RR]
  --motor NAME        nome motore (solo per CSV)

Output:
  - stampa velocità medie
  - stampa errori percentuali
  - aggiunge una riga a mecanum_test_log.csv
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import math
import argparse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class WheelMonitor(Node):
    def __init__(self, duration, target, motor_name, save_csv=True):
        super().__init__('wheel_monitor')

        self.duration = duration
        self.target = target
        self.motor_name = motor_name
        self.save_csv = save_csv

        self.samples = {
            "FL": [],
            "FR": [],
            "RL": [],
            "RR": []
        }

        # QoS più compatibile con /joint_states in ROS 2 Jazzy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            qos
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.check_timeout)  # 20 Hz

        self.get_logger().info(f"Monitoraggio avviato per {duration} secondi...")

    def joint_callback(self, msg):
        idx = {}
        for i, name in enumerate(msg.name):
            if name == "front_left_wheel_joint":
                idx["FL"] = i
            elif name == "front_right_wheel_joint":
                idx["FR"] = i
            elif name == "rear_left_wheel_joint":
                idx["RL"] = i
            elif name == "rear_right_wheel_joint":
                idx["RR"] = i

        for key in ["FL", "FR", "RL", "RR"]:
            if key in idx:
                try:
                    v = float(msg.velocity[idx[key]])
                    if not math.isnan(v):
                        self.samples[key].append(v)
                except Exception:
                    pass

    def check_timeout(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed >= self.duration:
            self.finish()

    def finish(self):
        self.timer.cancel()

        avg = {}
        for key, values in self.samples.items():
            if len(values) > 0:
                avg[key] = sum(values) / len(values)
            else:
                avg[key] = float('nan')

        if all(math.isnan(v) for v in avg.values()):
            self.get_logger().warn(
                "Nessun campione valido ricevuto da /joint_states nella finestra di misura."
            )

        self.get_logger().info("\n=== VELOCITÀ MEDIE (rad/s) ===")
        for key in ["FL", "FR", "RL", "RR"]:
            self.get_logger().info(f"{key}: {avg[key]:.3f}")

        err = {}
        for i, key in enumerate(["FL", "FR", "RL", "RR"]):
            t = self.target[i]
            if t == 0 or math.isnan(avg[key]):
                err[key] = float('nan')
            else:
                err[key] = ((avg[key] - t) / t) * 100

        self.get_logger().info("\n=== ERRORI (%) ===")
        for key in ["FL", "FR", "RL", "RR"]:
            self.get_logger().info(f"{key}: {err[key]:.2f}%")

        if self.save_csv:
            with open("mecanum_test_log.csv", "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow([
                    self.get_clock().now().to_msg().sec,
                    self.motor_name,
                    *self.target,
                    avg["FL"], avg["FR"], avg["RL"], avg["RR"],
                    err["FL"], err["FR"], err["RL"], err["RR"]
                ])
            self.get_logger().info("Salvato in mecanum_test_log.csv")

        self.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--target", nargs=4, type=float, required=True)
    parser.add_argument("--motor", type=str, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = WheelMonitor(args.duration, args.target, args.motor)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
