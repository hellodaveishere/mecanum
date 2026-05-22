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
  --motor NAME        nome motore (FL/FR/RL/RR o 1..4)
  --analyze-oscillation  analizza oscillazioni e periodo (per autotuning PID)
  --validate-pid         valuta overshoot/stabilità (per test PID)

Output:
  - stampa velocità medie
  - stampa errori percentuali
  - aggiunge una riga a mecanum_test_log.csv
  - in modalità --analyze-oscillation:
        stampa su stdout: "OSCILLATION_DETECTED PERIOD=<valore>" se rilevate
  - in modalità --validate-pid:
        stampa su stdout un breve report per la velocità testata
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import csv
import math
import argparse
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def detect_oscillation_and_period(times, values, target):
    """Rileva oscillazioni tramite zero-crossing rispetto al target e stima il periodo medio."""
    if len(times) < 5:
        return False, float('nan')

    # errore rispetto al target
    err = [v - target for v in values]

    # soglia minima di ampiezza per considerare oscillazione
    amp = max(abs(e) for e in err)
    if amp < max(0.05 * abs(target), 0.1):
        return False, float('nan')

    # zero-crossing (cambio di segno dell'errore)
    zc_times = []
    prev_e = err[0]
    prev_t = times[0]
    for t, e in zip(times[1:], err[1:]):
        if prev_e == 0:
            prev_e = e
            prev_t = t
            continue
        if (prev_e > 0 and e < 0) or (prev_e < 0 and e > 0):
            # interpolazione lineare del punto di zero
            dt = t - prev_t
            if dt > 0:
                alpha = abs(prev_e) / (abs(prev_e) + abs(e))
                zc_t = prev_t + alpha * dt
                zc_times.append(zc_t)
        prev_e = e
        prev_t = t

    if len(zc_times) < 4:
        return False, float('nan')

    # periodo ≈ differenza tra zero-crossing alternati (due zero-crossing = mezzo periodo)
    periods = []
    for i in range(2, len(zc_times)):
        T = zc_times[i] - zc_times[i - 2]
        if T > 0:
            periods.append(T)

    if not periods:
        return False, float('nan')

    Tu = sum(periods) / len(periods)
    return True, Tu


def compute_overshoot(values, target):
    """Calcola overshoot percentuale rispetto al target (solo target > 0)."""
    if target <= 0:
        return float('nan')
    vmax = max(values) if values else float('nan')
    if math.isnan(vmax):
        return float('nan')
    return (vmax - target) / target * 100.0


class WheelMonitor(Node):
    def __init__(self, duration, target, motor_name,
                 analyze_oscillation=False, validate_pid=False,
                 save_csv=True):
        super().__init__('wheel_monitor')

        self.duration = duration
        self.target = target
        self.motor_name = motor_name
        self.save_csv = save_csv
        self.analyze_oscillation = analyze_oscillation
        self.validate_pid = validate_pid

        self.samples = {
            "FL": [],
            "FR": [],
            "RL": [],
            "RR": []
        }
        self.time_samples = {
            "FL": [],
            "FR": [],
            "RL": [],
            "RR": []
        }

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

        now = self.get_clock().now()
        t_rel = (now - self.start_time).nanoseconds / 1e9

        for key in ["FL", "FR", "RL", "RR"]:
            if key in idx:
                try:
                    v = float(msg.velocity[idx[key]])
                    if not math.isnan(v):
                        self.samples[key].append(v)
                        self.time_samples[key].append(t_rel)
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
            try:
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
            except Exception as e:
                self.get_logger().warn(f"Errore salvataggio CSV: {e}")

        # Se richiesto, analisi oscillazioni / validazione PID
        key_sel = None
        if self.motor_name in ["FL", "FR", "RL", "RR"]:
            key_sel = self.motor_name
        else:
            # se motor_name è numerico 1..4
            if self.motor_name == "1":
                key_sel = "FL"
            elif self.motor_name == "2":
                key_sel = "FR"
            elif self.motor_name == "3":
                key_sel = "RL"
            elif self.motor_name == "4":
                key_sel = "RR"

        if key_sel is None:
            # fallback: primo target non nullo
            mapping = ["FL", "FR", "RL", "RR"]
            for i, t in enumerate(self.target):
                if t != 0:
                    key_sel = mapping[i]
                    break

        if key_sel is not None:
            times = self.time_samples[key_sel]
            values = self.samples[key_sel]
            idx_map = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}
            t_target = self.target[idx_map[key_sel]]

        if self.analyze_oscillation:
            # ============================
            # Calcolo errore percentuale
            # ============================
            if abs(t_target) < 1e-6:
                err_pct = 0.0
            else:
                if len(values) > 0:
                    v_mean = sum(values) / len(values)
                    err_pct = (v_mean - t_target) / t_target * 100.0
                else:
                    err_pct = float('nan')

            # ============================
            # Rilevamento oscillazioni
            # ============================
            osc, Tu = detect_oscillation_and_period(times, values, t_target)

            if osc and not math.isnan(Tu):
                # Oscillazione stabile → Ku trovato
                print(f"OSCILLATION_DETECTED PERIOD={Tu:.4f} ERROR={err_pct:.2f}")
                self.destroy_node()
                rclpy.shutdown()
                return

            # ============================
            # Controllo oscillazione instabile
            # ============================
            if len(values) > 10:
                mid = len(values) // 2
                amp_start = max(abs(v - t_target) for v in values[:mid])
                amp_end   = max(abs(v - t_target) for v in values[mid:])

                if amp_end > amp_start * 1.5:
                    print(f"UNSTABLE_OSCILLATION ERROR={err_pct:.2f}")
                    self.destroy_node()
                    rclpy.shutdown()
                    return

            # ============================
            # Nessuna oscillazione → steady / no oscillation
            # ============================
            print(f"NO_OSCILLATION_DETECTED ERROR={err_pct:.2f}")
            self.destroy_node()
            rclpy.shutdown()
            return



            if self.validate_pid:
                overshoot = compute_overshoot(values, t_target)
                osc, _ = detect_oscillation_and_period(times, values, t_target)
                if math.isnan(overshoot):
                    print("VALIDATION_RESULT UNKNOWN (dati insufficienti)")
                else:
                    status = "OK"
                    if overshoot > 30 or osc:
                        status = "UNSTABLE"
                    print(f"VALIDATION_RESULT {status} OVERSHOOT={overshoot:.2f}%")

        self.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--target", nargs=4, type=float, required=True)
    parser.add_argument("--motor", type=str, required=True)
    parser.add_argument("--analyze-oscillation", action="store_true")
    parser.add_argument("--validate-pid", action="store_true")
    args = parser.parse_args()

    # mappa motore numerico → nome logico
    motor_map = {
        "1": "FL",
        "2": "FR",
        "3": "RL",
        "4": "RR",
        "FL": "FL",
        "FR": "FR",
        "RL": "RL",
        "RR": "RR",
    }
    motor_name = motor_map.get(args.motor, args.motor)

    rclpy.init()
    node = WheelMonitor(
        duration=args.duration,
        target=args.target,
        motor_name=motor_name,
        analyze_oscillation=args.analyze_oscillation,
        validate_pid=args.validate_pid,
        save_csv=True
    )
    rclpy.spin(node)


if __name__ == '__main__':
    main()
