import serial
import json
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
rx_buffer = ""

def send_command(cmd):
    packet = json.dumps({"cmd": cmd}) + "\n"
    ser.write(packet.encode())
    print("Sent command:", packet.strip())

last_command_time = 0  # Track the last time a command was sent

while True:
    # Read incoming bytes and append to buffer
    rx_buffer += ser.read(ser.in_waiting).decode(errors="ignore")

    while "\n" in rx_buffer:
        line, rx_buffer = rx_buffer.split("\n", 1)
        try:
            # data = {"enc": [1234, 1200, 1190, 1210], "imu":  [0.02, -0.01, 9.81], "sonar": [150, 160, 170]}
            data = json.loads(line.strip())
            print("Encoder:", data.get("enc"))
            print("IMU:", data.get("imu"))
            print("Sonar:", data.get("sonar"))
        except json.JSONDecodeError as e:
            print(f"Malformed packet: {line} → JSON error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e} in line: {line}")

    # Send command every 5 seconds
    current_time = time.time()
    if current_time - last_command_time >= 5:
        send_command("motori_on")
        last_command_time = current_time
