import ujson
from machine import UART, Pin

uart = UART(0, baudrate=115200)
led = Pin(25, Pin.OUT)
buffer = ""

def handle_command(payload):
    if payload.get("action") == "led_on":
        led.value(1)
        return {"type": "status", "payload": {"status": "ok", "action": "led_on"}}
    elif payload.get("action") == "led_off":
        led.value(0)
        return {"type": "status", "payload": {"status": "ok", "action": "led_off"}}
    else:
        return {"type": "status", "payload": {"status": "unknown_command"}}

while True:
    if uart.any():
        c = uart.read(1).decode()
        if c == '\n':
            try:
                msg = ujson.loads(buffer)
                if msg.get("type") == "command":
                    response = handle_command(msg["payload"])
                    uart.write(ujson.dumps(response) + "\n")
            except ValueError:
                uart.write(ujson.dumps({"type": "status", "payload": {"status": "invalid_json"}}) + "\n")
            buffer = ""
        else:
            buffer += c