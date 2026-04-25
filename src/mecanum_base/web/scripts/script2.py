#!/usr/bin/env python3
import time

counter = 0

try:
    while True:
        counter += 1
        print(f"script2: step {counter}")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("script2: interrotto con SIGINT")
