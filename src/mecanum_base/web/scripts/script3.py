#!/usr/bin/env python3
import time
import random
from datetime import datetime

try:
    while True:
        # Simula un lavoro più pesante
        time.sleep(random.uniform(0.5, 1.5))
        print(f"{datetime.now()} - script3: task completato")

except KeyboardInterrupt:
    print("script3: SIGINT ricevuto, uscita immediata")
