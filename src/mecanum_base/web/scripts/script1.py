#!/usr/bin/env python3
import time
from datetime import datetime

try:
    while True:
        print(f"{datetime.now()} - script1: in esecuzione...")
        time.sleep(1)

except KeyboardInterrupt:
    # Questo viene eseguito quando riceve SIGINT
    print("script1: ricevuto SIGINT, chiusura pulita...")
