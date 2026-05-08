#!/bin/bash

echo "=== Test alimentazione Raspberry Pi 5 ==="
echo "CPU + USB stress + monitoraggio undervoltage"
echo

# Avvia monitoraggio undervoltage in background
echo "[Monitor] Avvio monitoraggio undervoltage..."
( while true; do vcgencmd get_throttled; sleep 1; done ) &
MONITOR_PID=$!

# Avvia stress CPU
echo "[Stress] Avvio stress CPU..."
stress-ng --cpu 4 --timeout 30s --metrics-brief

# Avvia stress USB (se hai ventole o periferiche collegate)
echo "[Stress] Avvio stress I/O USB..."
stress-ng --iomix 4 --timeout 20s --metrics-brief

# Ferma monitoraggio
kill $MONITOR_PID

echo
echo "=== Risultato finale ==="
vcgencmd get_throttled
echo
echo "0x0 = alimentazione perfetta"
echo "0x50000 = undervoltage passato"
echo "0x1 = undervoltage attuale"
echo "0x2 = throttling attuale"

