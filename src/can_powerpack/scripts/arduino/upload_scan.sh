#!/usr/bin/env zsh
# Arduino scan sketch 업로드 + 시리얼 읽기 스크립트
# 사용: ./upload_scan.sh [skip_upload]

SKETCH_DIR="/tmp/scan_sketch/scan_sketch"
ARDUINO_CLI="$HOME/.local/bin/arduino-cli"
FQBN="arduino:avr:uno"

# 최신 ACM 포트 자동 감지
PORT=$(ls /dev/ttyACM* 2>/dev/null | sort -V | tail -1)
if [[ -z "$PORT" ]]; then
    echo "ERROR: Arduino not found. Connect USB cable and retry."
    exit 1
fi
echo "Using port: $PORT"

# 업로드 (skip_upload 인자 없을 때만)
if [[ "$1" != "skip_upload" ]]; then
    echo "Uploading scan sketch..."
    $ARDUINO_CLI upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR" 2>&1
    if [[ $? -ne 0 ]]; then
        echo "Upload failed. Re-checking port..."
        PORT=$(ls /dev/ttyACM* 2>/dev/null | sort -V | tail -1)
        echo "New port: $PORT"
    fi
fi

# 업로드 후 새 포트 재감지
sleep 1
PORT=$(ls /dev/ttyACM* 2>/dev/null | sort -V | tail -1)
echo "Reading from $PORT ..."

python3 - "$PORT" <<'PYEOF'
import serial, time, sys

port = sys.argv[1]
try:
    ser = serial.Serial(port, 115200, timeout=1)
except Exception as e:
    print(f"Cannot open {port}: {e}")
    sys.exit(1)

print(f"Opened {port}. Waiting for I2C scan results (up to 15s)...")
t0 = time.time()
while time.time() - t0 < 15:
    line = ser.readline()
    if line:
        print(line.decode('ascii', errors='replace').rstrip())
ser.close()
PYEOF
