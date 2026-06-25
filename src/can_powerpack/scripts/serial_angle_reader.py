#!/usr/bin/env python3
"""
USB serial reader for Arduino-based angle sensor.

Arduino protocol (angle_sensor.ino):
  Each line: "<raw_12bit>,<angle_deg>\n"  at 500 Hz, baud 115200.

Usage:
    from serial_angle_reader import AngleSerialReader

    reader = AngleSerialReader('/dev/ttyUSB0')
    reader.connect()

    angle_deg = reader.get_angle_deg()   # latest reading
    angle_rad = reader.get_angle_rad()

    reader.close()
"""

import serial
import threading
import time
import numpy as np


class AngleSerialReader:
    """Thread-safe USB serial reader for AS5600 angle data from Arduino."""

    def __init__(self, port: str = '/dev/ttyUSB0',
                 baud: int = 115200,
                 zero_offset_deg: float = 0.0):
        """
        port            : serial port (Linux: /dev/ttyUSB0, Windows: COM3)
        baud            : must match Arduino sketch (default 115200)
        zero_offset_deg : subtract this from every reading (software zero)
        """
        self._port   = port
        self._baud   = baud
        self._offset = zero_offset_deg

        self._angle_deg  = 0.0
        self._raw        = 0
        self._lock       = threading.Lock()
        self._connected  = False
        self._stop_event = threading.Event()
        self._ser        = None
        self._thread     = None

    # ── Public API ────────────────────────────────────────────────────────────

    def connect(self, timeout_s: float = 5.0) -> bool:
        """Open port and start background reader thread. Returns True on success."""
        try:
            self._ser = serial.Serial(self._port, self._baud,
                                      timeout=0.05,
                                      write_timeout=0.05)
            # Flush stale bytes
            time.sleep(0.1)
            self._ser.reset_input_buffer()
        except serial.SerialException as exc:
            print(f'[AngleSerialReader] Cannot open {self._port}: {exc}')
            return False

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

        # Wait for first valid reading
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            if self._connected:
                return True
            time.sleep(0.01)

        print('[AngleSerialReader] Timeout waiting for first reading.')
        return False

    def get_angle_deg(self) -> float:
        """Latest angle [deg], zero-offset applied."""
        with self._lock:
            return self._angle_deg

    def get_angle_rad(self) -> float:
        """Latest angle [rad], zero-offset applied."""
        return np.deg2rad(self.get_angle_deg())

    def get_raw(self) -> int:
        """Latest raw 12-bit encoder count (0–4095)."""
        with self._lock:
            return self._raw

    def set_zero(self):
        """Set current angle as the zero reference."""
        with self._lock:
            self._offset = self._angle_deg + self._offset

    def is_connected(self) -> bool:
        return self._connected

    def close(self):
        """Stop reader thread and close serial port."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
        self._connected = False

    # ── Internal ──────────────────────────────────────────────────────────────

    def _read_loop(self):
        """Background thread: read lines from serial and parse angle."""
        while not self._stop_event.is_set():
            try:
                line = self._ser.readline().decode('ascii', errors='ignore').strip()
            except (serial.SerialException, OSError) as exc:
                print(f'[AngleSerialReader] Read error: {exc}')
                self._connected = False
                time.sleep(0.1)
                continue

            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 2:
                continue

            try:
                raw       = int(parts[0])
                angle_raw = float(parts[1])
            except ValueError:
                continue

            with self._lock:
                self._raw        = raw
                self._angle_deg  = angle_raw - self._offset
                self._connected  = True


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Read angle from Arduino')
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--zero', action='store_true',
                        help='Set current position as zero after connecting')
    args = parser.parse_args()

    reader = AngleSerialReader(args.port, args.baud)
    if not reader.connect():
        raise SystemExit('Failed to connect.')

    if args.zero:
        reader.set_zero()
        print(f'Zero set.')

    print(f'Connected to {args.port}. Press Ctrl+C to stop.\n')
    print(f'{"raw":>6}  {"angle_deg":>10}  {"angle_rad":>10}')
    try:
        while True:
            print(f'{reader.get_raw():>6}  {reader.get_angle_deg():>10.2f}  '
                  f'{reader.get_angle_rad():>10.4f}', end='\r')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
        print('\nDisconnected.')
