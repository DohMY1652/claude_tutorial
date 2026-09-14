#!/usr/bin/env python3
"""
Live monitor for the Teensy 6-channel encoder stream.

  python3 monitor.py                 # terminal dashboard
  python3 monitor.py --plot          # scrolling graph (needs matplotlib)
  python3 monitor.py --csv log.csv   # also log to CSV

Keys while running:
  r   reset the learned min/max range
  s   save current range to calib.json
  z   zero: use the current position as 0 deg
  q   quit

Angle is derived from the learned voltage range. Turn each shaft slowly
from one mechanical stop to the other once, then press 's'. From then on
the angle column is real.
"""

import argparse
import json
import os
import struct
import sys
import time
from collections import deque

import serial
import serial.tools.list_ports

FRAME_LEN = 24
SYNC = b"\xAA\x55"
LSB_UV = 62.5          # ADS1115 FSR +/-2.048V
NCH = 6
SPAN_DEG = 120.0       # mechanical travel
CALIB_FILE = "calib.json"
SD_WINDOW = 200        # 1 second at 200 Hz

# ----------------------------------------------------------------- terminal

if os.name == "nt":
    import msvcrt

    def key_setup():
        return None

    def key_restore(_):
        pass

    def key_get():
        return msvcrt.getch().decode(errors="ignore") if msvcrt.kbhit() else None
else:
    import select
    import termios
    import tty

    # 파이프·리다이렉트로 돌리면 stdin 이 TTY 가 아니다. 그때 termios 를 부르면
    # "Inappropriate ioctl for device" 로 죽는다 — 키 입력만 조용히 포기한다.
    def _tty():
        try:
            return sys.stdin.isatty()
        except Exception:
            return False

    def key_setup():
        if not _tty():
            return None
        try:
            old = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            return old
        except Exception:
            return None

    def key_restore(old):
        if old is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
            except Exception:
                pass

    def key_get():
        if not _tty():
            return None
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        except Exception:
            pass
        return None


HIDE, SHOW, HOME = "\033[?25l", "\033[?25h", "\033[H"


def bar(frac, width=22):
    """Position marker inside a track, e.g. [------o---------]"""
    if frac is None:
        return "[" + "?" * width + "]"
    frac = max(0.0, min(1.0, frac))
    pos = int(round(frac * (width - 1)))
    return "[" + "-" * pos + "o" + "-" * (width - 1 - pos) + "]"


# -------------------------------------------------------------------- frame

def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def find_port() -> str:
    for p in serial.tools.list_ports.comports():
        blob = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
        if "teensy" in blob or "16c0" in blob:
            return p.device
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Is the Teensy plugged in?")
    print("Teensy not auto-detected. Available ports:")
    for p in ports:
        print(f"  {p.device}  {p.description}")
    sys.exit("Re-run with --port <device>")


# --------------------------------------------------------------- calibration

def load_yaml_2pt():
    """powerpack_config.yaml 의 TeensyEncoder.channels 2점 보정을 읽는다.

    **이게 단일 출처다.** 예전에는 이 파일이 학습한 min/max × 120° 로 각도를
    따로 만들어, 브리지(board/analog)와 다른 값을 보여 줬다. 같은 화면을 보고
    다른 각도를 말하는 상황은 예전에 0° 가 158° 로 읽히던 사고와 같은 종류다.
    반환: {ch: (raw_0deg, raw_90deg)}, 그리고 출처 문자열.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [
        # lib/can_powerpack/  →  share/can_powerpack/config/  (설치본을 먼저)
        os.path.join(here, "..", "..", "share", "can_powerpack", "config",
                     "powerpack_config.yaml"),
        os.path.join(here, "..", "config", "powerpack_config.yaml"),
    ]
    for path in cands:
        path = os.path.normpath(path)
        if not os.path.exists(path):
            continue
        try:
            import yaml
            with open(path) as f:
                doc = yaml.safe_load(f)
            br = (doc.get("/pack2/can_bridge") or {}).get("ros__parameters", {}) or {}
            ch = (br.get("TeensyEncoder") or {}).get("channels", {}) or {}
            out = {}
            for k, v in ch.items():
                try:
                    c = int(k)
                except (TypeError, ValueError):
                    continue
                r0, r90 = v.get("raw_0deg"), v.get("raw_90deg")
                if (r0 is not None and r90 is not None
                        and abs(float(r90) - float(r0)) > 1e-9):
                    out[c] = (float(r0), float(r90))
            if out:
                return out, path
        except Exception as e:
            return {}, f"{path} 를 못 읽었다 ({e})"
    return {}, "powerpack_config.yaml 을 못 찾았다"


class Calib:
    def __init__(self):
        self.lo = [None] * NCH      # raw code at one mechanical stop
        self.hi = [None] * NCH
        self.zero = [0.0] * NCH     # angle offset in degrees
        # yaml 2점 보정 — 있으면 이쪽이 이긴다 (브리지와 같은 값을 봐야 한다)
        self.two_pt, self.src = load_yaml_2pt()

    def observe(self, ch, raw):
        if self.lo[ch] is None or raw < self.lo[ch]:
            self.lo[ch] = raw
        if self.hi[ch] is None or raw > self.hi[ch]:
            self.hi[ch] = raw

    def frac(self, ch, raw):
        tp = self.two_pt.get(ch)
        if tp is not None:
            r0, r90 = tp
            # 0°~90° 를 막대 전체로 본다 (그 밖으로 나가면 끝에 붙는다)
            return max(0.0, min(1.0, (raw - r0) / (r90 - r0)))
        lo, hi = self.lo[ch], self.hi[ch]
        if lo is None or hi is None or hi - lo < 200:
            return None          # not enough travel seen yet
        return (raw - lo) / (hi - lo)

    def deg(self, ch, raw):
        tp = self.two_pt.get(ch)
        if tp is not None:
            r0, r90 = tp
            # 브리지(CanBridge::sensor_routine)와 **같은 식**이다. 클램프하지 않는다 —
            # 0~90° 밖도 그대로 보여야 배선·장착 문제가 눈에 띈다.
            return (raw - r0) * 90.0 / (r90 - r0) - self.zero[ch]
        f = self.frac(ch, raw)
        return None if f is None else f * SPAN_DEG - self.zero[ch]

    def set_zero_here(self, raws):
        for i in range(NCH):
            self.zero[i] = 0.0                     # 먼저 지워야 절대각이 나온다
            d = self.deg(i, raws[i])
            self.zero[i] = d if d is not None else 0.0

    def reset(self):
        self.lo = [None] * NCH
        self.hi = [None] * NCH

    def save(self, path=CALIB_FILE):
        with open(path, "w") as f:
            json.dump({"lo": self.lo, "hi": self.hi, "zero": self.zero,
                       "span_deg": SPAN_DEG}, f, indent=2)

    def load(self, path=CALIB_FILE):
        if not os.path.exists(path):
            return False
        with open(path) as f:
            d = json.load(f)
        self.lo = d.get("lo", self.lo)
        self.hi = d.get("hi", self.hi)
        self.zero = d.get("zero", self.zero)
        return True


# --------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--csv", default=None)
    ap.add_argument("--plot", action="store_true", help="scrolling matplotlib graph")
    ap.add_argument("--no-start", action="store_true",
                    help="do not send 'r'; assume already streaming")
    args = ap.parse_args()

    if args.plot:
        return run_plot(args)

    port = args.port or find_port()
    ser = serial.Serial(port, 2000000, timeout=0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    if not args.no_start:
        ser.write(b"r")
        time.sleep(0.15)
        ser.reset_input_buffer()

    cal = Calib()
    loaded = cal.load()

    csv = open(args.csv, "w", buffering=1) if args.csv else None
    if csv:
        csv.write("seq,t_us," + ",".join(f"ch{i}_mV" for i in range(NCH)) + ",status\n")

    hist = [deque(maxlen=SD_WINDOW) for _ in range(NCH)]
    raws = [0] * NCH
    status = 0
    frames = crc_err = lost = 0
    win_frames = 0
    prev_seq = None
    hz = 0.0
    t_win = time.perf_counter()
    t_draw = 0.0
    if cal.two_pt:
        msg = f"yaml 2점 보정 {len(cal.two_pt)}채널 — {cal.src}"
    elif loaded:
        msg = f"{CALIB_FILE} 의 학습 범위 사용 (yaml 2점 보정 없음: {cal.src})"
    else:
        msg = f"보정 없음 — 축을 끝에서 끝까지 움직일 것 ({cal.src})"

    buf = bytearray()
    old_tty = key_setup()
    sys.stdout.write(HIDE)
    try:
        while True:
            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))

            while len(buf) >= FRAME_LEN:
                i = buf.find(SYNC)
                if i < 0:
                    del buf[:-1]
                    break
                if i > 0:
                    del buf[:i]
                    continue
                if len(buf) < FRAME_LEN:
                    break
                f = bytes(buf[:FRAME_LEN])
                if crc16(f[:22]) != struct.unpack_from("<H", f, 22)[0]:
                    crc_err += 1
                    del buf[:2]
                    continue
                del buf[:FRAME_LEN]

                seq = struct.unpack_from("<H", f, 2)[0]
                t_us = struct.unpack_from("<I", f, 4)[0]
                raws = list(struct.unpack_from("<6h", f, 8))
                status = struct.unpack_from("<H", f, 20)[0]

                if prev_seq is not None:
                    gap = (seq - prev_seq - 1) & 0xFFFF
                    if gap:
                        lost += gap
                prev_seq = seq
                frames += 1
                win_frames += 1

                for c in range(NCH):
                    hist[c].append(raws[c])
                    cal.observe(c, raws[c])

                if csv:
                    vals = ",".join(f"{v * LSB_UV / 1000.0:.4f}" for v in raws)
                    csv.write(f"{seq},{t_us},{vals},{status}\n")

            now = time.perf_counter()
            if now - t_win >= 1.0:
                hz = win_frames / (now - t_win)
                win_frames = 0
                t_win = now

            k = key_get()
            if k:
                k = k.lower()
                if k == "q":
                    break
                elif k == "r":
                    cal.reset()
                    msg = "range reset - move each shaft end to end"
                elif k == "s":
                    cal.save()
                    msg = f"saved to {CALIB_FILE}"
                elif k == "z":
                    cal.set_zero_here(raws)
                    msg = "zero set at current position"

            if now - t_draw >= 0.066:      # ~15 fps
                t_draw = now
                draw(hz, frames, lost, crc_err, status, raws, hist, cal, msg)

            if not n:
                time.sleep(0.001)

    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW)
        key_restore(old_tty)
        try:
            ser.write(b"x")
        except Exception:
            pass
        ser.close()
        if csv:
            csv.close()
        print(f"\n{frames} frames received.")


def draw(hz, frames, lost, crc_err, status, raws, hist, cal, msg):
    out = [HOME]
    warn = "" if hz >= 199.0 else "  << BELOW 200 Hz"
    out.append(f"\033[K 6ch encoder monitor    {hz:6.2f} Hz   frames {frames:<8d} "
               f"lost {lost:<5d} crc_err {crc_err:<4d}{warn}\n")
    out.append("\033[K" + "-" * 92 + "\n")
    out.append("\033[K CH   raw      mV       angle      sd(LSB)   range(mV)      position\n")
    out.append("\033[K" + "-" * 92 + "\n")

    for c in range(NCH):
        raw = raws[c]
        mv = raw * LSB_UV / 1000.0
        h = hist[c]
        if len(h) > 2:
            m = sum(h) / len(h)
            sd = (sum((x - m) ** 2 for x in h) / len(h)) ** 0.5
        else:
            sd = 0.0

        d = cal.deg(c, raw)
        ang = f"{d:7.2f}\u00b0" if d is not None else "  --.--  "

        lo, hi = cal.lo[c], cal.hi[c]
        if lo is not None and hi is not None:
            rng = f"{lo * LSB_UV / 1000:6.1f}-{hi * LSB_UV / 1000:6.1f}"
        else:
            rng = "     -       "

        flags = []
        if status & (1 << c):
            flags.append("I2C_ERR")
        if raw > 32000:
            flags.append("CLIP")
        if sd > 20:
            flags.append("NOISY")
        tag = " ".join(flags)

        out.append(f"\033[K  {c}  {raw:7d} {mv:8.2f} {ang}  {sd:7.1f}   {rng}  "
                   f"{bar(cal.frac(c, raw))} {tag}\n")

    out.append("\033[K" + "-" * 92 + "\n")
    out.append(f"\033[K {msg}\n")
    out.append("\033[K [r] reset range   [s] save calib   [z] set zero   [q] quit\n")
    out.append("\033[K sd under 20 LSB at rest is healthy. CLIP means over +/-2.048V.\n")
    out.append("\033[J")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


# ------------------------------------------------------------------- plot

def run_plot(args):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        sys.exit("matplotlib not installed.\n"
                 "  sudo apt install python3-matplotlib\n"
                 "  (or: pip install matplotlib)")

    port = args.port or find_port()
    ser = serial.Serial(port, 2000000, timeout=0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    if not args.no_start:
        ser.write(b"r")
        time.sleep(0.15)
        ser.reset_input_buffer()

    N = 1000                      # 5 seconds at 200 Hz
    ys = [deque([0.0] * N, maxlen=N) for _ in range(NCH)]
    buf = bytearray()

    fig, ax = plt.subplots(figsize=(11, 6))
    lines = [ax.plot(range(N), list(ys[c]), lw=1.0, label=f"CH{c}")[0]
             for c in range(NCH)]
    ax.set_xlabel("sample (200 Hz, 5 s window)")
    ax.set_ylabel("mV")
    ax.set_ylim(-50, 2100)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", ncol=6, fontsize=8)
    ax.set_title("6ch encoder - live")

    def update(_):
        n = ser.in_waiting
        if n:
            buf.extend(ser.read(n))
        while len(buf) >= FRAME_LEN:
            i = buf.find(SYNC)
            if i < 0:
                del buf[:-1]
                break
            if i > 0:
                del buf[:i]
                continue
            if len(buf) < FRAME_LEN:
                break
            f = bytes(buf[:FRAME_LEN])
            if crc16(f[:22]) != struct.unpack_from("<H", f, 22)[0]:
                del buf[:2]
                continue
            del buf[:FRAME_LEN]
            vals = struct.unpack_from("<6h", f, 8)
            for c in range(NCH):
                ys[c].append(vals[c] * LSB_UV / 1000.0)
        for c in range(NCH):
            lines[c].set_ydata(list(ys[c]))
        return lines

    anim = FuncAnimation(fig, update, interval=40, blit=True, cache_frame_data=False)
    try:
        plt.show()
    finally:
        try:
            ser.write(b"x")
        except Exception:
            pass
        ser.close()


if __name__ == "__main__":
    main()
