#!/usr/bin/env python3
"""
pp_check.py — 실기 점검 통합 모니터 (CAN + Teensy 엔코더 한 화면)

can_monitor.py(CAN 전용)와 teensy_monitor.py(엔코더 전용)로 갈라져 있던 것을
하나로 합쳤다. 20260903 에 엔코더가 CAN → Teensy 로 옮겨가면서 "값이 제대로
들어오나"를 확인하려면 두 화면을 동시에 봐야 했기 때문이다.

═══════════════════════════════════════════════════════════════════════════
 아침에 하는 순서 (이 순서대로 하면 문제를 층별로 가를 수 있다)
═══════════════════════════════════════════════════════════════════════════

 1단계  전원만 넣고, 아무것도 안 띄운 상태에서 — **값이 들어오나**
        python3 pp_check.py
        · CAN 보드 1~16 이 전부 500 Hz 로 오나
        · Teensy 6채널이 200 Hz 로 오나 (유실 0, CRC오류 0)
        · 축을 손으로 움직여 raw 가 따라 도나  ← 배선 확인은 이게 유일하다

 2단계  보정 2점 잡기 (아직 안 잡았다면)
        python3 pp_check.py --calib
        축을 0° 와 90° 에 놓고 키를 눌러 찍으면 config 에 넣을 줄을 출력해 준다.

 3단계  지령 부하를 줘서 — **200 Hz 로 쏴도 통신이 버티나**
        python3 pp_check.py --tx 200
        · 보드 수신이 얼마나 깎이는지 (예상: 500 → 약 260 Hz)
        · Teensy 는 CAN 과 무관하므로 200 Hz 를 그대로 유지해야 한다
          ← 여기서 Teensy 가 같이 떨어지면 USB/CPU 문제지 CAN 문제가 아니다

 4단계  브리지·제어기를 띄운 뒤 — **제어가 돌 때도 값이 멀쩡한가**
        (다른 창) ros2 launch can_powerpack control.launch.py
        python3 pp_check.py
        브리지가 떠 있으면 **자동으로 ROS 모드**로 바뀐다. 시리얼·CAN 송신을
        절대 건드리지 않고 토픽으로만 본다.

═══════════════════════════════════════════════════════════════════════════
 자동으로 피하는 충돌 두 가지 (중요)
═══════════════════════════════════════════════════════════════════════════
 · **시리얼**: /dev/ttyACM0 을 두 프로세스가 열면 바이트가 무작위로 쪼개져
   양쪽 다 깨진다. can_bridge_node 가 떠 있으면 시리얼을 **열지 않고**
   board/analog_raw 토픽에서 받는다.
 · **CAN 송신**: 브리지가 떠 있으면 --tx 를 거부한다. 같은 0x100/0x101 에
   둘이 쓰면 서로 덮어쓴다.
 CAN **수신**은 어느 경우든 직접 읽는다 — 브리지를 거치지 않은 실측이 필요하다.
 (Kvaser 는 핸들마다 프레임 사본을 주므로 읽기는 공존해도 된다.)

 busOff() 는 절대 부르지 않는다 — 공유 채널을 내리면 어댑터가 BUS_OFF+TX_PEND
 로 잠겨 재연결해야 풀린다 (20260902 에 겪음).
"""
import argparse
import os
import struct
import subprocess
import sys
import threading
import time
import unicodedata
from collections import deque

# ── CAN ─────────────────────────────────────────────────────────────────────
CAN_CHANNEL = 0
DATA_BITRATE = 5000000
FD_TSEG1, FD_TSEG2, FD_SJW = 11, 4, 4
BOARD_ID_BASE = 0x120
PWM_BOARDS = 16                 # CAN 에 남은 보드 (17~22 는 Teensy 로 이관)
CMD_IDS = (0x100, 0x101, 0x102)
GROUPS = [(0x100, 1, 10, 64, 60), (0x101, 11, 17, 48, 42)]   # 브리지와 동일

# ── Teensy ──────────────────────────────────────────────────────────────────
T_FRAME_LEN = 24
T_SYNC = b"\xAA\x55"
T_NCH = 6
T_BAUD = 2000000

HIDE, SHOW, HOME, CLR, CLREOS = "\033[?25l", "\033[?25h", "\033[H", "\033[K", "\033[J"


# ════════════════════════════════════════════════════════════════════════════
#  공통 유틸
# ════════════════════════════════════════════════════════════════════════════

def dwidth(t):
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in t)


def pad(t, w, right=True):
    d = w - dwidth(t)
    return (" " * max(0, d)) + t if right else t + (" " * max(0, d))


def proc_running(name):
    """정확히 그 실행파일이 도는지 (인자에 이름이 스쳐도 안 잡히게)."""
    r = subprocess.run(["pgrep", "-af", name], capture_output=True, text=True)
    if r.returncode != 0:
        return False
    for ln in r.stdout.splitlines():
        _, _, cmd = ln.partition(" ")
        if cmd and os.path.basename(cmd.split()[0]) == name:
            return True
    return False


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


class Rate:
    """1초 창 유량계. 값을 넣으면 최근 1초 주파수를 돌려준다."""

    def __init__(self):
        self.n = 0
        self.t = time.perf_counter()
        self.hz = 0.0

    def tick(self, k=1):
        self.n += k
        now = time.perf_counter()
        if now - self.t >= 1.0:
            self.hz = self.n / (now - self.t)
            self.n = 0
            self.t = now
        return self.hz


# ════════════════════════════════════════════════════════════════════════════
#  설정 읽기 (보정값)
# ════════════════════════════════════════════════════════════════════════════

def find_config():
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (
        os.path.join(here, "..", "..", "..", "install", "can_powerpack", "share",
                     "can_powerpack", "config", "powerpack_config.yaml"),
        os.path.join(here, "..", "config", "powerpack_config.yaml"),
    ):
        p = os.path.normpath(p)
        if os.path.exists(p):
            return p
    return None


# 압력 보정은 **can_monitor.py 를 그대로 import 해서 쓴다.** 표를 복제하면
# 단일 출처가 깨진다 — 예전에 값이 갈려 0° 가 158° 로 읽힌 적이 있다.
# can_monitor 는 import 안전하다 (CAN 접근이 전부 함수 안, __main__ 가드 있음).
try:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import can_monitor as _cm
except Exception as _e:      # pragma: no cover
    _cm = None
    _CM_ERR = str(_e)


def load_calib():
    """(teensy 2점, 압력보정 출처, 설정경로) 를 돌려준다."""
    tcal = {c: None for c in range(T_NCH)}
    path = find_config()
    if path:
        try:
            import yaml
            with open(path) as f:
                y = yaml.safe_load(f)
            br = (y.get("/pack2/can_bridge") or {}).get("ros__parameters", {}) or {}
            ch = (br.get("TeensyEncoder") or {}).get("channels", {}) or {}
            for k, v in ch.items():
                try:
                    c = int(k)
                except (TypeError, ValueError):
                    continue
                r0, r90 = v.get("raw_0deg"), v.get("raw_90deg")
                if (r0 is not None and r90 is not None
                        and abs(float(r90) - float(r0)) > 1e-9):
                    tcal[c] = (float(r0), float(r90))
        except Exception:
            pass
    src = getattr(_cm, "_CALIB_SOURCE", None) if _cm else "can_monitor import 실패"
    return tcal, src, path


def t_angle(raw, cal):
    """Teensy raw → 도. 보정이 없으면 None (0° 라고 거짓말하지 않는다)."""
    if cal is None:
        return None
    r0, r90 = cal
    return (raw - r0) * 90.0 / (r90 - r0)


# ════════════════════════════════════════════════════════════════════════════
#  CAN 수신
# ════════════════════════════════════════════════════════════════════════════

class CanRx(threading.Thread):
    """보드 프레임을 직접 읽어 압력 raw·전류·보드별 수신율을 유지한다."""

    daemon = True

    def __init__(self, ch):
        super().__init__()
        self.ch = ch
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.press = {}                                    # bid -> raw
        self.cur = {}                                      # bid -> [i1,i2,i3]
        self.cnt = {b: 0 for b in range(1, PWM_BOARDS + 1)}
        self.hz = {b: 0.0 for b in range(1, PWM_BOARDS + 1)}
        self.total = Rate()
        self.cmd = Rate()
        self.total_hz = 0.0
        self.cmd_hz = 0.0
        self.err = 0
        self._t = time.perf_counter()

    def run(self):
        from canlib import canlib
        while not self.stop.is_set():
            try:
                m = self.ch.read(timeout=50)
            except (canlib.canNoMsg, canlib.canError):
                self._roll()
                continue
            except Exception:
                self.err += 1
                continue
            self.total_hz = self.total.tick()
            if m.id in CMD_IDS:
                self.cmd_hz = self.cmd.tick()
                continue
            b = m.id - BOARD_ID_BASE
            if not (1 <= b <= PWM_BOARDS) or len(m.data) < 8:
                continue
            i1, i2, i3, p = struct.unpack_from("<HHHH", bytes(m.data), 0)
            with self.lock:
                self.press[b] = p
                self.cur[b] = [i1, i2, i3]
                self.cnt[b] += 1
            self._roll()

    def _roll(self):
        now = time.perf_counter()
        if now - self._t < 1.0:
            return
        dt = now - self._t
        self._t = now
        with self.lock:
            for b in self.cnt:
                self.hz[b] = self.cnt[b] / dt
                self.cnt[b] = 0

    def snap(self):
        with self.lock:
            return dict(self.press), dict(self.cur), dict(self.hz)


def raw_to_kpa(raw, board_id):
    """압력 raw → kPa. can_monitor 의 보정표와 공식을 그대로 쓴다."""
    if _cm is None:
        return float("nan")
    adc_mv = max(0.0, min(3300.0, raw * 3300.0 / 4095.0))
    return _cm.calc_pressure_kpa(_cm.calc_original_voltage_mv(adc_mv), board_id)


# ════════════════════════════════════════════════════════════════════════════
#  Teensy 수신 (시리얼 직접)
# ════════════════════════════════════════════════════════════════════════════

class TeensySerial(threading.Thread):
    daemon = True

    def __init__(self, port=None):
        super().__init__()
        self.want_port = port
        self.port = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.raw = [0] * T_NCH
        self.hist = [deque(maxlen=200) for _ in range(T_NCH)]
        self.rate = Rate()
        self.hz = 0.0
        self.frames = 0
        self.lost = 0
        self.crc_err = 0
        self.status = 0
        self.last = 0.0
        self.err = ""
        self.ser = None

    @staticmethod
    def find_port():
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            blob = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
            if "teensy" in blob or "16c0" in blob:
                return p.device
        return None

    def run(self):
        import serial
        while not self.stop.is_set():
            if self.ser is None:
                port = self.want_port or self.find_port()
                if not port:
                    self.err = "Teensy 포트를 못 찾음"
                    time.sleep(1.0)
                    continue
                try:
                    self.ser = serial.Serial(port, T_BAUD, timeout=0)
                except Exception as e:
                    self.err = f"열기 실패: {e}"
                    self.ser = None
                    time.sleep(1.0)
                    continue
                self.port = port
                self.err = ""
                time.sleep(0.3)
                self.ser.reset_input_buffer()
                self.ser.write(b"r")        # ← 이게 없으면 한 바이트도 안 온다
                time.sleep(0.15)
                self.ser.reset_input_buffer()
                buf = bytearray()
                prev_seq = None

            try:
                n = self.ser.in_waiting
                if n:
                    buf.extend(self.ser.read(n))
                else:
                    time.sleep(0.001)
            except Exception as e:
                self.err = f"read 실패: {e}"
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                continue

            if len(buf) > 8192:
                del buf[:-4096]

            while len(buf) >= T_FRAME_LEN:
                i = buf.find(T_SYNC)
                if i < 0:
                    del buf[:-1]
                    break
                if i:
                    del buf[:i]
                    continue
                if len(buf) < T_FRAME_LEN:
                    break
                f = bytes(buf[:T_FRAME_LEN])
                if crc16(f[:22]) != struct.unpack_from("<H", f, 22)[0]:
                    self.crc_err += 1
                    del buf[:2]            # 2바이트만 버리고 재동기
                    continue
                del buf[:T_FRAME_LEN]
                seq = struct.unpack_from("<H", f, 2)[0]
                vals = struct.unpack_from("<6h", f, 8)
                st = struct.unpack_from("<H", f, 20)[0]
                if prev_seq is not None:
                    self.lost += (seq - prev_seq - 1) & 0xFFFF
                prev_seq = seq
                self.frames += 1
                self.hz = self.rate.tick()
                self.last = time.perf_counter()
                with self.lock:
                    self.raw = list(vals)
                    self.status = st
                    for c in range(T_NCH):
                        self.hist[c].append(vals[c])

    def close(self):
        self.stop.set()
        time.sleep(0.15)
        if self.ser:
            try:
                self.ser.write(b"x")       # 스트리밍 정지
                self.ser.close()
            except Exception:
                pass

    def snap(self):
        with self.lock:
            sd = []
            for c in range(T_NCH):
                h = self.hist[c]
                if len(h) > 2:
                    m = sum(h) / len(h)
                    sd.append((sum((x - m) ** 2 for x in h) / len(h)) ** 0.5)
                else:
                    sd.append(0.0)
            return list(self.raw), sd, self.status


# ════════════════════════════════════════════════════════════════════════════
#  ROS 모드 (브리지가 떠 있을 때)
# ════════════════════════════════════════════════════════════════════════════

class RosTaps:
    """board/analog_raw·analog·sensors·pwm_cmd 를 구독해 값과 발행률을 잰다."""

    def __init__(self, ns="/pack2"):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray, UInt16MultiArray
        self.ok = False
        self.lock = threading.Lock()
        self.raw = [0] * T_NCH
        self.ang = [None] * T_NCH
        self.rate = {k: Rate() for k in ("analog", "analog_raw", "sensors", "pwm_cmd")}
        self.hz = {k: 0.0 for k in self.rate}
        self.last_analog = 0.0
        rclpy.init(args=None)
        self.node = Node("pp_check")

        def mk(key, typ, topic, cb):
            self.node.create_subscription(typ, f"{ns}/{topic}", cb, 10)

        def on_raw(m):
            with self.lock:
                self.raw = [int(v) for v in m.data[:T_NCH]]
            self.hz["analog_raw"] = self.rate["analog_raw"].tick()

        def on_ang(m):
            with self.lock:
                self.ang = [float(v) for v in m.data[:T_NCH]]
            self.hz["analog"] = self.rate["analog"].tick()
            self.last_analog = time.perf_counter()

        def on_sens(m):
            self.hz["sensors"] = self.rate["sensors"].tick()

        def on_pwm(m):
            self.hz["pwm_cmd"] = self.rate["pwm_cmd"].tick()

        mk("analog_raw", UInt16MultiArray, "board/analog_raw", on_raw)
        mk("analog", Float64MultiArray, "board/analog", on_ang)
        mk("sensors", UInt16MultiArray, "board/sensors", on_sens)
        mk("pwm_cmd", UInt16MultiArray, "board/pwm_cmd", on_pwm)
        self.ok = True
        self._stop = threading.Event()
        self.th = threading.Thread(target=self._spin, daemon=True)
        self.th.start()

    def _spin(self):
        import rclpy
        # spin_once 루프로 돌린다. rclpy.spin() 은 shutdown 때 스레드가 물려 있어
        # "terminate called without an active exception" 으로 코어를 뱉는다.
        while not self._stop.is_set():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            except Exception:
                break

    def snap(self):
        with self.lock:
            return list(self.raw), list(self.ang)

    def close(self):
        try:
            self._stop.set()
            self.th.join(timeout=1.0)
            import rclpy
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════════════
#  CAN 송신 부하 (--tx)
# ════════════════════════════════════════════════════════════════════════════

class CanTx(threading.Thread):
    daemon = True

    def __init__(self, ch, hz):
        super().__init__()
        from canlib import canlib
        self.ch = ch
        self.hz = hz
        self.flags = canlib.MessageFlag.FDF | canlib.MessageFlag.BRS
        self.stop = threading.Event()
        self.rate = Rate()
        self.actual = 0.0
        self.late = 0

    def run(self):
        from canlib import Frame
        hb = 0
        nxt = time.perf_counter()
        while not self.stop.is_set():
            hb = (hb + 1) & 0xFF
            for cid, b0, b1, total, moff in GROUPS:
                p = bytearray(total)          # 전부 0 — 밸브는 움직이지 않는다
                p[moff] = 0
                p[moff + 1] = 0
                p[min(moff + 5, total - 1)] = hb
                try:
                    self.ch.write(Frame(id_=cid, data=bytes(p), flags=self.flags))
                except Exception:
                    pass
            self.actual = self.rate.tick()
            nxt += 1.0 / self.hz
            d = nxt - time.perf_counter()
            if d > 0:
                time.sleep(d)
            else:
                self.late += 1
                nxt = time.perf_counter()

    def send_zero(self):
        from canlib import Frame
        for cid, b0, b1, total, moff in GROUPS:
            try:
                self.ch.write(Frame(id_=cid, data=bytes(total), flags=self.flags))
            except Exception:
                pass


# ════════════════════════════════════════════════════════════════════════════
#  화면
# ════════════════════════════════════════════════════════════════════════════

# 파이프·리다이렉트로 돌리면 stdin 이 TTY 가 아니다. 그때 termios 를 부르면
# "Inappropriate ioctl for device" 로 죽는다 — 키 입력만 조용히 포기한다.
def _tty():
    try:
        return sys.stdin.isatty()
    except Exception:
        return False


def key_setup():
    if os.name == "nt" or not _tty():
        return None
    try:
        import termios, tty
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return old
    except Exception:
        return None


def key_restore(old):
    if old is None or os.name == "nt":
        return
    try:
        import termios
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    except Exception:
        pass


def key_get():
    if os.name == "nt":
        import msvcrt
        return msvcrt.getch().decode(errors="ignore") if msvcrt.kbhit() else None
    if not _tty():
        return None
    try:
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
    except Exception:
        pass
    return None


def draw(a, can, teensy, ros, tx, tcal, psrc, cfgpath, cal0, cal90, msg, t0):
    o = [HOME]
    mode = ("ROS (브리지 실행 중 — 시리얼·송신 안 함)" if ros else
            (f"직접 + 지령 {tx.hz:g} Hz 송신" if tx else "직접 (송신 없음)"))
    o.append(f"{CLR} pp_check   경과 {time.perf_counter()-t0:6.1f} s   모드: {mode}\n")
    o.append(f"{CLR} 보정: {psrc}\n")
    o.append(f"{CLR}{'─'*96}\n")

    verdict = []

    # ── CAN ────────────────────────────────────────────────────────────────
    if can is None:
        o.append(f"{CLR} [CAN] 열지 않음 (--no-can 또는 어댑터 없음)\n")
        verdict.append(("주의", "CAN 을 안 봤다 — 압력·밸브는 확인되지 않았다"))
        press = cur = hz = {}
        alive = []
    else:
      press, cur, hz = can.snap()
      alive = [b for b in range(1, PWM_BOARDS + 1) if hz.get(b, 0) > 1.0]
      lo = min((hz[b] for b in alive), default=0.0)
      lob = min(alive, key=lambda b: hz[b], default=0)
      o.append(f"{CLR} [CAN] 버스 {can.total_hz:6.0f} f/s   보드 {len(alive)}/{PWM_BOARDS} 수신"
               f"   최저 {lo:5.0f} Hz (board {lob})"
               f"   지령 {can.cmd_hz:5.0f} f/s\n")
      o.append(f"{CLR}  " + "".join(pad(t, w) for t, w in (
          ("board", 6), ("압력 kPa", 11), ("raw", 7),
          ("I1 mV", 8), ("I2 mV", 8), ("I3 mV", 8), ("Rx Hz", 8))) + "\n")
      for b in range(1, PWM_BOARDS + 1):
          r = press.get(b)
          c = cur.get(b, [0, 0, 0])
          h = hz.get(b, 0.0)
          if r is None:
              o.append(f"{CLR}  {b:>6}{pad('— 수신 없음 —', 11)}"
                       f"{pad('', 7)}{pad('', 8)}{pad('', 8)}{pad('', 8)}{0:8.0f}\n")
              continue
          kpa = raw_to_kpa(r, b)
          mark = "" if h > 150 else ("  <<" if h > 1 else "  << 두절")
          o.append(f"{CLR}  {b:>6}{kpa:11.1f}{r:7d}"
                   f"{c[0]*3300/4095:8.0f}{c[1]*3300/4095:8.0f}{c[2]*3300/4095:8.0f}"
                   f"{h:8.0f}{mark}\n")
      if len(alive) == PWM_BOARDS:
          verdict.append(("OK", f"CAN 보드 {PWM_BOARDS}개 전부 수신 (최저 {lo:.0f} Hz)"))
      else:
          miss = [b for b in range(1, PWM_BOARDS + 1) if b not in alive]
          verdict.append(("실패", f"CAN 보드 미수신: {miss} — 배선·전원·펌웨어"))

    # ── Teensy ─────────────────────────────────────────────────────────────
    o.append(f"{CLR}{'─'*96}\n")
    if ros:
        raws, angs = ros.snap()
        thz = ros.hz["analog_raw"]
        age = time.perf_counter() - ros.last_analog if ros.last_analog else 999
        o.append(f"{CLR} [엔코더] board/analog {ros.hz['analog']:6.1f} Hz   "
                 f"analog_raw {thz:6.1f} Hz   마지막 수신 {age*1000:.0f} ms 전\n")
        sd = [0.0] * T_NCH
        status = 0
    else:
        raws, sd, status = teensy.snap()
        thz = teensy.hz
        angs = [t_angle(raws[c], tcal.get(c)) for c in range(T_NCH)]
        o.append(f"{CLR} [Teensy] {thz:6.1f} Hz   유실 {teensy.lost}   CRC오류 {teensy.crc_err}"
                 f"   status 0x{status:04X}   {teensy.port or teensy.err}\n")
    o.append(f"{CLR}  " + "".join(pad(t, w) for t, w in (
        ("ch", 4), ("raw", 8), ("각도°", 10), ("1s σ", 9), ("보정", 22))) + "\n")
    ncal = 0
    for c in range(T_NCH):
        cal = tcal.get(c)
        if cal:
            ncal += 1
            cs = f"{cal[0]:.0f} → {cal[1]:.0f}"
        else:
            cs = "**미보정**"
        astr = f"{angs[c]:10.2f}" if angs[c] is not None else pad("--.--", 10)
        o.append(f"{CLR}  {c:>4}{raws[c]:8d}{astr}{sd[c]:9.2f}  {pad(cs, 20, False)}\n")

    if thz >= 195:
        verdict.append(("OK", f"엔코더 {thz:.0f} Hz"))
    elif thz > 0:
        verdict.append(("주의", f"엔코더 {thz:.0f} Hz — 200 Hz 여야 한다"))
    else:
        verdict.append(("실패", "엔코더 프레임 0 — 케이블·전원 확인"))
    if not ros:
        if teensy.crc_err == 0 and teensy.lost == 0:
            verdict.append(("OK", "엔코더 유실 0, CRC오류 0"))
        else:
            verdict.append(("주의", f"엔코더 유실 {teensy.lost}, CRC오류 {teensy.crc_err}"))
        if status:
            verdict.append(("실패", f"ADS1115 I2C 오류 status=0x{status:04X}"))
    if ncal < T_NCH:
        verdict.append(("주의", f"엔코더 {T_NCH-ncal}채널 미보정 — 각도가 0° 로 나간다 "
                                "(--calib 로 잡을 것)"))

    # ── ROS ────────────────────────────────────────────────────────────────
    if ros:
        o.append(f"{CLR}{'─'*96}\n")
        o.append(f"{CLR} [ROS] board/sensors {ros.hz['sensors']:6.1f} Hz  (= 제어 루프 주기)"
                 f"   board/pwm_cmd {ros.hz['pwm_cmd']:6.1f} Hz\n")
        s = ros.hz["sensors"]
        if s >= 190:
            verdict.append(("OK", f"제어 루프 {s:.0f} Hz"))
        elif s > 0:
            verdict.append(("주의", f"제어 루프 {s:.0f} Hz — 200 Hz 를 기대했다"))

    # ── 송신 ───────────────────────────────────────────────────────────────
    if tx:
        o.append(f"{CLR}{'─'*96}\n")
        o.append(f"{CLR} [송신] 목표 {tx.hz:g} Hz  실제 {tx.actual:6.1f} Hz  "
                 f"주기 미달 {tx.late}회   (페이로드 전부 0 — 밸브는 안 움직인다)\n")

    # ── 판정 ───────────────────────────────────────────────────────────────
    o.append(f"{CLR}{'─'*96}\n")
    for tag, txt in verdict:
        col = {"OK": "\033[32m", "주의": "\033[33m", "실패": "\033[31m"}[tag]
        o.append(f"{CLR} {col}[{tag}]\033[0m {txt}\n")

    o.append(f"{CLR}{'─'*96}\n")
    if a.calib:
        o.append(f"{CLR} 보정 모드: 축을 0°에 놓고 [0], 90°에 놓고 [9], 출력 [p], 종료 [q]\n")
        c0 = "  ".join("--" if v is None else f"{v:.0f}" for v in cal0)
        c9 = "  ".join("--" if v is None else f"{v:.0f}" for v in cal90)
        o.append(f"{CLR}   0° 찍힌 값: {c0}\n")
        o.append(f"{CLR}  90° 찍힌 값: {c9}\n")
    else:
        o.append(f"{CLR} [q] 종료   [r] 카운터 리셋\n")
    if msg:
        o.append(f"{CLR} {msg}\n")
    o.append(CLREOS)
    sys.stdout.write("".join(o))
    sys.stdout.flush()


# ════════════════════════════════════════════════════════════════════════════
#  main
# ════════════════════════════════════════════════════════════════════════════

def main():
    global a
    ap = argparse.ArgumentParser(
        description="실기 점검 통합 모니터 (CAN + Teensy 엔코더)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("═══", 1)[-1] if "═══" in __doc__ else None)
    ap.add_argument("--tx", type=float, default=0.0,
                    help="지령 프레임을 이 주파수로 송신하며 영향을 본다 (예: 200). "
                         "페이로드는 전부 0 이라 밸브는 안 움직인다.")
    ap.add_argument("--calib", action="store_true",
                    help="2점 보정 모드 — 0°/90° 를 찍어 config 에 넣을 줄을 출력한다")
    ap.add_argument("--port", default=None, help="Teensy 포트 (기본 자동탐색)")
    ap.add_argument("--ns", default="/pack2", help="ROS 네임스페이스 (기본 /pack2)")
    ap.add_argument("--no-can", action="store_true", help="CAN 을 아예 열지 않는다")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="이 시간 뒤 자동 종료 (0 = 수동). 로그로 남길 때 쓴다.")
    ap.add_argument("--force-serial", action="store_true",
                    help="브리지가 떠 있어도 시리얼을 연다 (**바이트가 쪼개진다 — 위험**)")
    a = ap.parse_args()

    bridge = proc_running("can_bridge_node")
    use_ros = bridge and not a.force_serial

    if bridge and a.tx > 0:
        print("[중단] can_bridge_node 가 돌고 있다. 같은 0x100/0x101 에 둘이 쓰면 "
              "서로 덮어쓴다.\n        브리지를 내리고 --tx 를 쓸 것.")
        return 1
    if bridge and a.force_serial:
        print("[경고] 브리지가 시리얼을 잡고 있는데 --force-serial 이다. "
              "바이트가 두 프로세스로 쪼개져 양쪽 다 깨진다.")
        time.sleep(2)

    tcal, psrc, cfgpath = load_calib()

    # ── CAN 열기 (읽기 전용) ────────────────────────────────────────────────
    can = ch = None
    if not a.no_can:
        try:
            from canlib import canlib
            ch = canlib.openChannel(channel=CAN_CHANNEL, flags=canlib.Open.CAN_FD)
            try:
                ch.setBusParams(canlib.canBITRATE_1M)
                ch.setBusParamsFd(DATA_BITRATE, FD_TSEG1, FD_TSEG2, FD_SJW)
            except Exception:
                pass                       # 이미 설정돼 있으면 그대로 쓴다
            ch.busOn()
            can = CanRx(ch)
            can.start()
        except Exception as e:
            print(f"[경고] CAN 을 열 수 없다: {e}\n        엔코더만 본다.")
            can = ch = None

    # ── 엔코더 소스 ────────────────────────────────────────────────────────
    teensy = ros = None
    if use_ros:
        try:
            ros = RosTaps(a.ns)
        except Exception as e:
            print(f"[경고] ROS 구독 실패 ({e}) — 시리얼로 넘어간다.")
            use_ros = False
    if not use_ros:
        teensy = TeensySerial(a.port)
        teensy.start()

    tx = None
    if a.tx > 0 and ch is not None:
        tx = CanTx(ch, a.tx)
        tx.start()

    cal0 = [None] * T_NCH
    cal90 = [None] * T_NCH
    msg = ""
    t0 = time.perf_counter()
    old = key_setup()
    sys.stdout.write(HIDE + "\033[2J")
    try:
        while True:
            k = key_get()
            if k:
                k = k.lower()
                if k == "q":
                    break
                if k == "r" and teensy:
                    teensy.lost = teensy.crc_err = 0
                    msg = "카운터 리셋"
                if a.calib and k in ("0", "9"):
                    raws = (ros.snap()[0] if ros else teensy.snap()[0])
                    tgt = cal0 if k == "0" else cal90
                    for c in range(T_NCH):
                        tgt[c] = float(raws[c])
                    msg = f"{'0' if k=='0' else '90'}° 찍음"
                if a.calib and k == "p":
                    lines = ["", "  # config/powerpack_config.yaml 의",
                             "  # /pack2/can_bridge → TeensyEncoder.channels 에 넣을 것:",
                             "    TeensyEncoder:", "      channels:"]
                    for c in range(T_NCH):
                        if cal0[c] is None or cal90[c] is None:
                            lines.append(f'        "{c}": {{ raw_0deg: 0.0, raw_90deg: 0.0 }}'
                                         f"   # ← 아직 안 찍음")
                        else:
                            lines.append(f'        "{c}": {{ raw_0deg: {cal0[c]:.1f}, '
                                         f"raw_90deg: {cal90[c]:.1f} }}")
                    sys.stdout.write(SHOW)
                    key_restore(old)
                    print("\n".join(lines))
                    print("\n(엔터를 누르면 모니터로 돌아간다)")
                    input()
                    old = key_setup()
                    sys.stdout.write(HIDE + "\033[2J")

            draw(a, can, teensy, ros, tx, tcal, psrc, cfgpath, cal0, cal90, msg, t0)
            if a.seconds > 0 and time.perf_counter() - t0 >= a.seconds:
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW)
        key_restore(old)
        if tx:
            tx.stop.set()
            time.sleep(0.1)
            tx.send_zero()                 # 전 밸브 0 으로 마무리
        if can:
            can.stop.set()
        if teensy:
            teensy.close()
        if ros:
            ros.close()
        time.sleep(0.2)
        if ch is not None:
            try:
                ch.close()                 # busOff() 는 절대 부르지 않는다
            except Exception:
                pass
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
