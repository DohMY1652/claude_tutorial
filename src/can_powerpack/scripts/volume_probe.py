#!/usr/bin/env python3
"""
volume_probe.py — 밸브를 닫고 팔을 손으로 움직여 **챔버 부피 곡선 V(θ)** 를 잰다

═══════════════════════════════════════════════════════════════════════════
 원리
═══════════════════════════════════════════════════════════════════════════
밸브를 전부 닫으면 챔버는 닫힌 계다. 등온 이상기체라

    P·V = const   →   V(θ) ∝ 1 / P(θ)

즉 **압력의 역수가 곧 부피 곡선**이다. 절대 부피 상수 C 는 몰라도 된다 —
1/P 를 θ_rad 에 직접 피팅하면
    1/P = a + b·θ_rad,   a = V_off/C,  b = ±m·g/C
      →  m = ±(b/a)·V_off/g          ← C 가 소거된다
로 배수 m 이 바로 나온다 (V_off = tank + A·off/1000, g = A·reel/1000).
기준 각도나 기준 압력을 고를 필요가 없다.

합성 데이터로 검증했다: 참 배수 2.40 → 양·음압 모두 2.40 으로 복원.
비선형(1.50→3.50) 도 2 차항으로 1.57→3.67 로 잡아낸다.

이 액추에이터는 단순 피스톤이 아니다 — 수축하면서 옆면도 같이 좁아져서
부피 변화가 A_바닥 × 스트로크 보다 크다. 그 배수(또는 비선형 곡선)를
여기서 실측한다.

기하 모델은
    V⁺(θ) = tank⁺ + A·(off⁺ + m·reel·θ_rad)
    V⁻(θ) = tank⁻ + A·(off⁻ − m·reel·θ_rad)
이고, m 이 Geometry.stroke_volume_mult 다. 1.0 이면 단순 피스톤.

═══════════════════════════════════════════════════════════════════════════
 하는 법
═══════════════════════════════════════════════════════════════════════════
 1. 챔버에 압력을 담아 둔다 (**중요 — 대기압이면 못 잰다**)
      제어기로 원하는 각도까지 올린 뒤 Ctrl-C 로 내린다.
      밸브가 닫히면 그 압력이 챔버에 갇힌다.
      P⁺ 가 대기압(101.3) 근처면 신호가 잡음에 묻힌다. 130 kPa 이상 권장.

 2. 제어기·브리지를 **완전히 내린다** (같은 CAN ID 로 서로 덮어쓴다)
      pkill -f "ros2 launch"; pkill -f can_bridge_node; pkill -f pp_controller

 3. 이 스크립트를 띄운다 — 즉시 전 채널 밸브를 0(폐쇄)으로 잡는다
      python3 volume_probe.py --axes 0

 4. **팔을 손으로 천천히 끝에서 끝까지** 움직인다. 왕복 2~3 회.
      · 느리게 (누설이 있으면 오래 걸릴수록 오차가 커진다)
      · 화면의 P⁺ 가 각도와 반대로, P⁻ 가 같이 움직이면 제대로 잡히는 것이다

 5. q 로 끝내면 CSV 를 쓰고 그 자리에서 피팅 결과를 보여 준다.

 나중에 다시 피팅만:
      python3 volume_probe.py --fit ~/result/volprobe_<ts>.csv

═══════════════════════════════════════════════════════════════════════════
 안전
═══════════════════════════════════════════════════════════════════════════
 · 채널 밸브(보드 5~16)는 **전부 0** 으로 고정한다. 그게 이 시험의 전제다.
 · 라인 릴리프(board1 v1)와 음압 admit(board2 v1)는 브리지의 안전상태와 똑같이
   **열어 둔다** — 펌프가 돌면 레일이 무한정 오르기 때문이다.
   `--hold-rail-closed` 로 끌 수 있지만, 그때는 **펌프를 반드시 꺼 둘 것.**
 · 종료 시(Ctrl-C 포함) 안전상태를 다시 보내고 끝난다.
 · busOff() 는 부르지 않는다 (어댑터가 잠긴다).
"""
import argparse
import math
import os
import struct
import subprocess
import sys
import threading
import time

# ── CAN ─────────────────────────────────────────────────────────────────────
CAN_CHANNEL = 0
DATA_BITRATE = 5000000
FD_TSEG1, FD_TSEG2, FD_SJW = 11, 4, 4
BOARD_ID_BASE = 0x120
PWM_BOARDS = 16
GROUPS = [(0x100, 1, 10, 64, 60), (0x101, 11, 17, 48, 42)]   # 브리지와 동일
TX_HZ = 50.0

# ── Teensy ──────────────────────────────────────────────────────────────────
T_FRAME_LEN, T_SYNC, T_NCH, T_BAUD = 24, b"\xAA\x55", 6, 2000000

# ── 기하 (config Geometry 와 같은 값. 피팅 기준으로만 쓴다) ──────────────────
PISTON_DIA_MM = 50.0
REEL_MM = 25.0
OFF_POS_MM, OFF_NEG_MM = 40.0, 90.0
TANK_POS_ML, TANK_NEG_ML = 50.0, 50.0
A_MM2 = math.pi * PISTON_DIA_MM * PISTON_DIA_MM / 4.0

POS_BD = [5, 6, 7, 8, 9, 10]
NEG_BD = [11, 12, 13, 14, 15, 16]

HIDE, SHOW, HOME, CLR, CLREOS = "\033[?25l", "\033[?25h", "\033[H", "\033[K", "\033[J"

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    import can_monitor as _cm          # 압력 보정 단일 출처
except Exception:
    _cm = None


def kpa(raw, bid):
    if _cm is None:
        return float("nan")
    adc_mv = max(0.0, min(3300.0, raw * 3300.0 / 4095.0))
    return _cm.calc_pressure_kpa(_cm.calc_original_voltage_mv(adc_mv), bid)


def crc16(d):
    c = 0xFFFF
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def proc_running(name):
    r = subprocess.run(["pgrep", "-af", name], capture_output=True, text=True)
    if r.returncode != 0:
        return False
    for ln in r.stdout.splitlines():
        _, _, cmd = ln.partition(" ")
        if cmd and os.path.basename(cmd.split()[0]) == name:
            return True
    return False


# ════════════════════════════════════════════════════════════════════════════
#  수집
# ════════════════════════════════════════════════════════════════════════════

class CanIO(threading.Thread):
    """전 채널 밸브를 0 으로 잡으면서 보드 압력을 읽는다."""

    daemon = True

    def __init__(self, ch, hold_rail_closed):
        super().__init__()
        from canlib import canlib
        self.ch = ch
        self.flags = canlib.MessageFlag.FDF | canlib.MessageFlag.BRS
        self.hold_rail_closed = hold_rail_closed
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.press = {}
        self.hz = {}
        self._cnt = {}
        self._t = time.perf_counter()

    def _frames(self):
        from canlib import Frame
        # 브리지 안전상태와 동일: 채널 밸브 0, 라인 릴리프·admit 개방
        tgt = {b: [0, 0, 0] for b in range(1, 23)}
        if not self.hold_rail_closed:
            tgt[1][0] = 4095        # board1 v1 = 양압 라인 릴리프
            tgt[2][0] = 4095        # board2 v1 = 음압 라인 admit
        out = []
        for cid, b0, b1, total, moff in GROUPS:
            p = bytearray(total)
            off = 0
            for b in range(b0, b1 + 1):
                struct.pack_into("<HHH", p, off, *tgt[b])
                off += 6
            p[moff] = 0
            p[moff + 1] = 0
            out.append(Frame(id_=cid, data=bytes(p), flags=self.flags))
        return out

    def send_safe(self, n=3):
        for _ in range(n):
            for f in self._frames():
                try:
                    self.ch.write(f)
                except Exception:
                    pass
            time.sleep(0.01)

    def run(self):
        from canlib import canlib
        frames = self._frames()
        nxt = time.perf_counter()
        while not self.stop.is_set():
            now = time.perf_counter()
            if now >= nxt:
                for f in frames:
                    try:
                        self.ch.write(f)
                    except Exception:
                        pass
                nxt = now + 1.0 / TX_HZ
            try:
                m = self.ch.read(timeout=5)
            except (canlib.canNoMsg, canlib.canError):
                self._roll()
                continue
            except Exception:
                continue
            b = m.id - BOARD_ID_BASE
            if 1 <= b <= PWM_BOARDS and len(m.data) >= 8:
                p = struct.unpack_from("<HHHH", bytes(m.data), 0)[3]
                with self.lock:
                    self.press[b] = p
                    self._cnt[b] = self._cnt.get(b, 0) + 1
            self._roll()

    def _roll(self):
        now = time.perf_counter()
        if now - self._t < 1.0:
            return
        dt = now - self._t
        self._t = now
        with self.lock:
            for b, c in self._cnt.items():
                self.hz[b] = c / dt
            self._cnt = {}

    def snap(self):
        with self.lock:
            return dict(self.press), dict(self.hz)


class Teensy(threading.Thread):
    daemon = True

    def __init__(self, port):
        super().__init__()
        self.port = port
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.raw = [0] * T_NCH
        self.n = 0
        self.err = ""

    @staticmethod
    def find():
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            blob = f"{p.description} {p.manufacturer or ''} {p.hwid}".lower()
            if "teensy" in blob or "16c0" in blob:
                return p.device
        return None

    def run(self):
        import serial
        port = self.port or self.find()
        if not port:
            self.err = "Teensy 포트를 못 찾음"
            return
        try:
            ser = serial.Serial(port, T_BAUD, timeout=0)
        except Exception as e:
            self.err = f"{port} 열기 실패: {e}"
            return
        self.port = port
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(b"r")
        time.sleep(0.15)
        ser.reset_input_buffer()
        buf = bytearray()
        while not self.stop.is_set():
            k = ser.in_waiting
            if k:
                buf.extend(ser.read(k))
            else:
                time.sleep(0.001)
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
                    del buf[:2]
                    continue
                del buf[:T_FRAME_LEN]
                v = struct.unpack_from("<6h", f, 8)
                with self.lock:
                    self.raw = list(v)
                    self.n += 1
        try:
            ser.write(b"x")
            ser.close()
        except Exception:
            pass

    def snap(self):
        with self.lock:
            return list(self.raw), self.n


def load_tcal():
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.path.join(here, "..", "..", "share", "can_powerpack", "config",
                           "powerpack_config.yaml"),
              os.path.join(here, "..", "config", "powerpack_config.yaml")):
        p = os.path.normpath(p)
        if not os.path.exists(p):
            continue
        try:
            import yaml
            y = yaml.safe_load(open(p))
            ch = ((y.get("/pack2/can_bridge") or {}).get("ros__parameters", {})
                  .get("TeensyEncoder", {}) or {}).get("channels", {}) or {}
            out = {}
            for k, v in ch.items():
                r0, r90 = v.get("raw_0deg"), v.get("raw_90deg")
                if r0 is not None and r90 is not None and abs(float(r90) - float(r0)) > 1e-9:
                    out[int(k)] = (float(r0), float(r90))
            if out:
                return out, p
        except Exception:
            pass
    return {}, None


# ════════════════════════════════════════════════════════════════════════════
#  피팅
# ════════════════════════════════════════════════════════════════════════════

def polyfit(xs, ys, deg):
    """작은 정규방정식 (numpy 없이)."""
    n = deg + 1
    M = [[sum(x ** (i + j) for x in xs) for j in range(n)] for i in range(n)]
    b = [sum(ys[k] * xs[k] ** i for k in range(len(xs))) for i in range(n)]
    for i in range(n):                       # 가우스 소거
        p = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-14:
            return None
        M[i], M[p] = M[p], M[i]
        b[i], b[p] = b[p], b[i]
        for r in range(i + 1, n):
            f = M[r][i] / M[i][i]
            for c in range(i, n):
                M[r][c] -= f * M[i][c]
            b[r] -= f * b[i]
    c = [0.0] * n
    for i in range(n - 1, -1, -1):
        s = b[i] - sum(M[i][j] * c[j] for j in range(i + 1, n))
        c[i] = s / M[i][i]
    return c


def fit_axis(th, P, off_mm, tank_ml, sign, label):
    """P·V=const 를 이용해 V(θ) 를 피팅하고 스트로크 배수 m 을 낸다.

    P·V = C  →  1/P(θ) = V(θ)/C.  기하 모델은
        V(θ) = V_off ± m·g·θ_rad ,   V_off = tank + A·off/1000 [mL],
                                     g     = A·reel/1000       [mL/rad]
    이므로 **1/P 를 θ_rad 에 직접 피팅**하면
        1/P = a + b·θ_rad  →  a = V_off/C,  b = ±m·g/C
        →  m = ±(b/a)·V_off/g          ← C 가 소거된다
    기준 각도·기준 압력을 고를 필요가 없다. 이게 핵심이다.

    sign = +1 : 각도가 커지면 부피가 커진다 (양압 챔버)
    sign = -1 : 각도가 커지면 부피가 작아진다 (음압 챔버)
    """
    good = [(a, p) for a, p in zip(th, P) if p == p and p > 20.0 and a == a]
    if len(good) < 200:
        print(f'    {label}: 표본 부족 ({len(good)})')
        return None
    th2 = [g[0] for g in good]
    P2 = [g[1] for g in good]
    span = max(th2) - min(th2)
    dP = max(P2) - min(P2)
    if span < 5.0:
        print(f'    {label}: 각도 범위가 {span:.1f}° 뿐이다 — 더 크게 움직일 것')
        return None
    if dP < 1.0:
        print(f'    {label}: 압력 변화가 {dP:.2f} kPa 뿐이다 — '
              f'챔버에 압력을 담고(≥130 kPa) 다시 할 것')
        return None

    V_off = tank_ml + A_MM2 * max(1.0, off_mm) / 1000.0    # mL, m 과 무관
    g = A_MM2 * REEL_MM / 1000.0                           # mL/rad (단순 피스톤)

    x = [a * math.pi / 180.0 for a in th2]                 # rad
    y = [1.0 / p for p in P2]

    res = {}
    for deg in (1, 2):
        c = polyfit(x, y, deg)
        if c is None:
            continue
        pred = [sum(c[i] * xx ** i for i in range(len(c))) for xx in x]
        ss = sum((y[k] - pred[k]) ** 2 for k in range(len(y)))
        ym = sum(y) / len(y)
        st = sum((v - ym) ** 2 for v in y)
        res[deg] = (c, 1.0 - ss / st if st > 1e-18 else 0.0, (ss / len(y)) ** 0.5)

    if 1 not in res:
        return None
    c1, r2_1, rms1 = res[1]
    a1, b1 = c1[0], c1[1]
    if abs(a1) < 1e-12:
        return None
    m = sign * (b1 / a1) * V_off / g

    print(f'    {label}')
    print(f'      각도 {min(th2):6.1f} ~ {max(th2):6.1f}°   압력 {min(P2):6.1f} ~ {max(P2):6.1f} kPa'
          f'   표본 {len(th2)}')
    print(f'      1차 (1/P = a + b·θ_rad):  a {a1:.6e}  b {b1:.6e}   R² {r2_1:.4f}')
    print(f'            dV/dθ = {sign*m*g:+8.2f} mL/rad   (단순 피스톤 {sign*g:+.2f})'
          f'   **배수 m = {m:.2f}**')

    if 2 in res:
        c2, r2_2, rms2 = res[2]
        a2, b2, cc2 = c2
        # 구간 양 끝에서의 국소 배수 — 비선형성을 눈으로 보여 준다
        def m_at(th_deg):
            xx = th_deg * math.pi / 180.0
            return sign * ((b2 + 2 * cc2 * xx) / a2) * V_off / g
        lo, hi = min(th2), max(th2)
        print(f'      2차:  R² {r2_2:.4f}  (RMS {rms1:.3e} → {rms2:.3e})'
              f'   국소 배수  {lo:.0f}° {m_at(lo):.2f}  /  {hi:.0f}° {m_at(hi):.2f}')
        if rms2 < rms1 * 0.7:
            print(f'      → **2 차항이 유의하다.** 부피가 각도에 1 차가 아니다 — '
                  f'국소 배수가 {m_at(lo):.2f} → {m_at(hi):.2f} 로 변한다.')
            print(f'         지금 코드는 1 차(상수 배수)만 지원한다. 상용 구간의 '
                  f'평균값 {m:.2f} 를 쓰거나, 곡선을 넣으려면 알려줄 것.')
        else:
            print(f'      → 2 차항 이득이 작다 — 1 차(상수 배수 {m:.2f})로 충분하다.')

    # 누설 점검: 같은 각도를 두 번 지날 때 압력이 얼마나 달라졌나
    mid = 0.5 * (min(th2) + max(th2))
    first = [P2[k] for k in range(len(th2) // 3) if abs(th2[k] - mid) < span * 0.05]
    last = [P2[k] for k in range(2 * len(th2) // 3, len(th2)) if abs(th2[k] - mid) < span * 0.05]
    if len(first) > 5 and len(last) > 5:
        d = sum(last) / len(last) - sum(first) / len(first)
        tag = '  ← 누설 의심, 더 빨리 움직일 것' if abs(d) > 0.05 * dP else ''
        print(f'      누설 점검: 같은 각도({mid:.0f}°) 전·후 압력 차 {d:+.2f} kPa{tag}')
    return m


def do_fit(path):
    import csv
    rows = list(csv.DictReader(open(path)))
    if not rows:
        print("빈 파일")
        return 1
    t = [float(r["t"]) for r in rows]
    print(f'\n  {path}')
    print(f'  {len(rows)} 행, {t[-1]-t[0]:.1f} s')
    # 누설 점검: 시작과 끝의 같은 각도에서 압력이 얼마나 다른가
    print()
    for ax in range(6):
        ca = f"ang{ax}"
        if ca not in rows[0]:
            continue
        th = [float(r[ca]) for r in rows]
        if max(th) - min(th) < 5.0:
            continue
        print(f'  ══ axis{ax} ══')
        for pre, off, tank, sign, lbl in (("pp", OFF_POS_MM, TANK_POS_ML, +1, "양압 P⁺"),
                                          ("pn", OFF_NEG_MM, TANK_NEG_ML, -1, "음압 P⁻")):
            col = f"{pre}{ax}"
            if col not in rows[0]:
                continue
            P = [float(r[col]) for r in rows]
            fit_axis(th, P, off, tank, sign, lbl)
        print()
    print('  배수를 config 에 넣는 곳:  Geometry.stroke_volume_mult')
    print('  (부피는 dP/dt 의 분모라 그대로 내층 이득이다 — 한 번에 하나씩 바꿀 것)')
    return 0


# ════════════════════════════════════════════════════════════════════════════
#  main
# ════════════════════════════════════════════════════════════════════════════

def key_setup():
    if not sys.stdin.isatty():
        return None
    try:
        import termios, tty
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return old
    except Exception:
        return None


def key_restore(old):
    if old is None:
        return
    try:
        import termios
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
    except Exception:
        pass


def key_get():
    if not sys.stdin.isatty():
        return None
    try:
        import select
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
    except Exception:
        pass
    return None


def main():
    ap = argparse.ArgumentParser(
        description="밸브를 닫고 팔을 움직여 챔버 부피 곡선 V(θ) 를 잰다",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("하는 법")[-1])
    ap.add_argument("--fit", metavar="CSV", help="기록 대신 CSV 를 피팅한다")
    ap.add_argument("--axes", default="0,1,2,3,4,5", help="기록할 축 (기본 전부)")
    ap.add_argument("--seconds", type=float, default=0.0, help="이 시간 뒤 자동 종료 (0=수동)")
    ap.add_argument("--out", default=None, help="CSV 경로")
    ap.add_argument("--port", default=None, help="Teensy 포트")
    ap.add_argument("--hold-rail-closed", action="store_true",
                    help="라인 릴리프도 닫는다. **이때는 펌프를 반드시 끌 것.**")
    ap.add_argument("--force", action="store_true", help="제어기가 떠 있어도 진행")
    a = ap.parse_args()

    if a.fit:
        return do_fit(a.fit)

    axes = [int(x) for x in a.axes.replace(" ", "").split(",") if x != ""]
    for name in ("can_bridge_node", "pp_controller"):
        if proc_running(name) and not a.force:
            print(f"[중단] {name} 이 돌고 있다. 같은 CAN ID 로 서로 덮어쓴다.\n"
                  f"        pkill -f \"ros2 launch\"; pkill -f can_bridge_node; "
                  f"pkill -f pp_controller")
            return 1

    tcal, cpath = load_tcal()
    missing = [c for c in axes if c not in tcal]
    if missing:
        print(f"[경고] 엔코더 ch{missing} 가 미보정이다 — 각도가 안 나온다.")

    from canlib import canlib
    ch = canlib.openChannel(channel=CAN_CHANNEL, flags=canlib.Open.CAN_FD)
    try:
        ch.setBusParams(canlib.canBITRATE_1M)
        ch.setBusParamsFd(DATA_BITRATE, FD_TSEG1, FD_TSEG2, FD_SJW)
    except Exception:
        pass
    ch.busOn()

    can = CanIO(ch, a.hold_rail_closed)
    can.send_safe()
    can.start()
    tee = Teensy(a.port)
    tee.start()

    ts = time.strftime("%Y%m%d_%H%M%S")
    out = a.out or os.path.expanduser(f"~/result/volprobe_{ts}.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    f = open(out, "w", buffering=1)
    cols = ["t"] + [f"ang{c}" for c in axes] + [f"raw{c}" for c in axes] \
        + [f"pp{c}" for c in axes] + [f"pn{c}" for c in axes]
    f.write(",".join(cols) + "\n")

    old = key_setup()
    sys.stdout.write(HIDE + "\033[2J")
    t0 = time.perf_counter()
    n = 0
    try:
        while True:
            k = key_get()
            if k and k.lower() == "q":
                break
            if a.seconds > 0 and time.perf_counter() - t0 >= a.seconds:
                break

            press, hz = can.snap()
            raws, tn = tee.snap()
            now = time.perf_counter() - t0

            ang, pp, pn = [], [], []
            for c in axes:
                cal = tcal.get(c)
                ang.append((raws[c] - cal[0]) * 90.0 / (cal[1] - cal[0])
                           if cal else float("nan"))
                rp = press.get(POS_BD[c])
                rn = press.get(NEG_BD[c])
                pp.append(kpa(rp, POS_BD[c]) if rp is not None else float("nan"))
                pn.append(kpa(rn, NEG_BD[c]) if rn is not None else float("nan"))

            if tn > 0 and press:
                f.write(",".join([f"{now:.4f}"]
                                 + [f"{v:.4f}" for v in ang]
                                 + [str(raws[c]) for c in axes]
                                 + [f"{v:.4f}" for v in pp]
                                 + [f"{v:.4f}" for v in pn]) + "\n")
                n += 1

            o = [HOME, f"{CLR} volume_probe   {now:6.1f} s   기록 {n} 행   →  {out}\n"]
            o.append(f"{CLR} 밸브: 채널 전부 폐쇄"
                     + ("  |  라인 릴리프도 폐쇄 (**펌프 꺼 둘 것**)\n"
                        if a.hold_rail_closed else "  |  라인 릴리프 개방 (안전상태)\n"))
            o.append(f"{CLR} 엔코더 {tn} 프레임   {tee.err}\n")
            o.append(f"{CLR}{'─'*72}\n")
            o.append(f"{CLR}  axis    각도°       P⁺ kPa     P⁻ kPa    수신 Hz\n")
            for i, c in enumerate(axes):
                o.append(f"{CLR}   {c}   {ang[i]:9.2f}  {pp[i]:10.2f} {pn[i]:10.2f}"
                         f"   {hz.get(POS_BD[c],0):5.0f}/{hz.get(NEG_BD[c],0):<5.0f}\n")
            o.append(f"{CLR}{'─'*72}\n")
            o.append(f"{CLR} 팔을 **천천히 끝에서 끝까지** 움직일 것 (왕복 2~3 회).\n")
            o.append(f"{CLR} P⁺ 가 각도와 **반대로**, P⁻ 가 **같이** 움직이면 제대로 잡히는 것이다.\n")
            o.append(f"{CLR} [q] 종료하고 피팅\n")
            o.append(CLREOS)
            sys.stdout.write("".join(o))
            sys.stdout.flush()
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write(SHOW)
        key_restore(old)
        can.stop.set()
        tee.stop.set()
        time.sleep(0.2)
        can.send_safe()
        f.close()
        try:
            ch.close()          # busOff() 는 부르지 않는다
        except Exception:
            pass
        print(f"\n{n} 행 기록 → {out}")

    if n > 200:
        do_fit(out)
    else:
        print("표본이 너무 적어 피팅하지 않는다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
