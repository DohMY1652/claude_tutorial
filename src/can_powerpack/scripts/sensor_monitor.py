#!/usr/bin/env python3
"""센서 통합 모니터 — CAN 압력·전류 + Teensy 엔코더 + **수신 주파수**를 한 화면에.

can_monitor.py 는 엔코더까지 CAN 에서 읽던 시절 것이다. 20260912 에 엔코더가
Teensy USB CDC 로 넘어가면서 소스가 둘로 갈렸는데, 둘을 따로 띄우면
"압력은 멀쩡한데 각도가 얼어붙은" 상태를 한눈에 못 본다. 그래서 합쳤다.

**주파수를 같이 보여 주는 이유가 핵심이다.** 통신이 죽으면 화면의 압력·각도는
마지막 값에 얼어붙는데, 숫자만 보면 멀쩡해 보인다. Hz 가 그걸 구별하는
유일한 표시다. 제어 루프가 200 Hz 라 그 아래로 떨어지는 보드는 **stale 데이터**를
컨트롤러에 먹이고 있는 것이다.

두 가지 모드로 돈다 (기본 auto)
--------------------------------
  ros    : 브리지(can_bridge_node)가 떠 있으면 **토픽으로만** 읽는다.
  direct : 브리지가 없으면 CAN 핸들과 시리얼 포트를 직접 연다.

auto 가 브리지 프로세스를 보고 알아서 고른다. 이게 중요한 안전장치다 —
**/dev/ttyACM* 를 두 프로세스가 열면 바이트가 무작위로 쪼개져 양쪽 다 깨진다.**
브리지가 도는 중에 direct 로 열면 실험 중인 엔코더를 망가뜨린다.

사용
----
    python3 sensor_monitor.py              # 알아서 고른다
    python3 sensor_monitor.py --source direct   # 브리지 없이 벤치 점검
"""

from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import threading
import time
from collections import deque

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

NAMESPACE = "/pack2"
NUM_BOARDS = 16          # CAN 압력·전류 보드
NCH = 6                  # Teensy 엔코더 채널
CTRL_HZ = 200.0          # 제어 루프 주기 — 이 아래면 stale 데이터다
ENC_HZ_NOM = 200.0

SEP = "=" * 86
RULE = ("|------|---------------|---------|---------|---------|"
        "------------|--------|-------|")

BOARD_NAMES = {
    1: "P_line_pos", 2: "P_line_neg", 3: "P_macro_pos", 4: "P_macro_neg",
    **{5 + i: f"pos ch{i}" for i in range(6)},
    **{11 + i: f"neg ch{i}" for i in range(6)},
}


# ════════════════════════════════════════════════════════════════════════════
#  공유 상태 — 어느 모드든 여기에 채운다
# ════════════════════════════════════════════════════════════════════════════
class State:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.kpa = [float("nan")] * (NUM_BOARDS + 1)     # [bid]
        self.cur = {b: [0.0, 0.0, 0.0] for b in range(1, NUM_BOARDS + 1)}
        self.hz = [0.0] * (NUM_BOARDS + 1)               # [bid]
        self.ang = [float("nan")] * NCH
        self.raw = [0] * NCH
        self.enc_hz = 0.0
        self.enc_lost = 0
        self.enc_crc = 0
        self.enc_status = 0
        self.mode = "?"
        self.note = ""
        self.port = ""


def bridge_running() -> bool:
    """can_bridge_node 가 도는지. 시리얼·CAN 충돌을 피하는 유일한 판단 근거다."""
    try:
        out = subprocess.run(["pgrep", "-f", "can_bridge_node"],
                             capture_output=True, text=True, timeout=2.0)
        return out.returncode == 0 and bool(out.stdout.strip())
    except Exception:
        return False


# ════════════════════════════════════════════════════════════════════════════
#  ROS 모드 — 브리지가 이미 다 읽고 있으니 토픽만 받는다
# ════════════════════════════════════════════════════════════════════════════
def run_ros(st: State, args) -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray, UInt16MultiArray

    cfg = _load_calib()

    class Sub(Node):
        def __init__(self) -> None:
            super().__init__("sensor_monitor")
            ns = NAMESPACE
            self.create_subscription(UInt16MultiArray, f"{ns}/board/sensors",
                                     self._sens, 10)
            self.create_subscription(Float64MultiArray, f"{ns}/board/currents",
                                     self._cur, 10)
            self.create_subscription(Float64MultiArray, f"{ns}/board/analog",
                                     self._ang, 10)
            self.create_subscription(UInt16MultiArray, f"{ns}/board/analog_raw",
                                     self._raw, 10)
            self.create_subscription(Float64MultiArray, f"{ns}/board/rx_hz",
                                     self._hz, 10)

        def _sens(self, m):
            with st.lock:
                for i, v in enumerate(m.data[:NUM_BOARDS]):
                    bid = i + 1
                    st.kpa[bid] = _kpa(v, bid, cfg)

        def _cur(self, m):
            with st.lock:
                for b in range(1, NUM_BOARDS + 1):
                    k = (b - 1) * 3
                    if k + 2 < len(m.data):
                        st.cur[b] = list(m.data[k:k + 3])

        def _ang(self, m):
            with st.lock:
                for i, v in enumerate(m.data[:NCH]):
                    st.ang[i] = v

        def _raw(self, m):
            with st.lock:
                for i, v in enumerate(m.data[:NCH]):
                    st.raw[i] = int(v)

        def _hz(self, m):
            with st.lock:
                for i, v in enumerate(m.data[:NUM_BOARDS]):
                    st.hz[i + 1] = v
                if len(m.data) > NUM_BOARDS:
                    st.enc_hz = m.data[NUM_BOARDS]

    rclpy.init()
    node = Sub()
    st.mode = "ros"
    st.note = "브리지가 떠 있어 토픽으로 읽는다 (하드웨어를 직접 열지 않는다)"
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            _draw(st, args)
            time.sleep(max(0.0, 1.0 / args.hz - 0.05))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _load_calib():
    """압력 보정(offset/gain)을 yaml 에서 읽는다 — can_monitor 와 같은 출처."""
    try:
        import yaml
        p = os.path.join(_HERE, "..", "config", "powerpack_config.yaml")
        with open(p, encoding="utf-8") as fh:
            sc = yaml.safe_load(fh)["/pack2/pp_controller"]["ros__parameters"]["Sensor_calibration"]
        return ({int(k): float(v["offset"]) for k, v in sc["boards"].items()},
                {int(k): float(v["gain"]) for k, v in sc["boards"].items()},
                float(sc.get("atm_offset", 101.325)))
    except Exception:
        return ({}, {}, 101.325)


def _kpa(raw, bid, cfg):
    offs, gains, atm = cfg
    if raw == 0 or bid not in offs:
        return float("nan")
    return (float(raw) - offs[bid]) * gains[bid] + atm


# ════════════════════════════════════════════════════════════════════════════
#  direct 모드 — CAN 핸들과 시리얼을 직접 연다 (브리지가 없을 때만)
# ════════════════════════════════════════════════════════════════════════════
def run_direct(st: State, args) -> None:
    st.mode = "direct"
    st.note = "브리지가 없어 CAN·시리얼을 직접 연다"
    cfg = _load_calib()
    stop = threading.Event()
    cnt = [0] * (NUM_BOARDS + 1)
    enc_cnt = [0]

    th_can = threading.Thread(target=_can_thread, args=(st, cfg, cnt, stop), daemon=True)
    th_ser = threading.Thread(target=_serial_thread, args=(st, enc_cnt, stop, args), daemon=True)
    th_can.start()
    th_ser.start()

    prev = list(cnt)
    prev_e = 0
    t_prev = time.monotonic()
    try:
        while True:
            time.sleep(1.0 / args.hz)
            now = time.monotonic()
            if now - t_prev >= 1.0:
                span = now - t_prev
                with st.lock:
                    for b in range(1, NUM_BOARDS + 1):
                        st.hz[b] = (cnt[b] - prev[b]) / span
                    st.enc_hz = (enc_cnt[0] - prev_e) / span
                prev = list(cnt)
                prev_e = enc_cnt[0]
                t_prev = now
            _draw(st, args)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        time.sleep(0.2)


def _can_thread(st: State, cfg, cnt, stop) -> None:
    try:
        from canlib import canlib
    except Exception as exc:
        with st.lock:
            st.note = f"canlib 없음: {exc}"
        return
    try:
        ch = canlib.openChannel(channel=0, flags=canlib.Open.CAN_FD)
        try:
            ch.busOn()
        except Exception:
            pass
    except Exception as exc:
        with st.lock:
            st.note = f"CAN 열기 실패: {exc}"
        return
    TO_MV = 3300.0 / 4095.0
    while not stop.is_set():
        try:
            msg = ch.read(timeout=50)
        except Exception:
            continue
        if not (0x121 <= msg.id <= 0x130) or len(msg.data) < 8:
            continue
        bid = msg.id - 0x120
        cnt[bid] += 1
        raw = struct.unpack("<HHHH", bytes(msg.data[:8]))
        with st.lock:
            st.cur[bid] = [raw[i] * TO_MV / 10.0 for i in range(3)]
            st.kpa[bid] = _kpa(raw[3], bid, cfg)
    try:
        ch.busOff(); ch.close()
    except Exception:
        pass


def _serial_thread(st: State, enc_cnt, stop, args) -> None:
    import teensy_monitor as tm
    port = args.port or tm.find_port()
    if not port:
        with st.lock:
            st.note += " | Teensy 포트를 못 찾았다"
        return
    st.port = port
    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as exc:
        with st.lock:
            st.note += f" | 시리얼 열기 실패: {exc}"
        return
    import termios
    tio = termios.tcgetattr(fd)
    tio[0] = tio[1] = tio[3] = 0
    tio[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    tio[6][termios.VMIN] = 0
    tio[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, tio)
    os.write(fd, b"r")                      # 'r' 을 보내야 스트리밍이 시작된다
    buf = b""
    prev_seq = None
    try:
        while not stop.is_set():
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.002); continue
            except OSError:
                break
            if chunk:
                buf += chunk
                if len(buf) > 8192:
                    buf = buf[-4096:]
            i = 0
            while i + tm.FRAME_LEN <= len(buf):
                if buf[i:i+2] != tm.SYNC:
                    i += 1; continue
                fr = buf[i:i+tm.FRAME_LEN]
                if tm.crc16(fr[:22]) != struct.unpack("<H", fr[22:24])[0]:
                    with st.lock:
                        st.enc_crc += 1
                    i += 2; continue
                seq = struct.unpack("<H", fr[2:4])[0]
                chs = struct.unpack("<6h", fr[8:20])
                status = struct.unpack("<H", fr[20:22])[0]
                enc_cnt[0] += 1
                with st.lock:
                    st.raw = list(chs)
                    st.enc_status = status
                    if prev_seq is not None:
                        st.enc_lost += (seq - prev_seq - 1) & 0xFFFF
                prev_seq = seq
                i += tm.FRAME_LEN
            buf = buf[i:]
    finally:
        try:
            os.write(fd, b"x"); os.close(fd)
        except Exception:
            pass


# ════════════════════════════════════════════════════════════════════════════
#  화면
# ════════════════════════════════════════════════════════════════════════════
def _draw(st: State, args) -> None:
    with st.lock:
        kpa = list(st.kpa); cur = {k: list(v) for k, v in st.cur.items()}
        hz = list(st.hz); ang = list(st.ang); raw = list(st.raw)
        ehz, elost, ecrc, estat = st.enc_hz, st.enc_lost, st.enc_crc, st.enc_status
        mode, note, port = st.mode, st.note, st.port

    o = "\033[H"
    o += f"========== Sensor Monitor ({NAMESPACE})  [{mode}] ==========\n"
    o += f" {note}\n"

    # ── 수신율 요약 — 얼어붙은 값을 구별하는 유일한 표시다 ──────────────────
    alive = [h for h in hz[1:NUM_BOARDS + 1] if h > 1.0]
    lo = min(alive) if alive else 0.0
    lob = hz.index(lo) if alive else 0
    warn = ""
    if len(alive) < NUM_BOARDS:
        warn = f"   ** 보드 {NUM_BOARDS - len(alive)}개 두절 **"
    elif lo < CTRL_HZ:
        warn = f"   ** board{lob} 이 {lo:.0f} Hz — 제어 {CTRL_HZ:.0f} Hz 보다 느리다 **"
    if ehz < ENC_HZ_NOM * 0.95:
        warn += f"   ** 엔코더 {ehz:.0f} Hz **"
    o += (f" CAN {len(alive)}/{NUM_BOARDS} 수신  평균 {sum(alive)/max(1,len(alive)):.0f} Hz"
          f"  최저 {lo:.0f} Hz (board{lob})   |   Teensy {ehz:.0f} Hz{warn}\n")
    o += "-" * 86 + "\n"

    o += ("|  ID  | Name          |  I1(mA) |  I2(mA) |  I3(mA) |"
          " Press(kPa) | Rx(Hz) | State |\n")
    o += RULE + "\n"
    for bid in range(1, NUM_BOARDS + 1):
        c = cur.get(bid, [0.0, 0.0, 0.0])
        p = kpa[bid]
        p_s = f"{p:10.3f}" if p == p else "         -"
        h = hz[bid]
        h_s = f"{h:6.0f}" if h > 1.0 else "  두절"
        stt = "OK" if h > CTRL_HZ else ("느림" if h > 1.0 else "Lost")
        o += (f"|  {bid:02d}  | {BOARD_NAMES.get(bid, ''):<13} |"
              f" {c[0]:7.1f} | {c[1]:7.1f} | {c[2]:7.1f} |"
              f" {p_s} | {h_s} | {stt:^5} |\n")
        # 보드 묶음 경계. 4|5 = 라인·탱크 ↔ 양압, 10|11 = 양압 ↔ 음압.
        if bid in (4, 10):
            o += RULE + "\n"

    o += SEP + "\n"
    o += (f" 엔코더 (Teensy USB {port})   {ehz:.0f} Hz   "
          f"유실 {elost}   CRC오류 {ecrc}   status 0x{estat:04X}\n")
    if estat:
        o += "   ** status != 0 → ADS1115 I2C 오류다 (비트 = 칩 번호). 그 채널 값은 못 믿는다 **\n"
    o += "|  ch  |    raw   |   Angle(deg)  |\n"
    o += "|------|----------|---------------|\n"
    for c in range(NCH):
        a = ang[c]
        a_s = f"{a:10.2f}" if a == a else "         -"
        o += f"|   {c}  | {raw[c]:8d} | {a_s}    |\n"
    o += SEP + "\n"
    o += " raw 가 **전혀 안 떨리면** 그 채널은 죽은 것이다 (raw 0 도 각도로는 121~129° 로 보인다)\n"
    o += " Ctrl+C 로 종료\n"
    sys.stdout.write(o)
    sys.stdout.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description="CAN + Teensy 통합 센서 모니터")
    ap.add_argument("--source", choices=("auto", "ros", "direct"), default="auto",
                    help="auto = 브리지가 떠 있으면 ros, 없으면 direct")
    ap.add_argument("--port", default=None, help="Teensy 포트 (direct 모드, 비우면 자동탐색)")
    ap.add_argument("--hz", type=float, default=10.0, help="화면 갱신율")
    args = ap.parse_args()

    src = args.source
    running = bridge_running()
    if src == "auto":
        src = "ros" if running else "direct"
    if src == "direct" and running:
        print("[중단] can_bridge_node 가 돌고 있다. direct 모드로 시리얼을 열면\n"
              "       **바이트가 쪼개져 브리지의 엔코더까지 깨진다.**\n"
              "       --source ros 로 쓰거나 브리지를 먼저 내릴 것.", file=sys.stderr)
        return 2

    os.system("clear")
    st = State()
    if src == "ros":
        run_ros(st, args)
    else:
        run_direct(st, args)
    print("\n종료.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
