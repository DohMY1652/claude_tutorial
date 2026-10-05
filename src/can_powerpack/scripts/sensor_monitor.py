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
        # direct 모드에서 raw → 도 환산에 쓸 2점 보정. [ch] = (raw0, deg/count)
        # 또는 None(미보정). ros 모드에서는 브리지가 이미 환산해 주므로 안 쓴다.
        self.enc_cal = [None] * NCH


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
            # board/currents 는 **mV** 로 온다 (CanBridge: raw × 3300/4095).
            # mA = mV / 10 이다 (pp_logger 와 같은 환산, 아래 직접 CAN 경로와도 같다).
            # 예전에는 이 나눗셈이 빠져 mV 를 그대로 "I(mA)" 열에 찍었다 — 10 배로
            # 부풀어 2000 mA 처럼 보였다. 실제 코일 상한은 I_MAX 250.5 mA 다.
            with st.lock:
                for b in range(1, NUM_BOARDS + 1):
                    k = (b - 1) * 3
                    if k + 2 < len(m.data):
                        st.cur[b] = [v / 10.0 for v in m.data[k:k + 3]]

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


def _load_enc_calib():
    """Teensy 엔코더 2점 보정을 yaml 에서 읽는다 — **브리지와 같은 출처**여야 한다.

    환산식은 CanBridge.cpp 와 같다:  deg = (raw − raw_0deg) × 90 / (raw_90deg − raw_0deg)
    보정이 없거나 두 점이 같은 채널은 None 으로 둔다. 브리지는 그럴 때 각도를
    0° 로 **고정**하는데, 모니터에서 그러면 "0° 에 있다" 는 거짓말이 되므로
    여기서는 '미보정' 이라고 적는다.
    """
    cal = [None] * NCH
    try:
        import yaml
        p = os.path.join(_HERE, "..", "config", "powerpack_config.yaml")
        with open(p, encoding="utf-8") as fh:
            ch = (yaml.safe_load(fh)["/pack2/can_bridge"]["ros__parameters"]
                  ["TeensyEncoder"]["channels"])
        for c in range(NCH):
            e = ch.get(str(c)) or ch.get(c)
            if not e:
                continue
            r0, r90 = float(e["raw_0deg"]), float(e["raw_90deg"])
            if abs(r90 - r0) > 1e-6:
                cal[c] = (r0, 90.0 / (r90 - r0))
    except Exception:
        pass
    return cal


def _enc_deg(raw: int, cal) -> float:
    if cal is None:
        return float("nan")
    r0, scale = cal
    return (float(raw) - r0) * scale


def _p_mv(counts: int) -> float:
    """압력 ADC counts → 원신호 mV. CanBridge.cpp 의 p_mv_raw 와 같은 식이어야 한다.
    센서 1~5 V 가 반전 증폭으로 들어오므로 **빼는** 식이다 (곱이 아니다)."""
    return min(5000.0, max(0.0, 5000.0 - counts * 4000.0 / 4095.0))


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
    st.enc_cal = _load_enc_calib()
    if all(c is None for c in st.enc_cal):
        st.note += " | 엔코더 보정을 못 읽었다 — 각도 열이 빈다"
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
            # raw → mV(×TO_MV) → mA(÷10). 위 토픽 경로와 같은 단위여야 한다.
            st.cur[bid] = [raw[i] * TO_MV / 10.0 for i in range(3)]
            # 압력은 **반전 증폭**이라 counts 를 그대로 쓰면 안 된다.
            # CanBridge 와 같은 식으로 원신호 mV 를 복원한 뒤 _kpa 에 넣는다:
            #     p_mv = 5000 − counts·4000/4095          (CanBridge.cpp)
            # 예전에는 counts 를 mV 자리에 그대로 넣어 800 kPa 같은 값이 나왔다.
            # board/sensors 토픽은 브리지가 이미 mV 로 바꿔 실어 주므로 토픽
            # 경로(_sens)는 원래 맞았다 — 두 경로가 이제 같은 값을 낸다.
            st.kpa[bid] = _kpa(_p_mv(raw[3]), bid, cfg)
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
                    st.ang = [_enc_deg(r, st.enc_cal[k]) for k, r in enumerate(chs)]
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
        cal = list(st.enc_cal)
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
    warn_low = False
    for c in range(NCH):
        a = ang[c]
        a_s = f"{a:10.2f}" if a == a else "     미보정"
        # 기동 직후 Teensy 스트리밍 전에는 raw 가 1000 밑으로 떨어지는데, 그 값이
        # 각도로는 121~129° 라는 **그럴듯한 숫자**로 나온다. 죽은 센서와 구별이
        # 안 되므로 표에서 바로 보이게 한다.
        low = cal[c] is not None and abs(raw[c]) < 1000
        warn_low |= low
        o += f"|   {c}  | {raw[c]:8d} | {a_s} {'!' if low else ' '}  |\n"
    o += SEP + "\n"
    # 어느 보정으로 환산했는지 — 개체를 갈아 끼우면 여기가 바뀌어야 한다
    # (docs/액추에이터_개체_대장.md). ros 모드에서는 브리지가 환산해 주므로,
    # 브리지가 **먼저 뜬 뒤** yaml 을 고쳤다면 이 줄과 실제 각도가 어긋난다.
    if any(c is not None for c in cal):
        items = [f"ch{c}={r0:.0f}/{r0 + 90.0 / sc:.0f}"
                 for c, v in enumerate(cal) if v for r0, sc in [v]]
        o += " 2점 보정 (raw@0°/raw@90°, yaml 에서 읽음)\n"
        for k in range(0, len(items), 3):          # 한 줄에 3 채널 — SEP 폭 안에 든다
            o += "   " + "   ".join(items[k:k + 3]) + "\n"
    if warn_low:
        o += (" ** ! = raw 가 1000 미만이다. 각도로는 121~129° 라는 그럴듯한 값이 나오지만\n"
              "      기동 직후(약 4초, Teensy 스트리밍 전)이거나 센서가 죽은 것이다 **\n")
    o += " raw 가 **전혀 안 떨리면** 그 채널은 죽은 것이다 (raw 0 도 각도로는 121~129° 로 보인다)\n"
    o += " Ctrl+C 로 종료\n"
    # `\033[H` 는 커서를 홈으로 보낼 뿐 **지우지 않는다.** 그래서 이전 프레임보다
    # 짧아진 줄은 뒷부분이 그대로 남는다. 조건부로 나타났다 사라지는 줄(경고,
    # 보정 목록)이 생기면서 실제로 잔상이 보였다 — 각 줄 끝에서 줄의 나머지를,
    # 마지막에 화면의 나머지를 지운다. 이러면 줄 길이에 신경 쓸 필요가 없다.
    o = o.replace("\n", "\033[K\n") + "\033[J"
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
