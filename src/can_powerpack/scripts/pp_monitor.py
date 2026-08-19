#!/usr/bin/env python3
"""
pp_monitor.py  —  Powerpack terminal dashboard
  보드 1~16 압력 + 전류, 목표 압력, 엔코더 위치 실시간 표시
  't' 키: 표시 on/off 토글   'q' / Ctrl-C: 종료
"""
import sys, os, threading, termios, tty, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

NAMESPACE    = '/pack2'
NUM_CHANNELS = 12

BOARD_NAMES = {
    1:'P_line_pos',  2:'P_line_neg',
    3:'P_macro_pos', 4:'P_macro_neg',
    5:'pos ch0',  6:'pos ch1',  7:'pos ch2',  8:'pos ch3',
    9:'pos ch4', 10:'pos ch5', 11:'neg ch6', 12:'neg ch7',
   13:'neg ch8', 14:'neg ch9', 15:'neg ch10',16:'neg ch11',
}

SEP  = '------------------------------------------------------------------------'
SEP2 = '========================================================================'


# ─────────────── ROS2 Node ───────────────
class MonitorNode(Node):
    def __init__(self):
        super().__init__('pp_monitor')
        self._lock     = threading.Lock()
        self._kpa      = [101.325] * 25
        self._currents = {}
        self._refs     = [101.325] * NUM_CHANNELS
        self._encoders = []
        # position_dbg: 축마다 8개씩 이어붙임
        # [angle, angle_ref, p_pos, p_neg, p_pid, p_ff, p_friction, vel_dps] × num_actuators
        self._pos_dbg  = None

        ns = NAMESPACE
        self.create_subscription(Float64MultiArray, f'{ns}/controller/sensors_kpa',
                                 self._cb_kpa,      10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/currents',
                                 self._cb_currents,  10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/mpc_refs_kpa',
                                 self._cb_refs,      10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/analog',
                                 self._cb_analog,    10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/position_dbg',
                                 self._cb_pos_dbg,   10)

    def _cb_kpa(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < len(self._kpa): self._kpa[i] = v

    def _cb_currents(self, msg):
        with self._lock:
            for i in range(len(msg.data) // 3):
                self._currents[i+1] = [msg.data[i*3+j] / 10.0 for j in range(3)]

    def _cb_refs(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < NUM_CHANNELS: self._refs[i] = v

    def _cb_analog(self, msg):
        with self._lock:
            self._encoders = list(msg.data)

    def _cb_pos_dbg(self, msg):
        with self._lock:
            self._pos_dbg = list(msg.data)

    def snapshot(self):
        with self._lock:
            return list(self._kpa), dict(self._currents), list(self._refs), list(self._encoders), self._pos_dbg


# ─────────────── 화면 구성 ───────────────
def build_display(kpa, currents, refs, encoders, pos_dbg, display_on):
    if not display_on:
        return '\033[2J\033[H  [ Display OFF ]  press \'t\' to enable\n'

    o = '\033[H'
    o += f'========== Powerpack Monitor ({NAMESPACE}) ==========\n'
    o += f' [t] toggle display   [q] quit\n'
    o += SEP + '\n'
    o += '|  ID  | Name          |  I1(mA) |  I2(mA) |  I3(mA) | Press(kPa) |\n'
    o += '|------|---------------|---------|---------|---------|------------|\n'

    for bid in range(1, 17):
        p    = kpa[bid-1] if bid-1 < len(kpa) else 0.0
        cur  = currents.get(bid, [0.0, 0.0, 0.0])
        name = BOARD_NAMES.get(bid, f'board{bid}')
        o += (f'|  {bid:02d}  | {name:<13} |'
              f' {cur[0]:7.1f} | {cur[1]:7.1f} | {cur[2]:7.1f} |'
              f' {p:10.3f} |\n')

    o += SEP2 + '\n'
    o += ' Target Pressures\n'
    o += '  gid: ' + ' '.join(f'{i:4d}' for i in range(NUM_CHANNELS)) + '\n'
    o += '  ref: ' + ' '.join(f'{v:4.0f}' for v in refs)              + '\n'

    o += SEP + '\n'
    o += ' Encoders\n'
    if encoders:
        for start in range(0, len(encoders), 3):
            chunk = encoders[start:start+3]
            parts = [f'Bd{17+start+i}: {deg:6.1f} deg' for i, deg in enumerate(chunk)]
            o += '  ' + '   '.join(parts) + '\n'
    else:
        o += '  (no encoder data)\n'

    # ── 위치 제어 상태 (position_dbg 수신 시만 표시) ──
    # pos_dbg: 축마다 8개씩 이어붙임 [angle, angle_ref, p_pos, p_neg, p_pid, p_ff, p_friction, vel_dps] × N
    o += SEP + '\n'
    o += ' Position Control\n'
    if pos_dbg and len(pos_dbg) >= 8:
        for start in range(0, len(pos_dbg) - len(pos_dbg) % 8, 8):
            axis = start // 8
            angle, angle_ref, p_pos, p_neg, p_pid, p_ff, p_fric, vel = pos_dbg[start:start+8]
            err = angle_ref - angle
            o += f' [Axis {axis}]\n'
            o += (f'  Angle : now={angle:6.2f} deg   target={angle_ref:6.2f} deg'
                  f'   err={err:+6.2f} deg   vel={vel:+6.1f} dps\n')
            o += (f'  Output: P+={p_pos:6.1f} kPa   P-={p_neg:6.1f} kPa'
                  f'   (pid={p_pid:+5.1f}  ff={p_ff:+5.1f}  fric={p_fric:+5.1f})\n')
    else:
        o += '  (위치 제어 비활성 또는 데이터 없음 — control_mode=0 이거나 TCP 미수신)\n'

    o += SEP2 + '\n'
    return o


# ─────────────── 키 입력 ───────────────
def key_listener(toggle_event, stop_event):
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            ch = sys.stdin.read(1)
            if ch in ('t', 'T'):
                toggle_event.set()
            elif ch in ('q', 'Q'):
                stop_event.set()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ─────────────── Main ───────────────
def main():
    rclpy.init()
    node = MonitorNode()

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    toggle_event = threading.Event()
    stop_event   = threading.Event()
    threading.Thread(target=key_listener, args=(toggle_event, stop_event), daemon=True).start()

    display_on = True
    os.system('clear')

    try:
        while not stop_event.is_set():
            if toggle_event.is_set():
                display_on = not display_on
                toggle_event.clear()
                if display_on:
                    os.system('clear')

            kpa, currents, refs, encoders, pos_dbg = node.snapshot()
            sys.stdout.write(build_display(kpa, currents, refs, encoders, pos_dbg, display_on))
            sys.stdout.flush()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.system('clear')
        print('종료.')


if __name__ == '__main__':
    main()
