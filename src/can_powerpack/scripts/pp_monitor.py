#!/usr/bin/env python3
"""
pp_monitor.py  —  Powerpack terminal dashboard
  보드 1~16 압력 + 전류 + **목표압·오차**, 엔코더 위치 실시간 표시
  (목표압은 예전에 표 아래 한 줄로 있었는데 어느 보드 것인지 눈으로 맞춰야 했다.
   각 보드 행 옆으로 옮기고 오차를 같이 띄운다.)
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

# board 5~16 은 채널 gid 0~11 이다 (powerpack_config 의 channel_board_offset).
CHANNEL_BOARD_OFFSET = 5
# 20260903 부터 엔코더는 CAN 보드가 아니라 **Teensy 채널 0~5** 다.
# (board/analog 의 index = Teensy ch = 축의 actuator_idx)
NUM_AXES_SHOWN = 6


def _physical_axes_from_env():
    """논리 제어기 axis0..N을 실제 Teensy/공압 축 번호로 매핑한다."""
    try:
        axes = [int(x) for x in os.environ.get(
            'PP_PHYSICAL_AXES', '0,1,2,3,4,5').split(',') if x.strip()]
        if axes and len(set(axes)) == len(axes) and all(0 <= x < NUM_AXES_SHOWN for x in axes):
            return axes
    except ValueError:
        pass
    return list(range(NUM_AXES_SHOWN))


PHYSICAL_AXES = _physical_axes_from_env()

SEP  = '-' * 96
SEP2 = '=' * 96


# ─────────────── ROS2 Node ───────────────
class MonitorNode(Node):
    def __init__(self):
        super().__init__('pp_monitor')
        self._lock     = threading.Lock()
        self._kpa      = [101.325] * 25
        self._currents = {}
        self._refs     = [101.325] * NUM_CHANNELS
        # controller/pressure_ref_dbg 말미 6개:
        #   [rail_pos_sp, rail_neg_sp, tank, tank_low, boost g/s, eject g/s]
        self._rail_sp  = [float('nan'), float('nan')]
        self._axis_ang = []          # [(현재각, 목표각)] × 축  (control_mode 2)
        self._tank_low = False
        self._encoders = []
        # position_dbg: 축마다 8개씩 이어붙임
        # [angle, angle_ref, p_pos, p_neg, p_pid, p_ff, p_friction, vel_dps] × num_actuators
        self._pos_dbg  = None
        # 보드별·Teensy 수신율. **브리지만 아는 값**이라 토픽으로 받는다
        # (모니터가 CAN 핸들을 따로 열면 호스트 부하가 늘고 오송신 위험도 생긴다).
        self._rx_hz = []

        ns = NAMESPACE
        self.create_subscription(Float64MultiArray, f'{ns}/controller/sensors_kpa',
                                 self._cb_kpa,      10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/currents',
                                 self._cb_currents,  10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/mpc_refs_kpa',
                                 self._cb_refs,      10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/pressure_ref_dbg',
                                 self._cb_refgen,    10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/analog',
                                 self._cb_analog,    10)
        self.create_subscription(Float64MultiArray, f'{ns}/controller/position_dbg',
                                 self._cb_pos_dbg,   10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/rx_hz',
                                 self._cb_rx_hz,     10)

    def _cb_kpa(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                if i < len(self._kpa): self._kpa[i] = v

    def _cb_currents(self, msg):
        with self._lock:
            for i in range(len(msg.data) // 3):
                self._currents[i+1] = [msg.data[i*3+j] / 10.0 for j in range(3)]

    def _cb_refgen(self, msg):
        # 축마다 12 개 + 말미 공용 6 개. 축 수를 몰라도 말미는 뒤에서 셀 수 있다.
        d = list(msg.data)
        if len(d) < 6:
            return
        # 축마다 12 개 + 말미 6 개.  축 블록: [angle, angle_ref, tau_ref, tau_ach, ...]
        n_ax = max(0, (len(d) - 6) // 12)
        ax = [(d[12 * a], d[12 * a + 1]) for a in range(n_ax)]
        with self._lock:
            self._rail_sp = [d[-6], d[-5]]
            self._tank_low = d[-3] > 0.5
            self._axis_ang = ax

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

    def _cb_rx_hz(self, msg):
        with self._lock:
            self._rx_hz = list(msg.data)

    def snapshot(self):
        with self._lock:
            return (list(self._kpa), dict(self._currents), list(self._refs),
                    list(self._encoders), self._pos_dbg,
                    list(self._rail_sp), self._tank_low, list(self._axis_ang),
                    list(self._rx_hz))


# ─────────────── 화면 구성 ───────────────
def build_display(kpa, currents, refs, encoders, pos_dbg, rail_sp, tank_low,
                  axis_ang, rx_hz, display_on):
    if not display_on:
        return '\033[2J\033[H  [ Display OFF ]  press \'t\' to enable\n'

    o = '\033[H'
    o += f'========== Powerpack Monitor ({NAMESPACE}) ==========\n'
    o += f' [t] toggle display   [q] quit\n'
    o += ' 제어 매핑: ' + ', '.join(
        f'logical axis{i} -> physical axis{p}' for i, p in enumerate(PHYSICAL_AXES)) + '\n'
    # ── 수신율 요약 ─────────────────────────────────────────────────────
    # 통신이 죽으면 아래 압력·각도는 **얼어붙은 값**이다. 그걸 모르고 보면
    # 값이 멀쩡해 보인다 — 그래서 맨 위에 둔다.
    if rx_hz and len(rx_hz) >= 17:
        bd = [h for h in rx_hz[:16]]
        alive = [h for h in bd if h > 1.0]
        enc = rx_hz[16]
        lo = min(alive) if alive else 0.0
        lob = (bd.index(lo) + 1) if alive else 0
        warn = ''
        if len(alive) < 16:
            warn = f"   ** 보드 {16-len(alive)}개 두절 **"
        elif lo < 150:
            warn = f'   ** board{lob} 이 {lo:.0f} Hz 로 굶는다 **'
        if enc < 190:
            warn += f'   ** 엔코더 {enc:.0f} Hz **'
        o += (f' CAN 보드 {len(alive)}/16 수신  평균 {sum(alive)/max(1,len(alive)):.0f} Hz'
              f'  최저 {lo:.0f} Hz (board{lob})   |   엔코더 {enc:.0f} Hz{warn}\n')
    else:
        o += ' 수신율: board/rx_hz 대기 중 (브리지가 안 떠 있거나 구버전)\n'
    o += SEP + '\n'
    o += ('|  ID  | Name          |  I1(mA) |  I2(mA) |  I3(mA) |'
          ' Press(kPa) |   Ref(kPa) |  Err(kPa) | Rx(Hz) |\n')
    o += ('|------|---------------|---------|---------|---------|'
          '------------|------------|-----------|--------|\n')

    for bid in range(1, 17):
        p    = kpa[bid-1] if bid-1 < len(kpa) else 0.0
        cur  = currents.get(bid, [0.0, 0.0, 0.0])
        name = BOARD_NAMES.get(bid, f'board{bid}')

        # 목표압: 채널 보드는 그 채널의 레퍼런스, 라인 보드는 레일 셋포인트.
        # 탱크(3)·이젝터(4)는 추종 목표가 없다.
        gid = bid - CHANNEL_BOARD_OFFSET
        if 0 <= gid < NUM_CHANNELS and gid < len(refs):
            ref = refs[gid]
        elif bid == 1:
            ref = rail_sp[0] if rail_sp else float('nan')
        elif bid == 2:
            ref = rail_sp[1] if len(rail_sp) > 1 else float('nan')
        else:
            ref = float('nan')

        if ref == ref:                       # NaN 이 아니면
            ref_s, err_s = f'{ref:10.3f}', f'{p - ref:+9.2f}'
        else:
            ref_s, err_s = ' ' * 10, ' ' * 9

        if rx_hz and bid - 1 < len(rx_hz):
            h = rx_hz[bid - 1]
            hz_s = f'{h:6.0f}' if h > 1.0 else '  두절'
        else:
            hz_s = '     -'
        o += (f'|  {bid:02d}  | {name:<13} |'
              f' {cur[0]:7.1f} | {cur[1]:7.1f} | {cur[2]:7.1f} |'
              f' {p:10.3f} | {ref_s} | {err_s} | {hz_s} |\n')

    if tank_low:
        o += '  [경고] 탱크 압력이 운전 하한 미만이다 — macro 부스트를 쓸 수 없다\n'

    # ── 각도: 보드 하나당 한 행 ─────────────────────────────────────────
    #
    # 예전에는 'Encoders'(board/analog 9 개)와 'Position Control'(축별)이 따로 있어
    # 같은 각도가 두 곳에 나왔다 — 화면에 12 개가 찍혔다. 한 표로 합친다.
    # 목표각은 control_mode 2 면 pressure_ref_dbg, 1 이면 position_dbg 에서 온다.
    o += SEP + '\n'
    enc_hz = rx_hz[16] if (rx_hz and len(rx_hz) >= 17) else None
    hz_note = f'   [Teensy {enc_hz:.0f} Hz]' if enc_hz is not None else ''
    o += f'  엔코더 (Teensy USB){hz_note}\n'
    o += '| Phys | Ctrl |  Angle(deg) | Target(deg) |   Err(deg) |\n'
    o += '|------|------|-------------|-------------|------------|\n'

    tgt = {}
    for logical, (_ang, ref) in enumerate(axis_ang or []):
        if logical < len(PHYSICAL_AXES):
            tgt[PHYSICAL_AXES[logical]] = ref
    if not tgt and pos_dbg and len(pos_dbg) >= 8:
        for logical in range(min(len(pos_dbg) // 8, len(PHYSICAL_AXES))):
            tgt[PHYSICAL_AXES[logical]] = pos_dbg[8 * logical + 1]

    logical_for_physical = {p: i for i, p in enumerate(PHYSICAL_AXES)}
    for a in range(NUM_AXES_SHOWN):
        now = encoders[a] if a < len(encoders) else float('nan')
        ref = tgt.get(a, float('nan'))
        now_s = f'{now:11.2f}' if now == now else ' ' * 11
        if ref == ref and now == now:
            ref_s, err_s = f'{ref:11.2f}', f'{ref - now:+10.2f}'
        elif ref == ref:
            ref_s, err_s = f'{ref:11.2f}', ' ' * 10
        else:
            ref_s, err_s = ' ' * 11, ' ' * 10
        ctrl_s = f'L{logical_for_physical[a]}' if a in logical_for_physical else '-'
        o += f'|  {a:>2}  | {ctrl_s:^4} | {now_s} | {ref_s} | {err_s} |\n'

    # 위치 제어 세부 (mode 1 에서만 나온다)
    if pos_dbg and len(pos_dbg) >= 8:
        o += SEP + '\n'
        for logical in range(min(len(pos_dbg) // 8, len(PHYSICAL_AXES))):
            physical = PHYSICAL_AXES[logical]
            b = 8 * logical
            _ang, _ref, p_pos, p_neg, p_pid, p_ff, p_fric, vel = pos_dbg[b:b + 8]
            o += (f' [L{logical}->physical {physical}] vel={vel:+6.1f} dps   '
                  f'P+={p_pos:6.1f}  P-={p_neg:6.1f} kPa'
                  f'   (pid={p_pid:+5.1f} ff={p_ff:+5.1f} fric={p_fric:+5.1f})\n')

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

            (kpa, currents, refs, encoders, pos_dbg,
             rail_sp, tank_low, axis_ang, rx_hz) = node.snapshot()
            sys.stdout.write(build_display(kpa, currents, refs, encoders, pos_dbg,
                                           rail_sp, tank_low, axis_ang, rx_hz,
                                           display_on))
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
