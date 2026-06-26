#!/usr/bin/env python3
"""
Powerpack live controller.
  - 상단: 활성 보드 압력/전류/엔코더 실시간 표시
  - 하단: PWM 명령 입력

Commands:
  <board 1-16> <valve 1-3> <pwm 0-4095>   예) 5 1 2048
  clear                                    전체 PWM 0
  q                                        종료
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float64MultiArray
import threading
import sys
import shutil
import time
import os

NAMESPACE = '/pack2'

CALIB = {
    1:  (1107.0,  0.250),
    2:  (1020.0, -0.02525),
    3:  (1020.0,  0.250),
    4:  (1020.0, -0.02525),
    5:  (1064.0,  0.250),
    6:  (1077.0,  0.250),
    7:  (1089.0,  0.250),
    8:  (1065.0,  0.250),
    9:  (1072.0,  0.250),
    10: (1070.0,  0.250),
    11: (1070.0,  0.250),
    12: (1020.0, -0.02525),
    13: (1012.0, -0.02525),
    14: (1032.0, -0.02525),
    15: (1010.0, -0.02525),
    16: (1030.0, -0.02525),
}
ATM_KPA = 101.325

BOARD_NAMES = {
    1:  'P_micro_pos',  2:  'P_micro_neg',
    3:  'P_macro_pos',  4:  'P_macro_neg',
    5:  '양압 ch0',     6:  '양압 ch1',    7:  '양압 ch2',
    8:  '양압 ch3',     9:  '양압 ch4',   10: '양압 ch5',
    11: '음압 ch0',    12: '음압 ch1',   13: '음압 ch2',
    14: '음압 ch3',    15: '음압 ch4',   16: '음압 ch5',
}

DISPLAY_HEIGHT = 32   # 화면 상단에 예약할 줄 수


def mv_to_kpa(board_id, mv):
    if board_id not in CALIB:
        return None
    offset, gain = CALIB[board_id]
    return (mv - offset) * gain + ATM_KPA


class PackNode(Node):
    def __init__(self):
        super().__init__('pack_live_control')
        self._lock     = threading.Lock()
        self._sensors  = {}
        self._currents = {}
        self._angles   = []
        self._pwm      = [[0, 0, 0] for _ in range(16)]

        ns = NAMESPACE
        self.create_subscription(UInt16MultiArray,  f'{ns}/board/sensors',  self._cb_sensors,  10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/currents', self._cb_currents, 10)
        self.create_subscription(Float64MultiArray, f'{ns}/board/analog',   self._cb_analog,   10)
        self._pub = self.create_publisher(UInt16MultiArray, f'{ns}/board/pwm_cmd', 10)

    def _cb_sensors(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                self._sensors[i + 1] = v

    def _cb_currents(self, msg):
        with self._lock:
            n = len(msg.data) // 3
            for i in range(n):
                self._currents[i + 1] = [msg.data[i*3], msg.data[i*3+1], msg.data[i*3+2]]

    def _cb_analog(self, msg):
        with self._lock:
            self._angles = list(msg.data)

    def snapshot(self):
        with self._lock:
            return (dict(self._sensors), dict(self._currents),
                    list(self._angles), [r[:] for r in self._pwm])

    def set_pwm(self, board_id, valve_idx, value):
        with self._lock:
            self._pwm[board_id - 1][valve_idx] = value
        self._flush_pwm()

    def clear_all(self):
        with self._lock:
            self._pwm = [[0, 0, 0] for _ in range(16)]
        self._flush_pwm()

    def _flush_pwm(self):
        with self._lock:
            flat = [v for row in self._pwm for v in row]
        msg = UInt16MultiArray()
        msg.data = flat
        self._pub.publish(msg)


def _build_lines(sensors, currents, angles, pwm_state, cols):
    W = cols
    lines = []

    def L(s=''):
        lines.append(s[:W].ljust(W))

    active = sorted(b for b, mv in sensors.items() if mv > 0 and 1 <= b <= 16)

    L('═' * W)
    L(f'  LIVE CONTROL   {NAMESPACE}')
    L('─' * W)

    if active:
        L(f'  {"Bd":>2}  {"Name":<14}  {"mV":>5}  {"kPa":>9}     '
          f'{"I_v1":>7}  {"I_v2":>7}  {"I_v3":>7} mV     '
          f'{"PWM_v1":>6}  {"PWM_v2":>6}  {"PWM_v3":>6}')
        L('  ' + '─' * (W - 4))
        for bid in active:
            mv   = sensors[bid]
            kpa  = mv_to_kpa(bid, mv)
            name = BOARD_NAMES.get(bid, f'ch{bid}')
            kpa_s = f'{kpa:9.2f}' if kpa is not None else '      N/A'
            cur   = currents.get(bid, [0.0, 0.0, 0.0])
            pw    = pwm_state[bid - 1]
            L(f'  {bid:2d}  {name:<14}  {mv:5d}  {kpa_s} kPa  '
              f'{cur[0]:7.1f}  {cur[1]:7.1f}  {cur[2]:7.1f}     '
              f'{pw[0]:6d}  {pw[1]:6d}  {pw[2]:6d}')
    else:
        L('  (센서 데이터 없음 — can_bridge_node 실행 확인)')

    L('')

    valid_enc = [(i, deg) for i, deg in enumerate(angles) if deg > 0.0]
    if valid_enc:
        L('  Encoder')
        for i, deg in valid_enc:
            L(f'  Board {17 + i}:  {deg:8.2f}°')
        L('')

    L('═' * W)
    return lines


def _redraw(lines, cols):
    out = ['\033[s', '\033[1;1H']
    for i in range(DISPLAY_HEIGHT):
        txt = lines[i][:cols] if i < len(lines) else ''
        out.append(f'\033[2K{txt}\r\n')
    out.append('\033[u')
    sys.stdout.write(''.join(out))
    sys.stdout.flush()


# 하단 고정 행 번호 (1-based)
# 초기화 시: clear → DISPLAY_HEIGHT개 \n → 구분선 → 힌트 → 입력
# row 1..DISPLAY_HEIGHT : 표시 영역
# row DISPLAY_HEIGHT+1  : 구분선
# row DISPLAY_HEIGHT+2  : 힌트
# row DISPLAY_HEIGHT+3  : "> " 입력
# row DISPLAY_HEIGHT+4  : 마지막 명령 상태
INPUT_ROW  = DISPLAY_HEIGHT + 3
STATUS_ROW = DISPLAY_HEIGHT + 4


def main():
    rclpy.init()
    node = PackNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    cols = shutil.get_terminal_size((120, 40)).columns

    # 초기 화면 구성
    os.system('clear')
    sys.stdout.write('\n' * DISPLAY_HEIGHT)                        # 표시 영역 확보
    sys.stdout.write('─' * cols + '\n')                           # 구분선
    sys.stdout.write('  <board 1-16> <valve 1-3> <pwm 0-4095>'
                     '   |   clear   |   q\n')                    # 힌트
    sys.stdout.flush()

    stop_evt = threading.Event()

    def display_loop():
        while not stop_evt.is_set():
            try:
                sensors, currents, angles, pwm = node.snapshot()
                lines = _build_lines(sensors, currents, angles, pwm, cols)
                _redraw(lines, cols)
            except Exception:
                pass
            time.sleep(0.15)

    disp = threading.Thread(target=display_loop, daemon=True)
    disp.start()

    last_status = ''

    try:
        while rclpy.ok():
            # 입력 행 위치 잡기
            sys.stdout.write(f'\033[{INPUT_ROW};1H\033[2K> ')
            if last_status:
                sys.stdout.write(f'\033[{STATUS_ROW};1H\033[2K  {last_status}')
                sys.stdout.write(f'\033[{INPUT_ROW};3H')
            sys.stdout.flush()

            try:
                cmd = input().strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue

            lc = cmd.lower()
            if lc in ('q', 'quit', 'exit'):
                break

            if lc == 'clear':
                node.clear_all()
                last_status = 'all PWM cleared'
                continue

            parts = cmd.split()
            if len(parts) == 3:
                try:
                    bid     = int(parts[0])
                    valve   = int(parts[1])
                    pwm_val = int(parts[2])
                    if not 1 <= bid <= 16:
                        last_status = '[ERR] board: 1~16'
                    elif not 1 <= valve <= 3:
                        last_status = '[ERR] valve: 1~3'
                    elif not 0 <= pwm_val <= 4095:
                        last_status = '[ERR] pwm: 0~4095'
                    else:
                        node.set_pwm(bid, valve - 1, pwm_val)
                        last_status = f'Board {bid}  v{valve} = {pwm_val}  (sent)'
                except ValueError:
                    last_status = '[ERR] 숫자만 입력하세요'
            else:
                last_status = '[ERR] 형식: board valve pwm   예) 5 1 2048'

    except KeyboardInterrupt:
        pass

    stop_evt.set()
    node.clear_all()
    sys.stdout.write(f'\033[{INPUT_ROW + 1};1H\n')
    sys.stdout.flush()
    node.destroy_node()
    rclpy.shutdown()
    print('종료.')


if __name__ == '__main__':
    main()
