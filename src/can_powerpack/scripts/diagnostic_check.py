#!/usr/bin/env python3
"""
Diagnostic: sensor monitoring + actuator PWM response check (1-actuator config).

Phase 1 - Sensor monitor (5초): 라인압 4개 + 액추에이터 압력/전류 + 엔코더 표시
Phase 2 - Actuator test: 각 밸브에 PWM 인가 후 전류 응답으로 작동 확인
Phase 3 - Continuous sensor monitor (Ctrl+C 종료)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float64MultiArray
import threading
import time
import sys

NAMESPACE  = '/pack2'
PWM_TEST   = 2048    # 50% PWM
SETTLE_S   = 0.5     # 밸브 안정화 대기
THRESHOLD  = 100.0   # mV: 전류 변화 판정 기준

LINE_BOARDS = {
    1: 'P_micro_pos',
    2: 'P_micro_neg',
    3: 'P_macro_pos',
    4: 'P_macro_neg',
}
ACT_BOARDS = [
    (5,  '양압 ch0'),
    (11, '음압 ch0'),
]

CALIB = {
    1:  (1107.0,  0.250),
    2:  (1020.0, -0.02525),
    3:  (1020.0,  0.250),
    4:  (1020.0, -0.02525),
    5:  (1064.0,  0.250),
    11: (1020.0, -0.02525),
}
ATM_KPA = 101.325


def mv_to_kpa(board_id, mv):
    if board_id not in CALIB:
        return None
    offset, gain = CALIB[board_id]
    return (mv - offset) * gain + ATM_KPA


class DiagNode(Node):
    def __init__(self):
        super().__init__('diag_node')
        ns = NAMESPACE
        self._sensors  = {}   # board_id -> mV (int)
        self._currents = {}   # board_id -> [i1, i2, i3] mV
        self._angles   = []   # deg, index 0 = board 17
        self._lock     = threading.Lock()

        self.create_subscription(
            UInt16MultiArray, f'{ns}/board/sensors',  self._on_sensors,  10)
        self.create_subscription(
            Float64MultiArray, f'{ns}/board/currents', self._on_currents, 10)
        self.create_subscription(
            Float64MultiArray, f'{ns}/board/analog',   self._on_analog,   10)
        self._pub = self.create_publisher(
            UInt16MultiArray, f'{ns}/board/pwm_cmd', 10)

    def _on_sensors(self, msg):
        with self._lock:
            for i, v in enumerate(msg.data):
                self._sensors[i + 1] = v

    def _on_currents(self, msg):
        with self._lock:
            n = len(msg.data) // 3
            for i in range(n):
                self._currents[i + 1] = [
                    msg.data[i*3],
                    msg.data[i*3 + 1],
                    msg.data[i*3 + 2],
                ]

    def _on_analog(self, msg):
        with self._lock:
            self._angles = list(msg.data)

    def snapshot(self):
        with self._lock:
            return dict(self._sensors), dict(self._currents), list(self._angles)

    def send_pwm(self, board_id, valve_idx, value):
        msg = UInt16MultiArray()
        msg.data = [0] * (16 * 3)
        msg.data[(board_id - 1) * 3 + valve_idx] = value
        self._pub.publish(msg)

    def send_pwm_zero(self):
        msg = UInt16MultiArray()
        msg.data = [0] * (16 * 3)
        self._pub.publish(msg)


def print_sensors(sensors, currents, angles):
    print("\033[H\033[2J", end='')
    print("━" * 65)
    print("  SENSOR MONITOR")
    print("━" * 65)

    print("\n▶ 라인압 센서 (boards 1~4)")
    for bid, name in LINE_BOARDS.items():
        mv  = sensors.get(bid, 0)
        kpa = mv_to_kpa(bid, mv)
        kpa_s = f"{kpa:7.1f} kPa" if kpa is not None else "    N/A  "
        print(f"  Board {bid} ({name:<15s}):  {mv:5d} mV  →  {kpa_s}")

    print("\n▶ 액추에이터 (board 5=양압, board 11=음압)")
    for bid, name in ACT_BOARDS:
        mv  = sensors.get(bid, 0)
        kpa = mv_to_kpa(bid, mv)
        kpa_s = f"{kpa:7.1f} kPa" if kpa is not None else "    N/A  "
        cur = currents.get(bid, [0.0, 0.0, 0.0])
        print(f"  Board {bid:2d} ({name}):  {mv:5d} mV  →  {kpa_s}"
              f"  |  전류 [{cur[0]:6.1f}, {cur[1]:6.1f}, {cur[2]:6.1f}] mV")

    print("\n▶ 엔코더 (board 17)")
    ang = angles[0] if angles else 0.0
    print(f"  {ang:6.2f}°")
    print()


def run_monitor(node, stop_event, duration=None):
    start = time.time()
    while not stop_event.is_set():
        sensors, currents, angles = node.snapshot()
        print_sensors(sensors, currents, angles)
        if duration and (time.time() - start) >= duration:
            break
        time.sleep(0.3)


def test_actuators(node):
    print("\033[H\033[2J", end='')
    print("━" * 65)
    print("  ACTUATOR TEST  (PWM 인가 → 전류 응답 확인)")
    print("━" * 65)
    print(f"  PWM={PWM_TEST}  판정: Δ ≥ {THRESHOLD:.0f} mV\n")

    VALVES = ['v1 (micro-in)', 'v2 (micro-out)', 'v3 (macro)']
    results = []

    for board_id, ch_name in ACT_BOARDS:
        print(f"  ▶ Board {board_id} ({ch_name})")
        for v_idx, v_name in enumerate(VALVES):
            _, cur_before, _ = node.snapshot()
            i_before = cur_before.get(board_id, [0.0]*3)[v_idx]

            node.send_pwm(board_id, v_idx, PWM_TEST)
            time.sleep(SETTLE_S)

            _, cur_after, _ = node.snapshot()
            i_after = cur_after.get(board_id, [0.0]*3)[v_idx]

            node.send_pwm_zero()
            time.sleep(0.3)

            delta  = abs(i_after - i_before)
            ok     = delta >= THRESHOLD
            status = "OK  ✓" if ok else "FAIL ✗"
            print(f"    {v_name:<16s}: {i_before:6.1f} → {i_after:6.1f} mV"
                  f"  (Δ={delta:5.1f})  [{status}]")
            results.append((board_id, v_name, ok))

        print()

    passed = sum(1 for _, _, ok in results if ok)
    total  = len(results)
    print(f"  결과: {passed}/{total} 통과\n")
    return results


def main():
    rclpy.init()
    node = DiagNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("센서 데이터 수신 대기 중... (2초)")
    time.sleep(2.0)

    # Phase 1: sensor monitor 5초
    stop_evt = threading.Event()
    monitor = threading.Thread(
        target=run_monitor, args=(node, stop_evt, 5.0), daemon=True)
    monitor.start()
    monitor.join()
    stop_evt.set()

    # Phase 2: actuator test
    test_actuators(node)

    # Phase 3: continuous sensor monitor
    print("센서 모니터링 모드 (Ctrl+C 종료)\n")
    time.sleep(1.0)
    stop_evt2 = threading.Event()
    try:
        run_monitor(node, stop_evt2)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
