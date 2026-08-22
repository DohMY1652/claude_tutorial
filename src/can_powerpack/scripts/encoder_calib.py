#!/usr/bin/env python3
"""encoder_calib.py — 엔코더 2점 캘리브레이션 (축별 0° / 90°)

── 왜 필요한가 ────────────────────────────────────────────────────────────────
엔코더 6개(board 17~22) 중 **17·18·19 만 실측값이 있고 20·21·22 는 없다** (6축인데 3축).
없는 보드는 일반 기본값(`encoder_offset: 1740`, `encoder_gain: 0.07570`)으로 도는데,
그것은 특정 유닛에서 잰 값이라 다른 유닛에서는 각도가 틀린다. 위치 제어의 전제다.

── 무엇을 재는가 ──────────────────────────────────────────────────────────────
`CanBridge` 와 **완전히 같은 식**을 쓴다 (src/CanBridge.cpp:19-22, 71-75):

    orig_mV = (4125 − raw·3300/4095) / 0.825        (반전증폭 역산)
    offset  = orig_mV(0°)
    gain    = 90 / (orig_mV(90°) − orig_mV(0°))     [deg/mV]
    angle   = (orig_mV − offset) · gain

그래서 이 스크립트는 **raw ADC 두 점만** 재서 yaml 에 넣는다. offset/gain 계산은
`CanBridge` 가 기동 시에 한다 — 값이 두 곳에서 갈릴 여지를 없앤다.

raw ADC 는 `board/analog_raw` 로 받는다. `board/analog` 은 이미 캘리브레이션이 적용된
각도라 캘리브레이션 자체에는 쓸 수 없다.

── 준비 ──────────────────────────────────────────────────────────────────────
`can_bridge_node` **만** 띄운다 (`pp_controller` 는 띄우지 않는다):

    ros2 run can_powerpack can_bridge_node --ros-args -r __ns:=/pack2 \\
         --params-file src/can_powerpack/config/powerpack_config.yaml \\
         -p num_actuators:=6

  · `num_actuators:=6` 이 있어야 board 17~22 가 모두 활성화된다.
  · 브리지는 기동 시 **안전 상태**(채널 밸브 폐쇄 + 라인 밸브 전개)로 시작하므로
    레일이 대기압으로 열려 축을 손으로 움직일 수 있다.
  · **펌프는 끈다.** 압력이 걸리면 축이 손으로 움직이지 않고 위험하다.

    python3 src/can_powerpack/scripts/encoder_calib.py --axes 0 1 2 3 4 5

── 각 축에서 하는 일 ──────────────────────────────────────────────────────────
축을 0° 자세로 두고 Enter → 평균 raw 기록 → 90° 자세로 두고 Enter → 평균 raw 기록.
0°/90° 는 **기계적 기준 자세**여야 한다 (지그·스토퍼·각도기). 두 점이 정확할수록
전 구간이 정확해진다.
"""
import argparse
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

ANALOG_BOARD_START = 17          # CanBridge.hpp 와 동일
N_ANALOG = 9                     # board 17~25

# CanBridge.cpp:19-22 와 동일해야 한다. 여기서 갈리면 캘리브레이션이 틀린다.
def raw_to_orig_mv(raw):
    adc_mv = np.clip(np.asarray(raw, dtype=float) * (3300.0 / 4095.0), 0.0, 3300.0)
    return (4125.0 - adc_mv) / 0.825


def calib_from_2pt(raw0, raw90):
    """CanBridge.cpp:71-75 와 동일."""
    mv0, mv90 = raw_to_orig_mv(raw0), raw_to_orig_mv(raw90)
    if abs(mv90 - mv0) < 1e-9:
        return None
    return dict(offset=float(mv0), gain=float(90.0 / (mv90 - mv0)),
                mv0=float(mv0), mv90=float(mv90))


class Rec(Node):
    def __init__(self):
        super().__init__('encoder_calib')
        self.raw = None
        self.deg = None
        self.n_raw = 0   # 새 메시지 수 — sample() 이 중복 표본을 걸러내는 데 쓴다

    def _raw(self, m):
        if len(m.data) >= N_ANALOG:
            self.raw = list(m.data[:N_ANALOG])
            self.n_raw += 1

    def _deg(self, m):
        if len(m.data) >= N_ANALOG:
            self.deg = list(m.data[:N_ANALOG])


def prompt(msg, allowed='ysq'):
    while True:
        a = input(msg).strip().lower()
        if a == '':
            return 'y'
        if a and a[0] in allowed:
            return a[0]
        print(f"    'y'(Enter) / {' / '.join(allowed[1:])} 중에서 입력할 것")


def sample(node, seconds, idx):
    """seconds 동안 raw 를 모아 평균·표준편차를 낸다. 흔들리면 자세가 안 잡힌 것이다."""
    vals = []
    seen = node.n_raw          # 새 메시지가 온 것만 담는다 — 같은 값을 반복 담으면
    t_end = time.time() + seconds   # 표준편차가 작아져 --max-std 게이트가 무력해진다
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.n_raw != seen and node.raw is not None:
            seen = node.n_raw
            vals.append(node.raw[idx])
    if len(vals) < 5:
        return None
    a = np.asarray(vals, dtype=float)
    return dict(mean=float(a.mean()), std=float(a.std()), n=int(a.size),
                lo=float(a.min()), hi=float(a.max()))


def write_yaml(results, out):
    """raw 두 점을 yaml 로 쓴다. offset/gain 은 적지 않는다 — CanBridge 가 계산한다."""
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    # **raw 두 점만 저장한다.** offset/gain 계산은 CanBridge 가 한다 (단일 출처).
    lines = [
        '# ============================================================',
        '# encoder_params.yaml — encoder_calib.py 가 생성한 파일 (직접 편집하지 말 것)',
        f'# 생성: {datetime.now().isoformat(timespec="seconds")}',
        '#',
        '# powerpack_config.yaml **뒤에** 병합한다 (launch 가 자동으로 한다).',
        '#',
        '# raw 두 점만 적는다 — offset/gain 은 CanBridge 가 기동 시 계산한다',
        '#   orig_mV = (4125 − raw·3300/4095) / 0.825',
        '#   offset  = orig_mV(0°),  gain = 90 / (orig_mV(90°) − orig_mV(0°))',
        '# 그래야 값이 두 곳에서 갈릴 여지가 없다.',
        '# ============================================================',
        '/pack2/can_bridge:',
        '  ros__parameters:',
        '    EncoderCalibration:',
        '      boards:',
    ]
    for bid in sorted(results):
        r = results[bid]
        lines.append(f'        "{bid}": {{ raw_0deg: {r["raw_0deg"]}, '
                     f'raw_90deg: {r["raw_90deg"]} }}'
                     f'   # 축 {bid - ANALOG_BOARD_START}, '
                     f'span {r["span"]} LSB, offset {r["offset"]}, gain {r["gain"]}')
    with open(out, 'w') as f:
        f.write('\n'.join(lines) + '\n')

    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--axes', type=int, nargs='+', default=list(range(6)),
                    help='캘리브레이션할 축 (0..5). board = 17 + 축')
    ap.add_argument('--seconds', type=float, default=2.0, help='한 점당 평균 구간 [s]')
    ap.add_argument('--ns', default='pack2')
    ap.add_argument('--out', default=None,
                    help='기본 config/encoder_params.yaml')
    ap.add_argument('--max-std', type=float, default=8.0,
                    help='허용 raw 표준편차 [LSB]. 넘으면 경고 (자세가 안 잡혔거나 진동)')
    ap.add_argument('--min-span', type=float, default=200.0,
                    help='0°~90° 최소 raw 차이 [LSB]. 작으면 기준 자세가 잘못됐다')
    args = ap.parse_args()

    rclpy.init(args=None)
    node = Rec()
    # 노드에 네임스페이스를 주지 않았으므로 상대명은 '/board/...' 로 풀린다 → 안 붙는다.
    # 절대명으로만 구독한다.
    node.create_subscription(UInt16MultiArray, f'/{args.ns}/board/analog_raw',
                             node._raw, 20)
    node.create_subscription(Float64MultiArray, f'/{args.ns}/board/analog',
                             node._deg, 20)

    print('=' * 74)
    print('엔코더 2점 캘리브레이션')
    print('  can_bridge_node 만 띄운 상태여야 한다 (pp_controller 금지, 펌프 OFF).')
    print('  브리지는 안전 상태(채널 폐쇄 + 라인 전개)로 시작하므로 손으로 움직일 수 있다.')
    print('=' * 74)

    print('\nboard/analog_raw 수신 대기...')
    t_end = time.time() + 10.0
    while time.time() < t_end and node.raw is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.raw is None:
        print('!! board/analog_raw 가 오지 않는다. 확인할 것:')
        print('   · can_bridge_node 가 돌고 있는가 (ros2 topic hz /%s/board/analog_raw)'
              % args.ns)
        print('   · 빌드가 최신인가 (analog_raw 발행은 이번에 추가됐다)')
        print('   · --ns 가 맞는가')
        return 1
    print(f'  수신 OK ({node.n_raw} 메시지)')

    results = {}
    try:
        for ax in args.axes:
            bid = ANALOG_BOARD_START + ax
            idx = ax                                  # analog[0] = board 17
            if idx < 0 or idx >= N_ANALOG:
                print(f'축 {ax} 는 범위 밖이다 — 건너뜀')
                continue
            print('\n' + '─' * 74)
            print(f'[축 {ax}] board {bid}')
            cur = node.raw[idx] if node.raw else None
            print(f'  현재 raw = {cur}  (참고: 현재 각도 표시 '
                  f'{node.deg[idx]:.2f}°)' if node.deg else f'  현재 raw = {cur}')

            a = prompt('  이 축을 캘리브레이션? (Enter=예, s=건너뛰기, q=종료): ', 'ysq')
            if a == 'q':
                break
            if a == 's':
                continue

            pts = {}
            for name, ang in (('raw_0deg', 0.0), ('raw_90deg', 90.0)):
                while True:
                    prompt(f'  축을 **{ang:.0f}°** 기준 자세로 두고 Enter: ', 'y')
                    st = sample(node, args.seconds, idx)
                    if st is None:
                        print('    !! 표본이 5개도 안 왔다 — 브리지가 발행하는지 확인 (--seconds 늘려도 된다)')
                        continue
                    print(f'    raw 평균 {st["mean"]:.1f}  표준편차 {st["std"]:.2f}  '
                          f'범위 {st["lo"]:.0f}~{st["hi"]:.0f}  (n={st["n"]})')
                    if st['std'] > args.max_std:
                        print(f'    !! 표준편차가 {args.max_std:.0f} LSB 를 넘는다 — '
                              f'자세가 안 잡혔거나 진동이 있다.')
                        if prompt('       다시 잴까? (Enter=다시, s=그대로 진행): ',
                                  'ys') == 'y':
                            continue
                    pts[name] = st
                    break

            r0, r90 = pts['raw_0deg']['mean'], pts['raw_90deg']['mean']
            span = abs(r90 - r0)
            cal = calib_from_2pt(r0, r90)
            print(f'\n  0°~90° raw 차이 {span:.1f} LSB')
            if span < args.min_span:
                print(f'  !! {args.min_span:.0f} LSB 미만이다 — 기준 자세가 잘못됐거나 '
                      f'엔코더가 그 구간에서 안 움직인다. 저장하지 않는다.')
                continue
            if cal is None:
                print('  !! 두 점이 같다 — 저장하지 않는다.')
                continue
            print(f'  → offset {cal["offset"]:.2f} mV,  gain {cal["gain"]:.6f} deg/mV')
            print(f'    (참고: 기존 기본값 offset 1740.00, gain 0.075702)')
            # 방향 확인 — 각도가 증가하는 쪽이 raw 증가인지 감소인지
            print(f'    raw {r0:.0f} → {r90:.0f} 이면 '
                  f'{"raw 증가 = 각도 증가" if r90 > r0 else "raw 감소 = 각도 증가"}')

            results[bid] = dict(raw_0deg=round(r0, 1), raw_90deg=round(r90, 1),
                                span=round(span, 1),
                                std0=round(pts['raw_0deg']['std'], 2),
                                std90=round(pts['raw_90deg']['std'], 2),
                                offset=round(cal['offset'], 3),
                                gain=round(cal['gain'], 8))
    except KeyboardInterrupt:
        print('\n중단 요청')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not results:
        print('\n저장할 결과가 없다.')
        return 1

    out = args.out or os.path.join('src', 'can_powerpack', 'config',
                                   'encoder_params.yaml')
    write_yaml(results, out)
    print('\n' + '=' * 74)
    print(f'저장: {out}')
    print(f'{"board":>6} {"축":>3} {"raw 0°":>9} {"raw 90°":>9} {"span":>7} '
          f'{"offset":>9} {"gain":>10}')
    for bid in sorted(results):
        r = results[bid]
        print(f'{bid:>6} {bid - ANALOG_BOARD_START:>3} {r["raw_0deg"]:>9.1f} '
              f'{r["raw_90deg"]:>9.1f} {r["span"]:>7.1f} {r["offset"]:>9.2f} '
              f'{r["gain"]:>10.6f}')
    print('=' * 74)
    print('다음: 재빌드하면 launch 가 이 파일을 병합한다.')
    print('  colcon build --packages-select can_powerpack '
          '--cmake-args -DCMAKE_BUILD_TYPE=Release')
    print('확인: board/analog 의 각도가 실제 자세와 맞는지 두 자세에서 눈으로 볼 것.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
