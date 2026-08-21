#!/usr/bin/env python3
"""
pump_fit_record.py — 펌프 파라미터 실험 구동 + 동기 기록

레일은 **대기 창구가 정확히 두 개뿐인 닫힌 회로**다 (board1 v1 = 양압→대기,
board2 v1 = 대기→음압). board 3(탱크)·board 4(이젝터)는 레일에 영향을 주지 못하고, 채널
v1(micro)만 레일에 붙으므로 채널 PWM 을 전부 0 으로 두면 끊긴다. 따라서 **두 밸브를 닫으면
밸브 유량이 0** 이 되고 부피만으로 펌프 유량이 직접 나온다:

    ṁ_pump = +V⁺/(R·T)·dP⁺/dt + leak⁺(P⁺)      (양압 레일)
           = −V⁻/(R·T)·dP⁻/dt + leak⁻(P⁻)      (음압 레일)

두 식이 질량보존으로 같아야 하므로 **매 측정점에서 교차검증**이 된다. 라인 밸브 특성 모델이
전혀 필요 없다는 것이 이 설계의 핵심이다.

── 3단계 ────────────────────────────────────────────────────────────────────
  leak      펌프 OFF. 양 밸브 폐쇄 상태의 감쇠 → 지수 시상수 τ.
            ΔV 회차의 τ 비가 (V+ΔV)/V 이므로 레일 부피와 누설이 함께 풀린다.
  map       펌프 ON. (u_relief, u_admit) 격자에서 정착 → 양 밸브 짧게 폐쇄 →
            두 레일의 초기 dP/dt → ṁ_pump(P⁺,P⁻) + 질량보존 교차검증.
  frontier  펌프 ON. board1 v1 폐쇄로 P⁺ 를 스톨까지 램프 → 능력경계 직접 측정.
            안전 상한에 먼저 닿으면 '하한 경계'로 기록한다.

── 전제 조건 ────────────────────────────────────────────────────────────────
  · pp_controller 를 띄우지 말 것 (같은 board/pwm_cmd 에 중재 없이 발행). 시작 시 검사한다.
  · can_bridge_node 만 기동:
      ros2 run can_powerpack can_bridge_node --ros-args --namespace pack2 \
          --params-file src/can_powerpack/config/powerpack_config.yaml
  · 펌프 on/off 는 **수동 스위치**다. 프롬프트에서 직접 조작한다 (타이밍 정확도는 불필요).
  · 채널 밸브는 전부 닫힌 상태로 유지된다 (스크립트가 매 사이클 0 을 발행).

── 안전 ────────────────────────────────────────────────────────────────────
  valve_fit_record.py 와 달리 **"전 밸브 0" 이 안전 상태가 아니다** — 펌프가 돌고 있어서
  board1 v1 을 닫으면 양압 레일이 무한정 올라간다. 안전 상태는 **양 밸브 전개**다.
  · 양압 상한 / 음압 하한 예측 정지 (밸브 닫힘 꼬리를 앞질러 판정) + 도달 시 즉시 전개
  · 종료·예외·Ctrl-C 시 양 밸브 전개로 두 레일을 대기압으로 되돌린 뒤 0
  · CanBridge 에는 워치독이 없다 — 마지막 명령이 250 Hz 로 영구히 나간다

사용 예:
  python3 pump_fit_record.py --phase leak
  python3 pump_fit_record.py --phase leak --extra-volume-ml 100 --extra-volume-rail pos
  python3 pump_fit_record.py --phase map
  python3 pump_fit_record.py --phase frontier --ppos-ceiling 500
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from valve_fit_record import (PCT_TO_PWM, PWM_BOARDS, PWM_LEN, PWM_PER_BOARD,  # noqa: E402
                              BOARD_LINE_POS, BOARD_LINE_NEG, BOARD_MACRO,
                              BOARD_MACRO_NEG, load_calib, prompt)

IDX_RELIEF = (BOARD_LINE_POS - 1) * PWM_PER_BOARD + 0   # board1 v1 = 0 : 양압 → 대기
IDX_ADMIT = (BOARD_LINE_NEG - 1) * PWM_PER_BOARD + 0    # board2 v1 = 3 : 대기 → 음압
READ_BOARDS = (BOARD_LINE_POS, BOARD_LINE_NEG, BOARD_MACRO, BOARD_MACRO_NEG)
# 0점 검증 대상 — 레일뿐이다. board 3(탱크 레귤레이터)·board 4(외부 진공)는
# 구조상 대기압이 될 수 없어서 검증에 넣으면 실기에서 항상 중단된다.
ZERO_CHECK_BOARDS = (BOARD_LINE_POS, BOARD_LINE_NEG)

CSV_HEADER = [
    't', 'phase', 'sub', 'point', 'pump_on', 'u_relief', 'u_admit',
    'P_pos', 'P_neg', 'P_macro', 'P_macro_neg', 'P_atm',
    'extra_vol_ml', 'extra_vol_rail',
]

DEFAULT_U_GRID = [25.0, 40.0, 55.0, 70.0, 85.0, 100.0]
DEFAULT_PNEG_TARGETS = [-80.0, -65.0, -50.0, -35.0]   # kPa gauge


class RailRecorder(Node):
    def __init__(self, args, calib):
        super().__init__('pump_fit_record')
        self.args = args
        self.calib = calib
        self.atm = calib['atm']

        ns = args.ns.strip('/')
        pre = f'/{ns}/' if ns else '/'
        self.topic_pwm = pre + 'board/pwm_cmd'
        self.pub = self.create_publisher(UInt16MultiArray, self.topic_pwm, 10)
        self.create_subscription(UInt16MultiArray, pre + 'board/sensors', self._on_sensors, 20)
        self.create_subscription(Float64MultiArray, pre + 'board/currents',
                                 self._on_currents, 20)

        self._lock = threading.Lock()
        self._pwm = [0] * PWM_LEN
        self._sens_mv = [0.0] * PWM_BOARDS
        self._seen = False
        self._offset = {b: calib[b][0] for b in calib if isinstance(b, int)}

        self.tag = dict(phase='idle', sub='-', point=-1, pump_on=1)
        self.rows = []
        self.t0 = time.time()
        self.tripped = None

        self.create_timer(1.0 / args.log_hz, self._tick)

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def _on_sensors(self, msg):
        with self._lock:
            n = min(len(msg.data), PWM_BOARDS)
            self._sens_mv[:n] = [float(v) for v in msg.data[:n]]
            self._seen = True

    def _on_currents(self, _msg):
        pass          # 라인 밸브 전류는 이 실험에 쓰이지 않는다 (모델이 필요 없다)

    # ── 압력 ──────────────────────────────────────────────────────────────
    def kpa(self, board_id):
        with self._lock:
            mv = self._sens_mv[board_id - 1]
        off = self._offset.get(board_id, 0.0)
        gain = self.calib.get(board_id, (0.0, 0.0))[1]
        return (mv - off) * gain + self.atm

    def p_pos(self):
        return self.kpa(BOARD_LINE_POS)

    def p_neg(self):
        return self.kpa(BOARD_LINE_NEG)

    # ── 밸브 ──────────────────────────────────────────────────────────────
    def set_valves(self, u_relief, u_admit):
        with self._lock:
            self._pwm = [0] * PWM_LEN     # 채널 밸브는 항상 0 으로 유지
            self._pwm[IDX_RELIEF] = int(round(max(0.0, min(100.0, u_relief)) * PCT_TO_PWM))
            self._pwm[IDX_ADMIT] = int(round(max(0.0, min(100.0, u_admit)) * PCT_TO_PWM))
        self.flush()

    def open_both(self):
        """안전 상태 — 양 밸브 전개로 두 레일을 대기압 쪽으로 되돌린다.
        '전부 0' 은 펌프가 도는 동안 양압 레일이 무한정 오르므로 안전하지 않다."""
        self.set_valves(100.0, 100.0)

    def all_zero(self):
        with self._lock:
            self._pwm = [0] * PWM_LEN
        self.flush()

    def flush(self):
        if self.args.dry_run and not self.args.publish_in_dry_run:
            return
        msg = UInt16MultiArray()
        with self._lock:
            msg.data = list(self._pwm)
        self.pub.publish(msg)

    # ── 기록 + 트립 ───────────────────────────────────────────────────────
    def _tick(self):
        if not self._seen:
            return
        pp, pn = self.p_pos(), self.p_neg()

        if self.tripped is None:
            if pp > self.args.trip_hi:
                self._trip(f'양압 레일 {pp:.1f} > {self.args.trip_hi:.1f} kPa')
            elif pn < self.args.trip_lo:
                self._trip(f'음압 레일 {pn:.1f} < {self.args.trip_lo:.1f} kPa')

        with self._lock:
            u_r = self._pwm[IDX_RELIEF] / PCT_TO_PWM
            u_a = self._pwm[IDX_ADMIT] / PCT_TO_PWM
        self.rows.append([
            round(time.time() - self.t0, 5), self.tag['phase'], self.tag['sub'],
            self.tag['point'], self.tag['pump_on'], round(u_r, 3), round(u_a, 3),
            round(pp, 4), round(pn, 4),
            round(self.kpa(BOARD_MACRO), 4), round(self.kpa(BOARD_MACRO_NEG), 4),
            self.atm, self.args.extra_volume_ml, self.args.extra_volume_rail,
        ])

    def _trip(self, why):
        self.tripped = why
        self.open_both()
        self.get_logger().error(f'[TRIP] {why} — 양 밸브 전개, 중단')

    # ── 유틸 ──────────────────────────────────────────────────────────────
    def zero_offsets(self, seconds=0.5, tol_kpa=8.0):
        """대기 개방 상태에서 offset 재취득 + 검증.

        압력이 남은 채 보정하면 그 값이 오프셋으로 흡수돼 트립 기준까지 틀어진다.
        단 **검증은 레일(board 1·2)만** 한다 — board 3(탱크 레귤레이터 출력)과
        board 4(외부 인가 진공)는 구조상 대기압이 될 수 없으므로 검증에 넣으면
        실기에서 항상 중단된다. offset 자체는 네 보드 모두 갱신한다.
        """
        acc = {b: [] for b in READ_BOARDS}
        t_end = time.time() + seconds
        while time.time() < t_end:
            with self._lock:
                for b in READ_BOARDS:
                    acc[b].append(self._sens_mv[b - 1])
            time.sleep(0.005)
        bad, new = [], {}
        for b in READ_BOARDS:
            mv = sum(acc[b]) / len(acc[b])
            yaml_off, gain = self.calib.get(b, (mv, 0.0))
            drift = abs(mv - yaml_off) * abs(gain)
            new[b] = (mv, drift)
            if b in ZERO_CHECK_BOARDS and drift > tol_kpa:
                bad.append(f'board {b}: {drift:.1f} kPa 상당 (mV {mv:.0f} vs yaml {yaml_off:.0f})')
        if bad:
            raise SystemExit('0점 보정값이 yaml 과 너무 다르다 — 레일에 압력이 남아 있다.\n  '
                             + '\n  '.join(bad)
                             + '\n**펌프를 끄고** 양 밸브를 열어 대기압으로 되돌린 뒤 다시 '
                               '시작할 것 (--no-zero 로 yaml 값 사용 가능).')
        for b, (mv, _) in new.items():
            self._offset[b] = mv
        return {b: f'{mv:.0f}mV(Δ{d:.1f}kPa)' for b, (mv, d) in new.items()}

    def rate(self, getter, window=0.12):
        """짧은 창에서 dP/dt [kPa/s] 를 선형회귀로."""
        ts, ps = [], []
        t_end = time.time() + window
        while time.time() < t_end:
            ts.append(time.time())
            ps.append(getter())
            time.sleep(0.005)
        if len(ts) < 4:
            return 0.0
        return float(np.polyfit(np.array(ts) - ts[0], np.array(ps), 1)[0])


# ══════════════════════════════════════════════════════════════════════════
# 대기 헬퍼
# ══════════════════════════════════════════════════════════════════════════
def wait_limits(node, args, timeout, settle_eps=None, dp_limit=None, poll=0.01):
    """상한/하한(예측 포함)·정착·압력변화폭·타임아웃 중 먼저 오는 것까지 대기.
    반환: 'limit' | 'settled' | 'dp' | 'timeout' | 'tripped'"""
    t_end = time.time() + timeout
    p0_pos, p0_neg = node.p_pos(), node.p_neg()
    prev = (time.time(), p0_pos, p0_neg)
    calm_since = None
    while time.time() < t_end:
        if node.tripped:
            return 'tripped'
        time.sleep(poll)
        now, pp, pn = time.time(), node.p_pos(), node.p_neg()
        dt = now - prev[0]
        if dt <= 0:
            continue
        rp, rn = (pp - prev[1]) / dt, (pn - prev[2]) / dt
        prev = (now, pp, pn)
        # 예측 정지 — 밸브 닫힘 꼬리를 앞지른다
        if max(pp, pp + rp * args.stop_lead) >= args.ppos_ceiling:
            return 'limit'
        if min(pn, pn + rn * args.stop_lead) <= args.pneg_floor:
            return 'limit'
        if dp_limit is not None and (abs(pp - p0_pos) >= dp_limit
                                     or abs(pn - p0_neg) >= dp_limit):
            return 'dp'
        if settle_eps is not None and abs(rp) <= settle_eps and abs(rn) <= settle_eps:
            calm_since = calm_since or now
            if now - calm_since >= 0.3:
                return 'settled'
        else:
            calm_since = None
    return 'timeout'


# ══════════════════════════════════════════════════════════════════════════
# Phase L — 누설 + 부피 (펌프 OFF)
# ══════════════════════════════════════════════════════════════════════════
def phase_leak(node, args):
    print('\n── Phase L: 누설 + 부피 (펌프 OFF) ──')
    print(f'  ΔV = {args.extra_volume_ml:.0f} mL  (레일: {args.extra_volume_rail})')
    if args.extra_volume_ml > 0:
        print(f'  → 알려진 부피 {args.extra_volume_ml:.0f} mL 용기를 '
              f'{args.extra_volume_rail} 레일에 티로 연결했는지 확인할 것')

    if prompt('  펌프를 켜고 Enter (q=종료): ', 'yq') == 'q':
        return False
    node.tag.update(phase='leak', sub='charge', point=0, pump_on=1)

    print('  양 밸브 폐쇄 — 레일을 벌린다 (상한/하한에서 멈춤)')
    node.set_valves(0.0, 0.0)
    why = wait_limits(node, args, args.charge_timeout)
    if why == 'tripped':
        return False
    print(f'    P⁺={node.p_pos():.1f}  P⁻={node.p_neg():.1f} kPa  ({why})')

    if prompt('  이제 **펌프를 끄고** Enter (q=종료): ', 'yq') == 'q':
        return False
    node.tag.update(sub='decay', pump_on=0)
    print(f'  감쇠 기록 {args.leak_seconds:.0f} s (양 밸브 폐쇄 유지)...')
    node.set_valves(0.0, 0.0)
    t_end = time.time() + args.leak_seconds
    while time.time() < t_end and not node.tripped:
        time.sleep(0.2)
    print(f'    종료  P⁺={node.p_pos():.1f}  P⁻={node.p_neg():.1f} kPa')
    node.tag.update(sub='-', pump_on=1)
    return True


# ══════════════════════════════════════════════════════════════════════════
# Phase M — 2D 유량 맵 (펌프 ON)
# ══════════════════════════════════════════════════════════════════════════
def phase_map(node, args):
    print('\n── Phase M: 2D 유량 맵 (펌프 ON) ──')
    if prompt('  펌프를 켜고 Enter (q=종료): ', 'yq') == 'q':
        return False

    grid = [(u1, u2) for u1 in args.u_grid for u2 in args.u_grid]
    print(f'  격자 {len(grid)} 점.  펄스 {args.pulse_s:.2f} s / Δ{args.pulse_dp:.0f} kPa 제한')
    n_ok = 0
    for k, (u1, u2) in enumerate(grid):
        if node.tripped:
            break
        node.tag.update(phase='map', sub='hold', point=k)
        node.set_valves(u1, u2)
        why = wait_limits(node, args, args.settle_timeout, settle_eps=args.settle_eps)
        if why == 'tripped':
            break
        pp, pn = node.p_pos(), node.p_neg()

        # 펄스: 양 밸브 완전 폐쇄 → 밸브 유량 0 → 부피만으로 펌프 유량이 나온다
        node.tag.update(sub='pulse')
        node.set_valves(0.0, 0.0)
        why2 = wait_limits(node, args, args.pulse_s, dp_limit=args.pulse_dp)
        node.tag.update(sub='reopen')
        node.open_both()
        time.sleep(args.reopen_s)
        n_ok += 1
        print(f'    [{k+1:3d}/{len(grid)}] u=({u1:5.1f},{u2:5.1f})  '
              f'P⁺={pp:6.1f} P⁻={pn:6.1f} kPa  펄스종료={why2}')
    node.tag.update(phase='map', sub='-', point=-1)
    print(f'  {n_ok}/{len(grid)} 점 기록')
    return not node.tripped


# ══════════════════════════════════════════════════════════════════════════
# Phase F — 능력경계 직접 측정 (펌프 ON)
# ══════════════════════════════════════════════════════════════════════════
def ramp_with_pneg_hold(node, args, pneg_target_abs, timeout):
    """board1 v1 을 닫아 P⁺ 를 램프하면서, admit 밸브로 P⁻ 를 목표에 붙여 둔다.

    고정 개도로 램프하면 펌프 유량이 줄어드는 만큼 P⁻ 가 대기 쪽으로 끌려가 능력경계
    표본이 한쪽으로 뭉친다 (실측: u_admit 40/60 이 모두 크래킹 아래라 같은 −85 kPa 로 수렴).
    비례 제어로 P⁻ 를 잡으면 원하는 음압 깊이에서의 경계를 곧바로 얻는다.
    """
    t_end = time.time() + timeout
    prev = (time.time(), node.p_pos(), node.p_neg())
    u_admit = args.admit_start
    calm_since = None
    while time.time() < t_end:
        if node.tripped:
            return 'tripped', u_admit
        time.sleep(0.02)
        now, pp, pn = time.time(), node.p_pos(), node.p_neg()
        dt = now - prev[0]
        if dt <= 0:
            continue
        rp = (pp - prev[1]) / dt
        prev = (now, pp, pn)
        # P⁻ 비례 제어 — 목표보다 깊으면 admit 을 더 열고, 얕으면 닫는다
        u_admit += args.admit_kp * (pneg_target_abs - pn) * dt
        u_admit = min(max(u_admit, args.admit_min), 100.0)
        node.set_valves(0.0, u_admit)
        if max(pp, pp + rp * args.stop_lead) >= args.ppos_ceiling:
            return 'limit', u_admit
        if abs(rp) <= args.stall_eps:
            calm_since = calm_since or now
            if now - calm_since >= 0.5:
                return 'settled', u_admit
        else:
            calm_since = None
    return 'timeout', u_admit


def phase_frontier(node, args):
    print('\n── Phase F: 능력경계 직접 측정 (펌프 ON) ──')
    print(f'  안전 상한 {args.ppos_ceiling:.0f} kPa abs '
          f'(= {args.ppos_ceiling - node.atm:.0f} kPa gauge). 여기 닿으면 하한 경계로 기록.')
    if prompt('  펌프를 켜고 Enter (q=종료): ', 'yq') == 'q':
        return False

    targets = [node.atm + t for t in args.pneg_targets]
    for k, tgt in enumerate(targets):
        if node.tripped:
            break
        # 먼저 양 밸브를 열어 대기압 근처에서 시작
        node.tag.update(phase='frontier', sub='reset', point=k)
        node.open_both()
        wait_limits(node, args, 8.0, settle_eps=args.settle_eps)

        node.tag.update(sub='ramp')
        print(f'    [{k+1}/{len(targets)}] P⁻ 목표 {tgt - node.atm:+.1f} kPa gauge '
              f'→ board1 v1 폐쇄, P⁺ 램프...')
        why, u_end = ramp_with_pneg_hold(node, args, tgt, args.frontier_timeout)
        # 상한에 닿아 멈춘 경우에는 **즉시 제동**해야 한다. 릴리프가 닫힌 채로 스톨 확인
        # 유지를 하면 그 시간만큼 P⁺ 가 더 오른다 — 리허설 계측: 1 s 유지에 30 kPa 더 올라
        # 상한 501 을 넘어 트립(541)이 걸렸다. 진짜 스톨(dP/dt≈0)일 때만 길게 유지한다.
        node.tag.update(sub='stall')
        time.sleep(args.stall_hold if why == 'settled' else args.limit_hold)
        pp2, pn2 = node.p_pos(), node.p_neg()
        # **배기 구간을 'stall' 로 태깅하면 안 된다.** 솔버는 sub='stall' 행의 중앙값을
        # 경계점으로 쓰는데, 릴리프를 연 뒤 reopen_s 동안 압력이 무너지므로 짧은
        # limit_hold 와 합쳐지면 중앙값이 경계가 아닌 배기 중간값이 된다
        # (리허설: 503 kPa gauge 경계가 276~299 로 읽혀 상한 도달 판정까지 놓쳤다).
        node.tag.update(sub='vent')
        node.open_both()
        time.sleep(args.reopen_s)
        kind = '하한 경계(상한 도달)' if why == 'limit' else \
               ('스톨' if why == 'settled' else f'미완({why})')
        print(f'       u_admit 종료값 {u_end:.1f}%')
        print(f'       P⁺={pp2:6.1f} ({pp2 - node.atm:+6.1f} gauge)  '
              f'P⁻={pn2:6.1f} kPa  → {kind}')
        # 순 초과 ≈ (폴링 주기 + 밸브 개방 시간) × 램프율 이라 램프율에 비례한다.
        # 레일 부피가 작으면 램프율이 커져 트립 여유를 잠식하므로 미리 경고한다.
        excess = pp2 - args.ppos_ceiling
        if excess > 0.5 * args.trip_margin:
            print(f'       !! 상한 초과 {excess:.1f} kPa — 트립 여유 {args.trip_margin:.0f} 의 '
                  f'절반을 넘었다. --stop-lead 를 {args.stop_lead * 2:.2f} 로 올리거나 '
                  f'--trip-margin 을 키울 것.')
    node.tag.update(phase='frontier', sub='-', point=-1)
    return not node.tripped


# ══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--phase', required=True, choices=['leak', 'map', 'frontier'])
    ap.add_argument('--extra-volume-ml', type=float, default=0.0,
                    help='이중 부피법: 티로 추가한 알려진 부피 [mL]')
    ap.add_argument('--extra-volume-rail', default='none', choices=['none', 'pos', 'neg'])
    ap.add_argument('--config', default=None)
    ap.add_argument('--ns', default='pack2')
    ap.add_argument('--out', default=None)
    ap.add_argument('--log-hz', type=float, default=200.0)

    ap.add_argument('--u-grid', type=float, nargs='*', default=DEFAULT_U_GRID,
                    help='Phase M 격자 개도 [%%] (relief × admit)')
    ap.add_argument('--pneg-targets', type=float, nargs='*', default=DEFAULT_PNEG_TARGETS,
                    help='Phase F 에서 잡아둘 음압 목표 [kPa gauge]')
    ap.add_argument('--admit-kp', type=float, default=3.0,
                    help='P⁻ 유지 비례 게인 [%%/(kPa·s)]')
    ap.add_argument('--admit-start', type=float, default=70.0,
                    help='admit 초기 개도 [%%]. 크래킹(≈62%% @대기압) 위에서 출발한다.')
    ap.add_argument('--admit-min', type=float, default=0.0)
    ap.add_argument('--leak-seconds', type=float, default=90.0)
    ap.add_argument('--charge-timeout', type=float, default=30.0)
    ap.add_argument('--settle-timeout', type=float, default=12.0)
    ap.add_argument('--frontier-timeout', type=float, default=60.0)
    ap.add_argument('--pulse-s', type=float, default=0.6,
                    help='양 밸브 폐쇄 펄스 길이 [s]. 동작점이 거의 안 움직일 만큼 짧게.')
    ap.add_argument('--pulse-dp', type=float, default=70.0,
                    help='펄스 중 허용 압력 변화폭 [kPa]. 유량이 크면 시간보다 이게 먼저 끝낸다.')
    ap.add_argument('--reopen-s', type=float, default=0.4)
    ap.add_argument('--settle-eps', type=float, default=1.5, help='정착 판정 |dP/dt| [kPa/s]')
    ap.add_argument('--stall-eps', type=float, default=0.4, help='스톨 판정 |dP/dt| [kPa/s]')
    ap.add_argument('--stall-hold', type=float, default=1.0,
                    help='진짜 스톨(dP/dt≈0) 확인 유지 [s]. 압력이 안 오르므로 길어도 안전.')
    ap.add_argument('--limit-hold', type=float, default=0.15,
                    help='상한 도달 시 유지 [s]. 릴리프가 닫힌 채라 짧아야 한다 — '
                         '솔버가 읽을 최소 표본(200 Hz 에서 30행)만.')

    ap.add_argument('--ppos-ceiling', type=float, default=500.0,
                    help='양압 레일 스윕 상한 [kPa **gauge**]')
    ap.add_argument('--pneg-floor-gauge', type=float, default=-95.0,
                    help='음압 레일 스윕 하한 [kPa gauge]')
    ap.add_argument('--trip-margin', type=float, default=40.0,
                    help='트립은 스윕 한계보다 이만큼 밖에 둔다 [kPa]')
    ap.add_argument('--stop-lead', type=float, default=0.12,
                    help='예측 정지 선행 시간 [s]. 밸브 닫힘 꼬리보다 크게.')

    ap.add_argument('--no-zero', action='store_true')
    ap.add_argument('--zero-tol-kpa', type=float, default=8.0)
    ap.add_argument('--vent-seconds', type=float, default=6.0,
                    help='0점 보정 전 대기압 복귀 시간 [s]')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--publish-in-dry-run', action='store_true')
    args = ap.parse_args()

    cfg = args.config
    if cfg is None:
        here = os.path.dirname(os.path.abspath(__file__))
        for cand in (os.path.join(here, '..', 'config', 'powerpack_config.yaml'),
                     os.path.join(here, '..', 'share', 'can_powerpack', 'config',
                                  'powerpack_config.yaml')):
            if os.path.exists(cand):
                cfg = cand
                break
    if cfg is None or not os.path.exists(cfg):
        raise SystemExit('powerpack_config.yaml 을 찾지 못했다 — --config 로 지정할 것')
    calib, ns_key = load_calib(cfg)

    # gauge → abs 로 변환하고 트립을 스윕 한계 밖에 둔다
    args.ppos_ceiling = calib['atm'] + args.ppos_ceiling
    args.pneg_floor = calib['atm'] + args.pneg_floor_gauge
    args.trip_hi = args.ppos_ceiling + args.trip_margin
    args.trip_lo = max(1.0, args.pneg_floor - args.trip_margin)

    outdir = args.out or os.path.join('results_pump', datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(outdir, exist_ok=True)

    print('=' * 74)
    print(f'단계   : {args.phase}')
    print(f'추가부피: {args.extra_volume_ml:.0f} mL → {args.extra_volume_rail} 레일')
    print(f'한계   : 스윕 P⁺≤{args.ppos_ceiling:.1f} / P⁻≥{args.pneg_floor:.1f}, '
          f'트립 {args.trip_hi:.1f} / {args.trip_lo:.1f} kPa abs')
    print(f'출력   : {outdir}')
    print(f'설정   : {cfg} ({ns_key})')
    if args.dry_run and args.publish_in_dry_run:
        print('*** DRY RUN (명령 발행) — virtual_powerpack 리허설용 ***')
    elif args.dry_run:
        print('*** DRY RUN — 밸브 명령을 발행하지 않는다 ***')
    print('안전 상태 = **양 밸브 전개**. 전 밸브 0 은 펌프가 도는 동안 위험하다.')
    print('=' * 74)

    rclpy.init()
    node = RailRecorder(args, calib)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def _sigint(_s, _f):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, _sigint)

    ok = False
    try:
        print('\n센서 수신 대기...')
        t_end = time.time() + 10.0
        while time.time() < t_end and not node._seen:
            time.sleep(0.1)
        if not node._seen:
            raise SystemExit('board/sensors 가 오지 않는다 — can_bridge_node 확인')

        others = []
        try:
            for info in node.get_publishers_info_by_topic(node.topic_pwm):
                if info.node_name != node.get_name():
                    others.append(f'{info.node_namespace}/{info.node_name}')
        except AttributeError:
            print('  (발행자 조회 미지원 — 충돌 검사 생략)')
        if others:
            raise SystemExit(f'board/pwm_cmd 에 다른 발행자가 있다: {others}. '
                             'pp_controller 를 종료하고 다시 시도할 것.')

        if args.no_zero:
            print('0점 보정 생략 — yaml offset 사용')
        else:
            # 펌프가 돌면 레일이 대기압에 도달하지 못한다 (펌프가 계속 당긴다).
            # 리허설 계측: 펌프 ON 상태에서 board 2 가 17.3 kPa 벗어났다.
            print('\n0점 보정은 **펌프를 끈 상태**에서 해야 한다 '
                  '(펌프가 돌면 레일이 대기압에 못 간다).')
            ans = prompt('  펌프를 끄고 Enter (s=0점 보정 건너뛰기, q=종료): ', 'ysq')
            if ans == 'q':
                raise KeyboardInterrupt
            if ans == 's':
                print('  0점 보정 건너뜀 — yaml offset 사용')
            else:
                node.tag.update(pump_on=0)
                print('  대기압 복귀 중 (양 밸브 전개)...')
                node.open_both()
                time.sleep(args.vent_seconds)
                print('  0점 보정 (레일 board 1·2 만 검증)...')
                print(f'    {node.zero_offsets(tol_kpa=args.zero_tol_kpa)}')
                node.tag.update(pump_on=1)

        ok = {'leak': phase_leak, 'map': phase_map,
              'frontier': phase_frontier}[args.phase](node, args)
        if node.tripped:
            print(f'\n!! 트립: {node.tripped}')

    except KeyboardInterrupt:
        print('\n중단 요청 — 안전 상태로 전환한다')
    finally:
        print('\n안전 상태 전환: 양 밸브 전개 3 s → 전부 0')
        try:
            node.open_both()
            time.sleep(3.0)
            node.all_zero()
        except Exception as exc:
            print(f'  경고: 안전 전환 실패 ({exc}) — 밸브와 펌프를 직접 확인할 것')

        tag = f'{args.phase}_{args.extra_volume_rail}{int(args.extra_volume_ml)}'
        path = os.path.join(outdir, f'{tag}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(node.rows)
        meta = dict(phase=args.phase, extra_volume_ml=args.extra_volume_ml,
                    extra_volume_rail=args.extra_volume_rail,
                    u_grid=list(args.u_grid), pneg_targets=list(args.pneg_targets),
                    ppos_ceiling_abs=args.ppos_ceiling, pneg_floor_abs=args.pneg_floor,
                    pulse_s=args.pulse_s, pulse_dp_kpa=args.pulse_dp,
                    log_hz=args.log_hz, atm=calib['atm'], rows=len(node.rows),
                    completed=bool(ok), tripped=node.tripped,
                    config=os.path.abspath(cfg),
                    recorded_at=datetime.now().isoformat(timespec='seconds'))
        with open(os.path.join(outdir, f'{tag}.meta.yaml'), 'w') as f:
            yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)
        print(f'저장: {path}  ({len(node.rows)} 행)')

        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
