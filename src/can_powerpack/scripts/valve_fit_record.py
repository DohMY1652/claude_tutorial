#!/usr/bin/env python3
"""
valve_fit_record.py — 밸브 특성 실험 구동 + 동기 기록

한 번 실행 = **(모드 1개) × (채널 목록)**. 36개를 한 번에 하지 않는다 — 외부 압력 인가 부품과
고정부피 탱크를 채널마다 사람이 옮겨야 하므로 채널 전환마다 프롬프트에서 멈춘다.

  모드          채널          외부 압력 인가        피팅되는 쌍
  pos_micro    양압 gid 0~5   board 1 (예 250 kPa)  v1(충전) + v2(배기)
  pos_macro    양압 gid 0~5   board 3 (예 700 kPa)  v3(부스트) + v2(배기)
  neg_micro    음압 gid 6~11  board 2 (예 30 kPa)   v1(흡입) + v2(유입)
  neg_macro    음압 gid 6~11  board 4 (외부 진공)   v3(이젝트) + v2(유입)

v2 는 두 모드에 모두 나오므로 독립 2회 피팅 → 교차검증이 된다.

**전제 조건**
  · pp_controller 를 띄우지 말 것. 같은 board/pwm_cmd 에 발행하고 중재가 없어서 500 Hz 로 덮어쓴다
    (valve_operate:false 도 소용없다 — 그 경우 전부 0을 발행한다). 시작 시 자동 검사한다.
  · can_bridge_node 만 기동:
      ros2 run can_powerpack can_bridge_node --ros-args --namespace pack2 \
          --params-file <powerpack_config.yaml>
  · 라인압은 외부 레귤레이터로 사람이 유지한다. 이 스크립트는 **감시만** 하고 제어하지 않는다.
  · 이젝터는 쓰지 않는다. board 4 라인에 외부 진공을 직접 인가하고 MacroSwitch 는 닫아 둔다.

**안전** — pp_controller 가 없으므로 과압 보호와 종료 처리가 전부 이 스크립트 책임이다.
  · 임계 초과 시 해당 채널 중립밸브(v2) 전개 + 대상밸브 폐쇄 후 중단.
  · 종료(정상·예외·Ctrl-C) 시 전 채널 v2 를 열어 대기압으로 되돌린 뒤 전부 0.
    "전부 0" 은 양압 채널에서 압력이 갇히는 상태라 안전 상태가 아니다.
  · CanBridge 에는 워치독이 없다 — 마지막 명령이 250 Hz 로 영구히 나간다.

사용 예:
  python3 valve_fit_record.py --mode pos_micro --line-kpa 250
  python3 valve_fit_record.py --mode pos_micro --line-kpa 250 --gids 0 --extra-volume-ml 100
  python3 valve_fit_record.py --mode neg_macro --line-kpa 30 --dry-run
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, UInt16MultiArray

import yaml

PWM_BOARDS = 16                 # CanBridge 가 처리하는 보드 수 (board/pwm_cmd 유효 길이 = 48)
PWM_PER_BOARD = 3
PWM_LEN = PWM_BOARDS * PWM_PER_BOARD
PCT_TO_PWM = 4095.0 / 100.0

ATM_KPA_DEFAULT = 101.325

# 라인 압력 센서 보드
BOARD_LINE_POS, BOARD_LINE_NEG, BOARD_MACRO, BOARD_MACRO_NEG = 1, 2, 3, 4
MACRO_SWITCH_IDX = (4 - 1) * PWM_PER_BOARD + 0      # board 4 v1 = 9

# 밸브 이름 → 보드 내 인덱스 (Controller 의 zoh_ 기록 순서와 동일)
VALVE_IDX = {'v1': 0, 'v2': 1, 'v3': 2}

# ── 모드 정의 ──────────────────────────────────────────────────────────────
#   target  : 외부 인가 라인과 챔버 사이의 밸브 (이번에 특성을 재는 주 대상)
#   neutral : 챔버를 대기압으로 되돌리는 밸브. 항상 v2 다.
#   sign    : target 이 챔버압을 올리면 +1, 내리면 -1
MODES = {
    'pos_micro': dict(gids=list(range(0, 6)), target='v1', neutral='v2',
                      line_board=BOARD_LINE_POS, sign=+1,
                      desc='양압 채널 micro(레일→챔버) + atm(챔버→대기)'),
    'pos_macro': dict(gids=list(range(0, 6)), target='v3', neutral='v2',
                      line_board=BOARD_MACRO, sign=+1,
                      desc='양압 채널 macro(탱크→챔버) + atm(챔버→대기)'),
    'neg_micro': dict(gids=list(range(6, 12)), target='v1', neutral='v2',
                      line_board=BOARD_LINE_NEG, sign=-1,
                      desc='음압 채널 micro(챔버→음압레일) + atm(대기→챔버)'),
    'neg_macro': dict(gids=list(range(6, 12)), target='v3', neutral='v2',
                      line_board=BOARD_MACRO_NEG, sign=-1,
                      desc='음압 채널 macro(챔버→외부진공) + atm(대기→챔버)'),
}

# 크래킹 구간(모델상 상류압에 따라 48~62%)을 촘촘히 뜨고, **그 위쪽도 충분히** 뜬다.
# 자기검증 결과: 크래킹 아래 레벨은 유량이 0 이라 정보가 없다. 위쪽 레벨 수가 부족하면
# (A_max, k_shape, C_k, alpha_shape) 의 평평한 다양체가 좁혀지지 않는다.
DEFAULT_LEVELS = [0, 40, 48, 52, 55, 58, 60, 62, 65, 68, 72, 76, 80, 85, 90, 95, 100]

# 세 밸브의 지령·실측 전류를 **항상 전부** 기록한다.
# 태그된 밸브만 기록하면 반대 밸브로 챔버를 초기화하는 구간에서 다른 밸브의 전류가
# 섞여 들어가고, 히스테리시스 상태(z)가 오염돼 피팅이 망가진다.
# 상/하류압은 (모드, 밸브, 챔버압, 라인압) 에서 유일하게 결정되므로 저장하지 않는다.
CSV_HEADER = [
    't', 'mode', 'gid', 'phase', 'valve', 'level_pct', 'sweep_dir',
    'u_v1', 'u_v2', 'u_v3', 'I_v1', 'I_v2', 'I_v3',
    'P_ch_abs', 'P_line_pos', 'P_line_neg', 'P_macro', 'P_macro_neg', 'P_atm',
    'extra_vol_ml',
]


class Recorder(Node):
    def __init__(self, args, calib, mode):
        super().__init__('valve_fit_record')
        self.args = args
        self.calib = calib          # {board_id: (offset_mv, gain)}
        self.mode = mode
        self.atm = calib['atm']

        ns = args.ns.strip('/')
        pre = f'/{ns}/' if ns else '/'
        self.pub = self.create_publisher(UInt16MultiArray, pre + 'board/pwm_cmd', 10)
        self.create_subscription(UInt16MultiArray, pre + 'board/sensors', self._on_sensors, 20)
        self.create_subscription(Float64MultiArray, pre + 'board/currents', self._on_currents, 20)

        self._lock = threading.Lock()
        self._pwm = [0] * PWM_LEN
        self._sens_mv = [0.0] * PWM_BOARDS
        self._cur_mv = [0.0] * PWM_LEN
        self._sens_seen = False
        self._cur_seen = False

        # 0점 보정 후 사용할 offset (yaml 값에서 시작해 zero() 가 갱신)
        self._offset = {b: calib[b][0] for b in calib if isinstance(b, int)}

        # 기록 상태 (시퀀스 스레드가 갱신, 타이머가 읽는다)
        self.tag = dict(gid=-1, phase='idle', valve='-', level=0.0, sweep='-')
        self.rows = []
        self.t0 = time.time()
        self.tripped = None

        self.create_timer(1.0 / args.log_hz, self._tick)

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def _on_sensors(self, msg):
        with self._lock:
            n = min(len(msg.data), PWM_BOARDS)
            self._sens_mv[:n] = [float(v) for v in msg.data[:n]]
            self._sens_seen = True

    def _on_currents(self, msg):
        with self._lock:
            n = min(len(msg.data), PWM_LEN)
            self._cur_mv[:n] = [float(v) for v in msg.data[:n]]
            self._cur_seen = True

    # ── 압력 / 전류 변환 ──────────────────────────────────────────────────
    def kpa(self, board_id):
        """kPa_abs = (mV − offset)·gain + atm  (Controller 의 SensorCalib 와 동일)"""
        with self._lock:
            mv = self._sens_mv[board_id - 1]
        off = self._offset.get(board_id, 0.0)
        gain = self.calib.get(board_id, (0.0, 0.0))[1]
        return (mv - off) * gain + self.atm

    def current_a(self, board_id, valve):
        """board/currents 는 mV. mA = mV/10 → A = mV/10000."""
        idx = (board_id - 1) * PWM_PER_BOARD + VALVE_IDX[valve]
        with self._lock:
            return self._cur_mv[idx] / 10000.0

    # ── 밸브 명령 ─────────────────────────────────────────────────────────
    def set_valve(self, board_id, valve, pct):
        idx = (board_id - 1) * PWM_PER_BOARD + VALVE_IDX[valve]
        with self._lock:
            self._pwm[idx] = int(round(max(0.0, min(100.0, pct)) * PCT_TO_PWM))
        self.flush()

    def close_channel(self, board_id):
        with self._lock:
            base = (board_id - 1) * PWM_PER_BOARD
            self._pwm[base:base + 3] = [0, 0, 0]
        self.flush()

    def all_zero(self):
        with self._lock:
            self._pwm = [0] * PWM_LEN
        self.flush()

    def flush(self):
        """48개를 매번 전부 발행한다 — 대상 외 밸브가 명시적으로 0 으로 유지되도록."""
        if self.args.dry_run and not self.args.publish_in_dry_run:
            return
        msg = UInt16MultiArray()
        with self._lock:
            msg.data = list(self._pwm)
        self.pub.publish(msg)

    def safe_state(self, gids):
        """안전 상태 = 전 대상 채널의 v2 개방(대기 방향). '전부 0' 은 압력이 갇혀 안전하지 않다."""
        with self._lock:
            self._pwm = [0] * PWM_LEN
            for gid in gids:
                base = (gid + self.args.board_offset - 1) * PWM_PER_BOARD
                self._pwm[base + VALVE_IDX['v2']] = 4095
        self.flush()

    # ── 기록 타이머 (+ 과압 트립) ─────────────────────────────────────────
    def _tick(self):
        if not (self._sens_seen and self._cur_seen):
            return
        gid = self.tag['gid']
        if gid < 0:
            return

        board = gid + self.args.board_offset
        p_ch = self.kpa(board)
        m = self.mode

        # 과압/과진공 트립
        if self.tripped is None:
            if m['sign'] > 0 and p_ch > self.args.trip_hi_kpa:
                self._trip(board, f'채널 압력 {p_ch:.1f} > {self.args.trip_hi_kpa} kPa')
            elif m['sign'] < 0 and p_ch < self.args.trip_lo_kpa:
                self._trip(board, f'채널 압력 {p_ch:.1f} < {self.args.trip_lo_kpa} kPa')

        base = (board - 1) * PWM_PER_BOARD
        with self._lock:
            u = [self._pwm[base + i] / PCT_TO_PWM for i in range(3)]
            cur = [self._cur_mv[base + i] / 10000.0 for i in range(3)]

        self.rows.append([
            round(time.time() - self.t0, 5), self.args.mode, gid,
            self.tag['phase'], self.tag['valve'], self.tag['level'], self.tag['sweep'],
            round(u[0], 3), round(u[1], 3), round(u[2], 3),
            round(cur[0], 6), round(cur[1], 6), round(cur[2], 6),
            round(p_ch, 4),
            round(self.kpa(BOARD_LINE_POS), 4), round(self.kpa(BOARD_LINE_NEG), 4),
            round(self.kpa(BOARD_MACRO), 4), round(self.kpa(BOARD_MACRO_NEG), 4),
            self.atm, self.args.extra_volume_ml,
        ])

    def _trip(self, board, why):
        self.tripped = why
        with self._lock:
            base = (board - 1) * PWM_PER_BOARD
            self._pwm[base + VALVE_IDX['v1']] = 0
            self._pwm[base + VALVE_IDX['v3']] = 0
            self._pwm[base + VALVE_IDX['v2']] = 4095
        self.flush()
        self.get_logger().error(f'[TRIP] {why} — 중립밸브 전개, 중단')

    # ── 유틸 ──────────────────────────────────────────────────────────────
    def zero_offsets(self, boards, seconds=0.5):
        """전 채널 대기 개방 상태에서 offset 재취득 (Controller 의 기동 0점 보정과 동일)."""
        acc = {b: [] for b in boards}
        t_end = time.time() + seconds
        while time.time() < t_end:
            with self._lock:
                for b in boards:
                    acc[b].append(self._sens_mv[b - 1])
            time.sleep(0.005)
        for b in boards:
            if acc[b]:
                self._offset[b] = sum(acc[b]) / len(acc[b])
        return {b: round(self._offset[b], 1) for b in boards}

    def line_stable(self, board, target_kpa, seconds, tol):
        """라인압 감시 — 제어하지 않는다. 지정값 근처에서 안정한지만 판정."""
        vals = []
        t_end = time.time() + seconds
        while time.time() < t_end:
            vals.append(self.kpa(board))
            time.sleep(0.01)
        mean = sum(vals) / len(vals)
        spread = max(vals) - min(vals)
        ok = abs(mean - target_kpa) <= tol and spread <= tol
        return ok, mean, spread


# ══════════════════════════════════════════════════════════════════════════
# 스윕 시퀀스
# ══════════════════════════════════════════════════════════════════════════
def wait_settle(node, board, timeout, eps_kpa_s, stable_for=0.2, poll=0.01):
    """|dP/dt| 가 eps 이하로 stable_for 초 유지되거나 timeout 까지 대기."""
    t_end = time.time() + timeout
    prev_p, prev_t = node.kpa(board), time.time()
    calm_since = None
    while time.time() < t_end:
        if node.tripped:
            return False
        time.sleep(poll)
        now, p = time.time(), node.kpa(board)
        dt = now - prev_t
        if dt <= 0:
            continue
        rate = abs(p - prev_p) / dt
        prev_p, prev_t = p, now
        if rate <= eps_kpa_s:
            calm_since = calm_since or now
            if now - calm_since >= stable_for:
                return True
        else:
            calm_since = None
    return True


def to_neutral(node, gid, args, mode):
    """챔버를 대기압 쪽으로 되돌린다 (v2 전개)."""
    board = gid + args.board_offset
    node.tag.update(phase='reset', valve=mode['neutral'], level=100.0, sweep='-')
    node.set_valve(board, mode['target'], 0.0)
    node.set_valve(board, mode['neutral'], 100.0)
    wait_settle(node, board, args.reset_timeout, args.settle_eps)
    node.set_valve(board, mode['neutral'], 0.0)
    time.sleep(0.15)


def charge_full(node, gid, args, mode):
    """대상 밸브를 100% 로 열어 챔버를 라인압 쪽으로 채운다 (중립밸브 스윕 준비)."""
    board = gid + args.board_offset
    node.tag.update(phase='charge', valve=mode['target'], level=100.0, sweep='-')
    node.set_valve(board, mode['neutral'], 0.0)
    node.set_valve(board, mode['target'], 100.0)
    wait_settle(node, board, args.charge_timeout, args.settle_eps)
    node.set_valve(board, mode['target'], 0.0)
    time.sleep(0.15)


def sweep_valve(node, gid, args, mode, valve, prep):
    """한 밸브의 상승/하강 스윕. prep 는 각 레벨 전 챔버 초기화 함수."""
    board = gid + args.board_offset
    levels = list(args.levels)
    passes = [('up', levels), ('down', list(reversed(levels))[1:])]

    for direction, seq in passes:
        for lv in seq:
            if node.tripped:
                return False
            prep(node, gid, args, mode)
            if node.tripped:
                return False
            node.tag.update(phase='sweep', valve=valve, level=float(lv), sweep=direction)
            node.set_valve(board, valve, float(lv))
            wait_settle(node, board, args.level_hold, args.settle_eps)
            node.set_valve(board, valve, 0.0)
            time.sleep(0.1)
    return True


def run_channel(node, gid, args, mode):
    print(f'\n  ── gid {gid} (board {gid + args.board_offset}) 시작 ──')
    # Phase A: 대상 밸브 (라인 ↔ 챔버)
    print(f'     Phase A: {mode["target"]} 스윕 ({len(args.levels)*2-1} 레벨)')
    if not sweep_valve(node, gid, args, mode, mode['target'], to_neutral):
        return False
    # Phase B: 중립 밸브 (챔버 ↔ 대기)
    print(f'     Phase B: {mode["neutral"]} 스윕')
    if not sweep_valve(node, gid, args, mode, mode['neutral'], charge_full):
        return False
    to_neutral(node, gid, args, mode)
    node.close_channel(gid + args.board_offset)
    return True


def prompt(msg, choices='ysrq'):
    while True:
        ans = input(msg).strip().lower()
        if ans == '':
            return 'y'
        if ans in choices:
            return ans
        print(f'    (선택: {"/".join(choices)} 또는 Enter)')


# ══════════════════════════════════════════════════════════════════════════
def load_calib(path):
    with open(path) as f:
        doc = yaml.safe_load(f) or {}
    for key, node in doc.items():
        params = (node or {}).get('ros__parameters', {})
        sc = params.get('Sensor_calibration')
        if sc:
            out = {'atm': float(sc.get('atm_offset', ATM_KPA_DEFAULT))}
            for bid, ch in (sc.get('boards') or {}).items():
                out[int(bid)] = (float(ch['offset']), float(ch['gain']))
            return out, key
    raise SystemExit(f'{path} 에서 Sensor_calibration 을 찾지 못했다')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', required=True, choices=sorted(MODES))
    ap.add_argument('--line-kpa', type=float, required=True,
                    help='외부 레귤레이터로 인가한 라인 압력 [kPa abs]. 감시용 기준값.')
    ap.add_argument('--gids', type=int, nargs='*', default=None,
                    help='기본은 모드의 전 채널. 하나만 하려면 --gids 0')
    ap.add_argument('--extra-volume-ml', type=float, default=0.0,
                    help='이중 부피법 2회차에서 티로 추가한 알려진 부피 [mL]')
    ap.add_argument('--config', default=None, help='powerpack_config.yaml 경로')
    ap.add_argument('--ns', default='pack2')
    ap.add_argument('--out', default=None, help='출력 디렉터리 (기본 ./results_fit/<timestamp>)')
    ap.add_argument('--board-offset', type=int, default=5, help='channel_board_offset')
    ap.add_argument('--levels', type=float, nargs='*', default=DEFAULT_LEVELS)
    ap.add_argument('--log-hz', type=float, default=200.0)
    ap.add_argument('--level-hold', type=float, default=3.0, help='레벨당 최대 유지 [s]')
    ap.add_argument('--reset-timeout', type=float, default=4.0)
    ap.add_argument('--charge-timeout', type=float, default=6.0)
    ap.add_argument('--settle-eps', type=float, default=2.0, help='정착 판정 |dP/dt| [kPa/s]')
    ap.add_argument('--trip-hi-kpa', type=float, default=190.0)
    ap.add_argument('--trip-lo-kpa', type=float, default=20.0)
    ap.add_argument('--line-tol-kpa', type=float, default=10.0)
    ap.add_argument('--dry-run', action='store_true',
                    help='밸브 명령을 발행하지 않고 시퀀스·기록·안전 경로만 확인')
    ap.add_argument('--publish-in-dry-run', action='store_true',
                    help='dry-run 이지만 명령은 발행 (virtual_powerpack 검증용)')
    ap.add_argument('--skip-line-check', action='store_true')
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

    mode = MODES[args.mode]
    gids = args.gids if args.gids else mode['gids']
    bad = [g for g in gids if g not in mode['gids']]
    if bad:
        raise SystemExit(f'모드 {args.mode} 의 채널이 아니다: {bad} (허용 {mode["gids"]})')

    outdir = args.out or os.path.join('results_fit', datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(outdir, exist_ok=True)

    print('=' * 72)
    print(f'모드   : {args.mode} — {mode["desc"]}')
    print(f'채널   : {gids}   (board {[g + args.board_offset for g in gids]})')
    print(f'인가   : board {mode["line_board"]} @ {args.line_kpa:.1f} kPa abs (외부 레귤레이터, 수동)')
    print(f'대상   : {mode["target"]} + {mode["neutral"]}    추가 부피 {args.extra_volume_ml:.0f} mL')
    print(f'출력   : {outdir}')
    print(f'설정   : {cfg}  (네임스페이스 키 {ns_key})')
    if args.dry_run:
        print('*** DRY RUN — 밸브 명령을 발행하지 않는다 ***')
    print('=' * 72)

    rclpy.init()
    node = Recorder(args, calib, mode)
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    stop = {'flag': False}

    def _sigint(_s, _f):
        stop['flag'] = True
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _sigint)

    completed = []
    try:
        print('\n센서 수신 대기...')
        t_end = time.time() + 10.0
        while time.time() < t_end and not (node._sens_seen and node._cur_seen):
            time.sleep(0.1)
        if not node._sens_seen:
            raise SystemExit('board/sensors 가 오지 않는다 — can_bridge_node 가 떠 있는지 확인')
        if not node._cur_seen:
            raise SystemExit('board/currents 가 오지 않는다 — 전류 실측이 모델 입력이라 필수다')

        # pp_controller 충돌 검사
        try:
            npub = node.count_publishers(f'/{args.ns}/board/pwm_cmd')
            if npub > 0:
                raise SystemExit(
                    f'board/pwm_cmd 에 이미 발행자가 {npub}개 있다 (pp_controller?). '
                    '중재가 없어 명령이 덮어써진다 — 종료하고 다시 시도할 것.')
        except AttributeError:
            print('  (count_publishers 미지원 — 충돌 검사 생략)')

        boards = [gid + args.board_offset for gid in gids] + \
                 [BOARD_LINE_POS, BOARD_LINE_NEG, BOARD_MACRO, BOARD_MACRO_NEG]
        print('0점 보정 (전 채널 대기 개방 상태여야 한다)...')
        print(f'  offset = {node.zero_offsets(sorted(set(boards)))}')

        for i, gid in enumerate(gids):
            board = gid + args.board_offset
            print('\n' + '─' * 72)
            print(f'[{i+1}/{len(gids)}] gid {gid} → board {board}')
            print(f'  1. 외부 압력 인가 부품을 board {mode["line_board"]} 라인에 연결 '
                  f'({args.line_kpa:.0f} kPa abs)')
            print(f'  2. 고정부피 탱크를 board {board} 채널로 옮길 것')
            if args.extra_volume_ml > 0:
                print(f'  3. 추가 부피 {args.extra_volume_ml:.0f} mL 용기를 티로 연결 (이중 부피법 2회차)')
            ans = prompt('  준비되면 Enter (s=건너뛰기, q=종료): ', 'ysq')
            if ans == 'q':
                break
            if ans == 's':
                print('  건너뜀')
                continue

            if not args.skip_line_check:
                ok, mean, spread = node.line_stable(
                    mode['line_board'], args.line_kpa, 1.0, args.line_tol_kpa)
                print(f'  라인압 {mean:.1f} kPa (변동 {spread:.2f}) → {"OK" if ok else "불안정"}')
                if not ok:
                    a2 = prompt('  그래도 진행? (Enter=yes, s=건너뛰기, q=종료): ', 'ysq')
                    if a2 == 'q':
                        break
                    if a2 == 's':
                        continue

            while True:
                node.tripped = None
                node.tag.update(gid=gid)
                t_start = time.time()
                ok = run_channel(node, gid, args, mode)
                node.tag.update(gid=-1, phase='idle', valve='-')
                dur = time.time() - t_start
                if node.tripped:
                    print(f'  !! 트립: {node.tripped}')
                elif ok:
                    print(f'  완료 ({dur:.0f} s, {len(node.rows)} 행 누적)')
                    completed.append(gid)
                a3 = prompt('  다음으로? (Enter=진행, r=이 채널 재실행, q=종료): ', 'yrq')
                if a3 == 'r':
                    continue
                break
            if a3 == 'q':
                break

    except KeyboardInterrupt:
        print('\n중단 요청 — 안전 상태로 전환한다')
    finally:
        print('\n안전 상태 전환: 전 채널 중립밸브 개방 → 2 s 후 전부 폐쇄')
        try:
            node.safe_state(gids)
            time.sleep(2.0)
            node.all_zero()
        except Exception as exc:
            print(f'  경고: 안전 상태 전환 실패 ({exc}) — 밸브를 직접 확인할 것')

        path = os.path.join(outdir, f'{args.mode}_vol{int(args.extra_volume_ml)}.csv')
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(node.rows)
        meta = dict(mode=args.mode, gids_requested=gids, gids_completed=completed,
                    line_kpa=args.line_kpa, line_board=mode['line_board'],
                    target=mode['target'], neutral=mode['neutral'], sign=mode['sign'],
                    extra_volume_ml=args.extra_volume_ml, board_offset=args.board_offset,
                    levels=list(args.levels), log_hz=args.log_hz,
                    config=os.path.abspath(cfg), rows=len(node.rows),
                    recorded_at=datetime.now().isoformat(timespec='seconds'))
        with open(os.path.join(outdir, f'{args.mode}_vol{int(args.extra_volume_ml)}.meta.yaml'),
                  'w') as f:
            yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)
        print(f'저장: {path}  ({len(node.rows)} 행, 완료 채널 {completed})')

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
