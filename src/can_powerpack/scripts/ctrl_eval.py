#!/usr/bin/env python3
"""ctrl_eval.py — 제어 성능 평가 (qp vs mppi A/B)

MPPI.md 의 성능 표를 만든 하네스다. 가상 하드웨어만 쓰고 실기가 필요 없다.

사용:
  ros2 launch 는 이 스크립트가 직접 띄운다. 미리 띄워 두지 말 것.
    python3 scripts/ctrl_eval.py qp   --tag qp
    python3 scripts/ctrl_eval.py mppi --tag mppi
    python3 scripts/ctrl_eval.py mppi --tag t10 \
            --overrides MPC_parameters.mppi_ref_tau_s=0.10

**단일 스텝으로 비교하지 말 것.** 이 하네스는 비결정론적이다(README 0절) —
스텝 1회는 잡음 큰 표본 1개이고, 유사 설정의 IAE 가 19.5~29.2 로 흩어졌다.
그래서 기본 프로파일이 스텝 4개이고 전이별 지표를 평균한다. 그렇게 하면
QP 재실행에서 IAE 13.0/14.4, 오버슈트 2.16/2.11 수준으로 재현된다.

위치 + **압력 추종**을 함께 잰다.

MPPI 가 직접 최적화하는 것은 챔버 압력의 P_ref 추종이다. 위치 지표만 보면
"MPPI 가 나아졌는데 위치는 상위 계층이 지배해서 안 변한 것" 과
"MPPI 가 나빠진 것" 을 구별할 수 없다. 그래서 둘을 같이 잰다.

지표:
  위치  IAE  — 스텝 이후 |각도−목표| 의 시간적분 [deg·s]. 오버슈트/정착보다
              분산이 작아 튜닝 신호로 쓸 수 있다.
  위치  오버슈트 / 정착(±0.5°) / 최종오차
  압력  RMSE — |P − P_ref| 의 RMS, 채널 12개 평균 [kPa]
  압력  최종 — 마지막 1 s 평균 |P − P_ref| [kPa]
"""
import argparse
import os
import signal
import socket
import struct
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

N_AXES = 6
N_CH = 12
BOARD_OFF = 5          # channel_board_offset → sensors_kpa[gid + BOARD_OFF - 1]
SETTLE_BAND = 0.5


class Rec(Node):
    def __init__(self):
        super().__init__('ctrl_eval_rec')
        self.ta, self.ang = [], []
        self.tp, self.pm, self.pr = [], [], []
        self.create_subscription(Float64MultiArray, '/pack2/board/analog', self._a, 50)
        self.create_subscription(Float64MultiArray, '/pack2/controller/sensors_kpa', self._s, 50)
        self.create_subscription(Float64MultiArray, '/pack2/controller/mpc_refs_kpa', self._r, 50)
        self.t0 = time.time()
        self._last_ref = None

    def _a(self, m):
        self.ta.append(time.time() - self.t0)
        self.ang.append(list(m.data[:N_AXES]))

    def _s(self, m):
        if self._last_ref is None:
            return
        d = list(m.data)
        if len(d) < N_CH + BOARD_OFF - 1:
            return
        self.tp.append(time.time() - self.t0)
        self.pm.append([d[g + BOARD_OFF - 1] for g in range(N_CH)])
        self.pr.append(self._last_ref)

    def _r(self, m):
        if len(m.data) >= N_CH:
            self._last_ref = list(m.data[:N_CH])


def send_target(port, deg):
    s = socket.create_connection(('127.0.0.1', port), timeout=5.0)
    s.sendall(struct.pack('<' + 'd' * N_AXES, *([float(deg)] * N_AXES)))
    time.sleep(0.3)
    s.close()


def pos_metrics(t, a, target, t_step, y0=None):
    """y0 = 스텝 직전 각도. 하강 스텝에서 오버슈트를 방향에 맞게 재기 위해 필요하다
    (max−target 로 재면 45→15 스텝의 '오버슈트' 가 시작값 30° 로 잡힌다)."""
    t, a = np.asarray(t), np.asarray(a)
    m = t >= t_step
    tt, aa = t[m] - t_step, a[m]
    if tt.size < 10:
        return dict(iae=float('nan'), over=float('nan'), settle=float('nan'), fin=float('nan'))
    iae, over, settle, fin = [], [], [], []
    for j in range(aa.shape[1]):
        y = aa[:, j]
        iae.append(float(np.trapz(np.abs(y - target), tt)))
        start = y[0] if y0 is None else y0[j]
        sgn = 1.0 if target >= start else -1.0
        peak = np.max(y) if sgn > 0 else np.min(y)
        over.append(max(0.0, float((peak - target) * sgn)))
        bad = np.where(np.abs(y - target) > SETTLE_BAND)[0]
        settle.append(float(tt[bad[-1]]) if bad.size and bad[-1] + 1 < tt.size
                      else (float('nan') if bad.size else 0.0))
        fin.append(abs(float(y[-1] - target)))
    return dict(iae=float(np.mean(iae)), over=float(np.mean(over)),
                settle=float(np.nanmean(settle)), fin=float(np.mean(fin)))


def prs_metrics(t, pm, pr, t_step):
    t = np.asarray(t)
    if t.size < 10:
        return dict(rmse=float('nan'), fin=float('nan'))
    pm, pr = np.asarray(pm), np.asarray(pr)
    m = t >= t_step
    e = pm[m] - pr[m]
    tt = t[m]
    rmse = float(np.sqrt(np.mean(e ** 2)))
    lastm = tt >= tt[-1] - 1.0
    fin = float(np.mean(np.abs(e[lastm]))) if lastm.any() else float('nan')
    return dict(rmse=rmse, fin=fin)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('solver', choices=['qp', 'mppi'])
    # 한 번 실행에 전이를 여러 개 넣어 평균한다. 하네스가 비결정론적이라
    # 스텝 1회 = 잡음 큰 표본 1개이고, 유사 설정의 IAE 가 19.5~29.2 로 흩어졌다.
    ap.add_argument('--profile', default='45,15,40,25')
    ap.add_argument('--dwell', type=float, default=5.0)
    ap.add_argument('--warmup', type=float, default=8.0)
    ap.add_argument('--port', type=int, default=2293)
    ap.add_argument('--log', default=None)
    ap.add_argument('--overrides', default='')
    ap.add_argument('--tag', default='')
    args = ap.parse_args()

    log = open(args.log, 'w') if args.log else subprocess.DEVNULL
    cmd = ['ros2', 'launch', 'can_powerpack', 'virtual.launch.py',
           'control_mode:=2', f'solver:={args.solver}']
    if args.overrides:
        cmd.append(f'overrides:={args.overrides}')
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                            env=dict(os.environ), preexec_fn=os.setsid)
    try:
        rclpy.init()
        rec = Rec()
        tw = time.time() + args.warmup
        while time.time() < tw:
            rclpy.spin_once(rec, timeout_sec=0.05)

        targets = [float(x) for x in args.profile.split(',')]
        marks = []
        for tg in targets:
            send_target(args.port, tg)
            marks.append((time.time() - rec.t0, tg))
            te = time.time() + args.dwell
            while time.time() < te:
                rclpy.spin_once(rec, timeout_sec=0.02)

        per = []
        for i, (ts, tg) in enumerate(marks):
            te = marks[i + 1][0] if i + 1 < len(marks) else 1e18
            t = np.asarray(rec.ta)
            m = (t >= ts) & (t < te)
            if m.sum() < 20:
                continue
            idx = np.where(m)[0]
            y0 = rec.ang[max(0, idx[0] - 1)]
            per.append(pos_metrics(list(t[m]), [rec.ang[k] for k in idx], tg, ts, y0))
        if not per:
            print(f'RESULT\t{args.tag or args.solver}\tFAILED')
            return 1
        agg = {k: float(np.nanmean([p[k] for p in per])) for k in per[0]}
        pr_ = prs_metrics(rec.tp, rec.pm, rec.pr, marks[0][0])
        tag = args.tag or args.solver
        print(f'RESULT\t{tag}\tIAE={agg["iae"]:.3f}\tover={agg["over"]:.2f}\t'
              f'settle={agg["settle"]:.2f}\tfin={agg["fin"]:.3f}\t'
              f'pRMSE={pr_["rmse"]:.2f}\tpFin={pr_["fin"]:.2f}\tn={len(per)}')
        rec.destroy_node()
        rclpy.shutdown()
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=6)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        if args.log:
            log.close()


if __name__ == '__main__':
    sys.exit(main())
