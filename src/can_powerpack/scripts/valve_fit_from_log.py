#!/usr/bin/env python3
"""
valve_fit_from_log.py — 실행 로그(schema 3+ CSV)에서 **채널별** 밸브 특성을 뽑는다.

밸브 특성은 반드시 **전류 기준**으로 뽑는다. 스위칭 중에는 로그의 지령 열이
전류와 대응하지 않는다 (HANDOFF S-10).

무엇보다 먼저 **데이터가 쓸 만한지** 검사한다. 공급이 모자란 상태에서는 밸브를
아무리 열어도 챔버가 안 차므로, 그 로그로 피팅하면 밸브를 실제의 수십분의 1 로
잡는다. 그러면 컨트롤러가 전 채널을 100% 로 내게 된다.
  실기 20260828_181748: 탱크가 98 kPa(정상 ~600)로 비어 있었고 충전 중 라인−챔버
  차압이 21~34 kPa 뿐이었다. 그 로그에서 뽑은 면적은 1채널 정상 측정값의
  1/50 이었다.

사용법:
  valve_fit_from_log.py <run.csv> [--min-dp 60] [--force]

  --min-dp   충전 구동 차압(라인−챔버)의 최소 중앙값 [kPa]. 이보다 낮으면 거부.
  --force    검사에 걸려도 그냥 뽑는다 (진단용. 그 결과를 설정에 넣지 말 것)
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

KAPPA = 1.4
PHI_PCR = (2 / (KAPPA + 1)) ** (KAPPA / (KAPPA - 1))
PHI_C = math.sqrt(2 * KAPPA / (KAPPA - 1))
ATM = 101.325
LPM2KGPS, RGAS, TEMPK = 0.0002155, 287.0, 293.15

TANK_OK_KPA = 450.0      # 이보다 낮으면 macro 부스트를 못 쓴다
NOISE_AREA = 1.0e-5      # 압력 잡음이 만드는 면적 바닥


def phi(p_in, p_out):
    if p_in < 1e-9 or p_out >= p_in:
        return 0.0
    pr = min(max(p_out / p_in, 0.0), 1.0)
    if pr <= PHI_PCR:
        return math.sqrt(KAPPA) * (2 / (KAPPA + 1)) ** ((KAPPA + 1) / (2 * (KAPPA - 1)))
    return PHI_C * math.sqrt(max(0.0, pr ** (2 / KAPPA) - pr ** ((KAPPA + 1) / KAPPA)))


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f'빈 파일: {path}')
    return rows, {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def bins(cur, mask, lo0=60, hi0=210, step=15):
    for lo in range(lo0, hi0, step):
        yield lo, lo + step, mask & (cur >= lo) & (cur < lo + step)


def area_series(dp_dt, p_in, p_out, vol_m3, sign):
    q = sign * dp_dt * 1000.0 * vol_m3 / (RGAS * TEMPK) / LPM2KGPS
    ph = np.array([phi(a, b) for a, b in zip(p_in, p_out)])
    ok = ph > 1e-6
    if ok.sum() < 15:
        return None
    return q[ok] / np.maximum(1e-9, p_in[ok] * ph[ok])


def check_supply(A, n_ax, min_dp):
    """공급이 충분했는지 본다. (ok, 메시지 목록)"""
    msgs, ok = [], True
    tank = A['p_macro_pos_kpa']
    if np.median(tank) < TANK_OK_KPA:
        ok = False
        msgs.append(f'탱크 중앙 {np.median(tank):.0f} kPa < {TANK_OK_KPA:.0f} — '
                    f'macro 부스트를 쓸 수 없는 상태였다')
    line = A['p_line_pos_kpa']
    for ax in range(n_ax):
        act = A[f'p_pos_actual_kpa_axis{ax}']
        cur = A[f'cur_mA_pos_bd{ax + 5}_v1micro_axis{ax}']
        on = cur > 100
        if on.sum() < 100:
            continue
        dp = float(np.median(line[on] - act[on]))
        if dp < min_dp:
            ok = False
            msgs.append(f'ch{ax} 충전 구동 차압 중앙 {dp:.0f} kPa < {min_dp:.0f} — '
                        f'유량이 밸브가 아니라 공급으로 제한된다')
    return ok, msgs


def fit_channel(A, ax, vol_m3):
    """한 채널의 세 밸브 특성. {밸브: [(전류mA, 면적, n, dP/dt), ...]}"""
    t = A['time_sec']
    bp, bn = ax + 5, ax + 11
    out = {}

    act = A[f'p_pos_actual_kpa_axis{ax}']
    dP = np.gradient(act, t)
    line = A['p_line_pos_kpa']
    c = {v: A[f'cur_mA_pos_bd{bp}_{v}_axis{ax}'] for v in ('v1micro', 'v2atm', 'v3macro')}

    # 충전: 배기·macro 닫힘 + 레일이 챔버보다 충분히 높음
    base = (c['v2atm'] < 8) & (c['v3macro'] < 8) & (line > act + 25)
    out['pos.micro'] = _collect(c['v1micro'], base, dP, line, act, vol_m3, +1.0)
    # 배기: 챔버 → 대기
    base = (c['v1micro'] < 8) & (c['v3macro'] < 8) & (act > ATM + 12)
    out['pos.atm'] = _collect(c['v2atm'], base, dP, act, np.full_like(act, ATM), vol_m3, -1.0)

    nact = A[f'p_neg_actual_kpa_axis{ax}']
    ndP = np.gradient(nact, t)
    lneg = A['p_line_neg_kpa']
    nc = {v: A[f'cur_mA_neg_bd{bn}_{v}_axis{ax}'] for v in ('v1micro', 'v2atm', 'v3macro')}
    base = (nc['v2atm'] < 8) & (nc['v3macro'] < 8) & (nact > lneg + 15)
    out['neg.micro'] = _collect(nc['v1micro'], base, ndP, nact, lneg, vol_m3, -1.0)
    base = (nc['v1micro'] < 8) & (nc['v3macro'] < 8) & (nact < ATM - 10)
    out['neg.atm'] = _collect(nc['v2atm'], base, ndP, np.full_like(nact, ATM), nact, vol_m3, +1.0)
    return out


def _collect(cur, base, dP, p_in, p_out, vol_m3, sign):
    rows = []
    for lo, hi, m in bins(cur, base):
        if m.sum() < 40:
            continue
        a = area_series(dP[m], p_in[m], p_out[m], vol_m3, sign)
        if a is None:
            continue
        rows.append((0.5 * (lo + hi), max(float(np.median(a)), 0.0),
                     int(m.sum()), float(np.median(dP[m]))))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--min-dp', type=float, default=60.0)
    ap.add_argument('--axes', type=int, default=6)
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()

    _rows, A = load(a.csv)
    n_ax = min(a.axes, 6)
    vol_m3 = float(np.median(A['vol_pos_ml_axis0'][A['vol_pos_ml_axis0'] > 1])) * 1e-6 \
        if (A['vol_pos_ml_axis0'] > 1).any() else 50e-6

    ok, msgs = check_supply(A, n_ax, a.min_dp)
    print(f'파일 {os.path.basename(a.csv)}   챔버 부피 {vol_m3 * 1e6:.1f} mL   '
          f'길이 {A["time_sec"][-1]:.1f}s')
    if msgs:
        print('\n=== 공급 상태 경고 ===')
        for m in msgs:
            print('  ' + m)
    if not ok and not a.force:
        print('\n이 로그는 밸브 피팅에 쓸 수 없다. 공급이 모자란 상태에서는 유량이 밸브가')
        print('아니라 공급으로 제한되므로, 피팅하면 밸브를 실제보다 훨씬 작게 잡는다.')
        print('탱크를 채우고 채널을 하나씩 돌려 다시 받을 것. (--force 로 무시 가능)')
        return 2
    if ok:
        print('  공급 상태 정상 ✓')

    print('\n=== 채널별 밸브 특성 (전류 기준) ===')
    for ax in range(n_ax):
        res = fit_channel(A, ax, vol_m3)
        print(f'\n-- ch{ax} --')
        for role, rows in res.items():
            if not rows:
                print(f'  {role:10s} 표본 부족')
                continue
            hit = [r for r in rows if r[1] > NOISE_AREA]
            head = f'  {role:10s} '
            print(head + '  '.join(f'{r[0]:.0f}mA→{r[1]:.6f}' for r in rows))
            if hit:
                print(' ' * len(head) + f'열리기 시작 ≈ {hit[0][0]:.0f} mA, '
                      f'최대 관측 면적 {max(r[1] for r in rows):.6f}')
    print('\n채널 간 차이가 크면 valve_params_calibrate.py 의 FIT 표를 채널별로 나눠 넣는다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
