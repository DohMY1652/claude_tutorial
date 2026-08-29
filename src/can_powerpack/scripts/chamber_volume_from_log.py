#!/usr/bin/env python3
"""
chamber_volume_from_log.py — 실행 로그에서 채널별 **유효 부피**를 추정한다.

부피는 요구 유량에 그대로 곱해진다 (m_dot = P_dot·V/(R·T)). 전 채널을 같은
값으로 두면 실제 부피가 큰 채널은 필요한 유량의 일부만 요구해 느리고, 작은
채널은 과요구해 진동한다. 실기 20260829_142216 에서 채널 간 유효 부피가 수 배
차이났고, 그것이 "느린 채널"과 "진동하는 채널"의 공통 원인이었다.

원리: 충전 밸브만 열린 구간에서
      dP/dt = A_eff(I)·Pin·φ(Pin,Pout)·R·T·lpm2kgps / V
    이므로 같은 전류(=같은 A_eff)·같은 차압에서 dP/dt 가 작으면 V 가 크다.
    A_eff 를 몰라도 채널 **간 비율**은 구할 수 있다.

한계 — 반드시 읽을 것:
  · **채널을 하나씩 돌린 로그**여야 한다. 여러 채널이 같은 레일을 나눠 쓰면
    차압이 흔들려 추정이 흩어진다 (6축 로그에서 전류 창을 좁히면 같은 채널이
    0.15~2.45 배로 튀었다).
  · 공급이 넉넉해야 한다 (충전 중 라인−챔버 차압이 60 kPa 이상 권장).
  · 밸브 면적이 채널마다 같다고 가정한다. 그렇지 않으면 이 값은 "면적/부피"의
    역수이지 순수한 부피가 아니다 — 그래도 컨트롤러 보정에는 그대로 쓸 수 있다.

사용법:
  chamber_volume_from_log.py <run1.csv> [run2.csv ...] [--axes 6] [--ref-ml 50]

여러 로그를 함께 주면(채널별로 하나씩) 각각에서 자기 채널만 뽑아 합친다.
결과를 powerpack_config.yaml 의 channel_config.chN.volume_ml 에 넣는다.
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
LPM2KGPS, RGAS, TEMPK = 0.0002155, 287.0, 293.15
MIN_DP = 25.0        # 이보다 차압이 작으면 orifice 식이 잘 안 맞는다
MIN_N = 40


def phi(p_in, p_out):
    if p_in < 1e-9 or p_out >= p_in:
        return 0.0
    pr = min(max(p_out / p_in, 0.0), 1.0)
    if pr <= PHI_PCR:
        return math.sqrt(KAPPA) * (2 / (KAPPA + 1)) ** ((KAPPA + 1) / (2 * (KAPPA - 1)))
    return PHI_C * math.sqrt(max(0.0, pr ** (2 / KAPPA) - pr ** ((KAPPA + 1) / KAPPA)))


def inv_volume(A, ax, lo, hi):
    """전류 [lo,hi) 구간에서 dP/dt/(Pin·φ) 의 중앙값 ∝ A_eff/V. 표본 부족이면 None."""
    t = A['time_sec']
    act = A[f'p_pos_actual_kpa_axis{ax}']
    line = A['p_line_pos_kpa']
    dP = np.gradient(act, t)
    c1 = A[f'cur_mA_pos_bd{ax + 5}_v1micro_axis{ax}']
    c2 = A[f'cur_mA_pos_bd{ax + 5}_v2atm_axis{ax}']
    c3 = A[f'cur_mA_pos_bd{ax + 5}_v3macro_axis{ax}']
    m = ((c1 >= lo) & (c1 < hi) & (c2 < 8) & (c3 < 8)
         & (line > act + MIN_DP) & (dP > 5))
    if m.sum() < MIN_N:
        return None, int(m.sum())
    ph = np.array([phi(a, b) for a, b in zip(line[m], act[m])])
    ok = ph > 1e-6
    if ok.sum() < MIN_N // 2:
        return None, int(ok.sum())
    return float(np.median(dP[m][ok] / (line[m][ok] * ph[ok]))), int(ok.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+')
    ap.add_argument('--axes', type=int, default=6)
    ap.add_argument('--ref-ml', type=float, default=50.0,
                    help='기준 채널(가장 표본이 많은 창)의 부피 [mL]')
    a = ap.parse_args()

    # 채널별로 여러 전류 창에서 값을 모은다
    per_ch = {ax: [] for ax in range(a.axes)}
    for path in a.csv:
        rows = list(csv.DictReader(open(path)))
        if not rows:
            continue
        A = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
        for ax in range(a.axes):
            for lo in range(110, 180, 10):
                v, n = inv_volume(A, ax, lo, lo + 10)
                if v is not None and v > 0:
                    per_ch[ax].append((lo, v, n, os.path.basename(path)))

    print(f"{'축':>3s} {'창 수':>6s} {'표본합':>7s} {'상대 A/V':>10s} {'흩어짐':>8s}")
    inv = {}
    for ax in range(a.axes):
        rows = per_ch[ax]
        if not rows:
            print(f"{ax:3d} {0:6d} {0:7d}   표본 부족")
            continue
        vals = np.array([r[1] for r in rows])
        ntot = sum(r[2] for r in rows)
        spread = float(vals.max() / vals.min()) if vals.min() > 0 else float('inf')
        inv[ax] = float(np.median(vals))
        print(f"{ax:3d} {len(rows):6d} {ntot:7d} {inv[ax]:10.4f} {spread:7.1f}x")

    if not inv:
        return 2
    base = min(inv.values())          # A/V 가 가장 큰 = 부피가 가장 작은 채널
    print()
    print("추정 부피 [mL] — 가장 작은 채널을 --ref-ml 로 잡는다:")
    out = {}
    for ax, v in sorted(inv.items()):
        out[ax] = a.ref_ml * (base / v) if v > 0 else float('nan')
    small = min(out, key=lambda k: out[k])
    scale = a.ref_ml / out[small]
    for ax in sorted(out):
        print(f"  ch{ax}: {out[ax] * scale:7.1f} mL")

    print()
    print("powerpack_config.yaml 의 channel_config 에 넣을 형태:")
    for ax in sorted(out):
        print(f"      ch{ax}:")
        print(f"        volume_ml: {out[ax] * scale:.1f}")

    worst = max((np.array([r[1] for r in per_ch[ax]]).max()
                 / np.array([r[1] for r in per_ch[ax]]).min())
                for ax in inv if len(per_ch[ax]) > 1) if any(
                    len(per_ch[ax]) > 1 for ax in inv) else 1.0
    if worst > 3.0:
        print()
        print(f"⚠ 창 간 흩어짐이 최대 {worst:.1f} 배다. 여러 채널을 같이 돌린 로그이거나")
        print("  공급이 흔들린 로그일 수 있다. 채널을 하나씩 돌려 다시 받을 것.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
