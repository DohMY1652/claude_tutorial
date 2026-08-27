#!/usr/bin/env python3
"""하드코딩 기본 13-parameter 를 밸브별 오리피스 지름에 맞춰 스케일한다.

기본값은 **1.6 mm 오리피스**로 피팅된 세트다. 실기 밸브는 지름이 서로 다르다:

    양압 ch0~5 : v1 fill 2.3 mm, v2 vent 2.3 mm, v3 boost 1.6 mm
    음압 ch6~11: v1 suck 4.0 mm, v2 admit 4.0 mm, v3 eject 4.0 mm

`A_eff = A_max · sigmoid(k_shape·F_net)^alpha_shape` 에서 **A_max 만 완전개방 유효
면적**이고, 나머지 12개(k_shape·C_k·C_p·C_z·Bouc-Wen 3개·alpha_shape·2차 동특성 4개)
는 스풀을 움직이는 솔레노이드 쪽 특성이라 오리피스 지름과 무관하다. 그래서 A_max 만
면적비 (d/1.6)^2 로 곱한다.

  2.3 mm → (2.3/1.6)^2 = 2.066 배
  1.6 mm → 1.000 배 (기준)
  4.0 mm → (4.0/1.6)^2 = 6.250 배

한계: 완전개방에서 오리피스가 아니라 스풀이 병목이면 이 비례가 과대평가된다.
그때는 실측 Cd·eta(피팅 리포트 B절)로 교정해야 한다.
"""
import argparse
import os
import sys
from datetime import datetime

import yaml

NODE_KEY = '/pack2/pp_controller'
D_REF = 1.6                      # 기본값을 피팅한 오리피스 지름 [mm]

# Controller.cpp 의 하드코딩 기본값 (channel_config 로더의 flat() 기본 인자)
BASE = dict(I_MAX=0.30, A_max=0.2845, k_shape=33.09, C_k=0.0288, C_p=0.00012,
            C_z=0.0, A_bw=260649.5, beta_bw=179.0, gamma_bw=0.06,
            alpha_shape=3884.2, wn_up=40.0, zeta_up=1.2, wn_down=45.0,
            zeta_down=1.0)

# (채널군, 역할) → 오리피스 지름 [mm]
ORIFICE = {
    ('pos', 'micro'): 2.3,   ('pos', 'atm'): 2.3,   ('pos', 'macro'): 1.6,
    ('neg', 'micro'): 4.0,   ('neg', 'atm'): 4.0,   ('neg', 'macro'): 4.0,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n-pos', type=int, default=6, help='양압 채널 수 (ch0..)')
    ap.add_argument('--n-total', type=int, default=12, help='전체 채널 수')
    ap.add_argument('--chamber-volume-ml', type=float, default=None,
                    help='주면 chN.chamber_volume_ml 로 넣는다')
    ap.add_argument('--out', default=None, help='기본: config/valve_params.orifice.yaml')
    a = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    out = a.out or os.path.join(here, '..', 'config', 'valve_params.orifice.yaml')
    out = os.path.abspath(out)

    ch = {}
    print(f"{'채널':>6s} {'역할':>6s} {'지름[mm]':>9s} {'면적비':>8s} {'A_max':>10s}")
    for gid in range(a.n_total):
        side = 'pos' if gid < a.n_pos else 'neg'
        entry = {}
        for role in ('micro', 'atm', 'macro'):
            d = ORIFICE[(side, role)]
            ratio = (d / D_REF) ** 2
            q = dict(BASE)
            q['A_max'] = round(BASE['A_max'] * ratio, 8)
            entry[role] = q
            if gid in (0, a.n_pos):          # 대표 채널만 출력
                print(f"  ch{gid:<4d} {role:>6s} {d:9.1f} {ratio:8.3f} {q['A_max']:10.5f}")
        if a.chamber_volume_ml is not None:
            entry['chamber_volume_ml'] = float(a.chamber_volume_ml)
        ch[f'ch{gid}'] = entry

    doc = {NODE_KEY: {'ros__parameters': {'channel_config': ch}}}
    header = (f'# valve_params.orifice.yaml — valve_params_from_orifice.py 생성\n'
              f'# 생성: {datetime.now().isoformat(timespec="seconds")}\n'
              f'#\n'
              f'# 하드코딩 기본값(1.6 mm 오리피스로 피팅)을 밸브별 실제 지름에 맞춰\n'
              f'# A_max 만 면적비 (d/{D_REF})^2 로 스케일한 것이다. 나머지 12개 파라미터는\n'
              f'# 솔레노이드/스풀 특성이라 오리피스와 무관하므로 그대로 둔다.\n'
              f'#   양압 ch0~{a.n_pos-1}: v1 2.3 / v2 2.3 / v3 1.6 mm\n'
              f'#   음압 ch{a.n_pos}~{a.n_total-1}: 전부 4.0 mm\n')
    with open(out, 'w') as f:
        f.write(header)
        yaml.safe_dump(doc, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
    print(f'\n  기록: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
