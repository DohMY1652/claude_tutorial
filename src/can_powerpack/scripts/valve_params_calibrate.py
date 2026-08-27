#!/usr/bin/env python3
"""
valve_params_calibrate.py — 실기 로그로 밸브 곡선을 재보정한다.

배경 (2026-08-27 실기 20260827_195422):
  기존 파라미터는 alpha_shape = 3884.2 로 시그모이드에 3884승이 붙어 밸브가
  사실상 **계단**이 되어 있었다. 그 결과 역모델의 비례대역이 지령 64.5~68.0%
  단 3.5%p 밖에 안 됐고, MPPI 탐색 노이즈(sigma 8%)가 그 창보다 2.3배 넓어
  창 안에 착지할 수가 없었다. 명령은 0↔100 을 8.3 Hz 로 왕복했고 챔버는
  peak-to-peak 96 kPa 로 진동했다.

  같은 로그에서 유효면적을 역산하면 (배기 닫힘·라인압>챔버+20kPa 구간):
      v1 micro : 42.5% → 0.0248,  47.5% → 0.0319,  52.5% → 0.0325
      v2 atm   :  2.5% → 0.0     ,  42.5% → 0.0303,  47.5% → 0.0436,
                 52.5% → 0.0427,  57.5% → 0.0416
  즉 20 mA 아래는 완전히 닫혀 있고 ~47% 에서 이미 포화한다. 모델이 그 구간에서
  예측한 면적은 0.0000 (언더플로) 이었다 — 실기와 모델이 무관했다.

보정 방침:
  alpha_shape 를 1 로 되돌리고 (k_shape, C_k) 로 비례대역을 다시 세운다.
  두 점으로 고정한다:
      20 mA(6.7%)  에서 sigma = 0.002   → 사실상 닫힘
      55%          에서 sigma = 0.95    → 거의 전개
  결과 비례대역은 27.9~51.1% (23.2%p) 로 기존의 6.6배다.

  면적 절대값(A_max)은 챔버 부피 가정에 비례한다. 그러나 컨트롤러는 요구 유량을
  **같은 부피**로 계산하므로 (Controller.cpp: m_dot_pressure = P_dot·V/…),
  부피 오차는 폐루프에서 상쇄된다. 절대 LPM 값이 아니라 비율이 중요하다.

사용법:
  valve_params_calibrate.py --check      현재값과 보정값 비교만
  valve_params_calibrate.py --apply      config/valve_params.yaml 갱신
"""
import argparse
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
YAML = os.path.normpath(os.path.join(HERE, '..', 'config', 'valve_params.yaml'))

IMAX = 0.3
C_P = 0.00012
PIN_REF = 190.0          # 보정에 쓴 라인압 [kPa] — 실기 로그의 중앙값

# C_k 는 밸브마다 **자기 상류압**으로 잡아야 한다.
# F_net = I + C_z·z + C_p·Pin − C_k 에서 C_p·Pin 항이 상류압에 비례하는데,
# macro 는 상류가 탱크(≈800 kPa) 라 C_p·Pin = 0.096 으로 C_k 의 68% 를 먹는다.
# 전 밸브에 같은 C_k 를 쓰면 macro 는 지령 15% 에서 이미 반개가 되고, 시뮬에서
# MPPI 가 macro 를 4~8% 만 열어도 챔버가 199 kPa 까지 치솟아 과압 세이프티가
# 배기를 100% 로 래치했다 — 3 Hz 리밋사이클의 원인이었다.
# 각 밸브가 **자기 동작점에서** 39.5% 에 반개가 되도록 C_k 를 따로 잡는다.
PIN_ROLE = {
    ('pos', 'micro'): 190.0,   # 양압 레일 → 챔버
    ('pos', 'atm'):   130.0,   # 챔버 → 대기
    ('pos', 'macro'): 800.0,   # 탱크 → 챔버
    ('neg', 'micro'): 101.3,   # 챔버 → 음압 레일
    ('neg', 'atm'):   101.3,   # 대기 → 챔버
    ('neg', 'macro'): 101.3,   # 챔버 → 이젝터
}

# 비례대역을 정하는 두 점
U_CLOSED, S_CLOSED = 6.7, 0.002
U_OPEN,   S_OPEN   = 55.0, 0.95

# 실측 포화 유효면적 (양압 ch0 계측). 나머지는 기존 비율을 그대로 유지한다.
A_POS_MICRO = 0.033
A_POS_ATM   = 0.044


def solve_shape(Pin=PIN_REF):
    kl = math.log(S_CLOSED / (1 - S_CLOSED))
    kh = math.log(S_OPEN / (1 - S_OPEN))
    k = (kh - kl) / (IMAX / 100.0 * (U_OPEN - U_CLOSED))
    C_k = U_OPEN / 100.0 * IMAX + C_P * Pin - kh / k
    return k, C_k


def role_ck(ch, valve):
    return solve_shape(PIN_ROLE[('pos' if ch <= 5 else 'neg', valve)])[1]


def band(k, C_k):
    lo = (C_k - C_P * PIN_REF + math.log(0.10 / 0.90) / k) / IMAX * 100
    hi = (C_k - C_P * PIN_REF + math.log(0.90 / 0.10) / k) / IMAX * 100
    return lo, hi


def new_amax(ch, valve, cur):
    """기존 A_max 를 실측 스케일로 옮긴다. 채널·밸브 간 비율은 보존한다."""
    pos = ch <= 5
    if pos:
        if valve == 'micro':
            return A_POS_MICRO
        if valve == 'atm':
            return A_POS_ATM
        # macro 는 1.6 mm 오리피스 — 기존 macro/micro 비율을 그대로 옮긴다
        return A_POS_MICRO * (cur / 0.58789258)
    # 음압 채널은 직접 계측이 없다. 양압 micro 의 보정 배율을 그대로 적용한다.
    return cur * (A_POS_MICRO / 0.58789258)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    if not (a.apply or a.check):
        print(__doc__); return 0

    k, C_k = solve_shape()
    lo, hi = band(k, C_k)
    print(f"k_shape = {k:.6f}   C_k = {C_k:.8f}   alpha_shape = 1.0")
    print(f"반개 {(C_k - C_P*PIN_REF)/IMAX*100:.1f}%   "
          f"비례대역 {lo:.1f}~{hi:.1f}% ({hi-lo:.1f}%p)   "
          f"[기존 모델은 64.5~68.0%, 3.5%p]")
    print("밸브별 C_k (자기 상류압 기준, 반개 39.5% 로 통일):")
    for (sgn, v), Pin in PIN_ROLE.items():
        print(f"   {sgn}/{v:5s} Pin={Pin:6.1f} kPa → C_k = {solve_shape(Pin)[1]:.8f}")

    src = open(YAML, encoding='utf-8').read()
    out = []
    ch = valve = None
    changed = 0
    for line in src.split('\n'):
        m = re.match(r'^      (ch\d+):\s*$', line)
        if m:
            ch = int(m.group(1)[2:]); valve = None
        m = re.match(r'^        (micro|atm|macro):\s*$', line)
        if m:
            valve = m.group(1)
        if ch is not None and valve is not None:
            m = re.match(r'^(          )(A_max|k_shape|C_k|alpha_shape):\s*(\S+)\s*$', line)
            if m:
                ind, key, cur = m.group(1), m.group(2), float(m.group(3))
                if key == 'k_shape':      val = k
                elif key == 'C_k':        val = role_ck(ch, valve)
                elif key == 'alpha_shape': val = 1.0
                else:                     val = new_amax(ch, valve, cur)
                if abs(val - cur) > 1e-12:
                    changed += 1
                    if a.check and ch in (0, 6) and key in ('A_max', 'alpha_shape'):
                        print(f"   ch{ch}/{valve}/{key}: {cur} → {val:.8f}")
                line = f"{ind}{key}: {val:.8f}"
        out.append(line)

    print(f"\n바뀌는 항목 {changed}개")
    if a.apply:
        open(YAML, 'w', encoding='utf-8').write('\n'.join(out))
        print(f"기록: {YAML}")
    else:
        print("(--apply 를 붙여야 실제로 쓴다)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
