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

# ── 실측 상수 ────────────────────────────────────────────────────────────
#
# 지령 → 전류 이득 (실기 3개 실행 · 80 개 정상상태 구간 · R = 0.999):
#       전류[mA] = 2.505 · 지령[%] − 0.6
# 즉 실효 I_MAX = 0.2505 A 다. 모델은 0.3 A 를 가정하고 있었고, 그 20% 차이가
# 모든 것을 어긋나게 했다.
#
# 더 나쁜 것은 이전 보정이 그 잘못된 값으로 **환산**됐다는 점이다. 2026-08-27
# 실기는 8 Hz 로 스위칭하고 있어서 로그의 지령 열이 전류와 대응하지 않았고,
# 그래서 전류로 구간을 나눈 뒤 "지령 = 전류/3.0" 으로 되돌렸다. 실제는 /2.505
# 이므로 곡선 전체가 약 17% 아래로 밀렸다. 그 결과 모델은 크래킹을 41.1% 로
# 봤지만 실제는 49~52% 였고, 실기에서 배기가 49.4% 로 19 초를 버티는데도
# 열리지 않았다 (20260828_152809, 챔버가 141 kPa 에 고정).
#
# 그래서 이제 적합은 **전류(A) 기준**으로만 한다. 지령 환산을 거치지 않으므로
# I_MAX 를 잘못 알아도 곡선이 밀리지 않는다.
IMAX = 0.2505
C_P = 0.00012
PIN_REF = 190.0

# 전류 기준 최소자승 결과 (fit3). 충전은 11점 ±0.8%, 배기는 9점.
# 배기의 원 적합은 k_shape ≈ 1059 (비례대역 1.7%p) 로 사실상 계단이 나왔다.
# 같은 하드웨어이므로 충전과 같은 k_shape 를 쓰고 C_k 로 크래킹만 맞췄다
# (면적 −8.7% 오차를 감수한다). 계단보다 완만한 쪽이 제어 가능하다.
K_SHAPE = 284.5405
FIT = {                      # 역할 → (A_max, C_k, C_k 를 적합한 상류압)
    ('pos', 'micro'): (0.032258, 0.14605948, 190.0),
    ('pos', 'atm'):   (0.042874, 0.15046800, 141.0),
    ('pos', 'macro'): (0.032258 * (0.2845 / 0.58789258), 0.14605948, 190.0),
    ('neg', 'micro'): (0.032258 * (1.778125 / 0.58789258), 0.14605948, 190.0),
    ('neg', 'atm'):   (0.042874 * (1.778125 / 0.58789258), 0.15046800, 141.0),
    ('neg', 'macro'): (0.032258 * (1.778125 / 0.58789258), 0.14605948, 190.0),
}

# 각 밸브가 실제로 겪는 상류압. C_p·Pin 항이 상류압에 비례하므로 C_k 를 그만큼
# 옮겨야 밸브가 자기 동작점에서 적합했을 때와 같은 전류에 열린다.
PIN_ROLE = {
    ('pos', 'micro'): 190.0,   # 양압 레일 → 챔버
    ('pos', 'atm'):   141.0,   # 챔버 → 대기
    ('pos', 'macro'): 800.0,   # 탱크 → 챔버
    ('neg', 'micro'): 101.3,   # 챔버 → 음압 레일
    ('neg', 'atm'):   101.3,   # 대기 → 챔버
    ('neg', 'macro'): 101.3,   # 챔버 → 이젝터
}

# 뻑뻑함 여유폭 [지령 %p]. 위험한 방향은 하나뿐이다 — 모델은 유량이 있다고
# 믿는데 실제로는 0 일 때. 그러면 피드백이 없어 지령이 대역을 한참 지나칠
# 때까지 램프업한다. 자세한 것은 HANDOFF S-8·S-10.
STIFF_MARGIN_PCT = 1.5
STIFF_ROLES = {('pos', 'micro'), ('pos', 'macro'), ('pos', 'atm')}


def _role(ch, valve):
    return ('pos' if ch <= 5 else 'neg', valve)


def role_ck(ch, valve):
    role = _role(ch, valve)
    _amax, C_k_fit, Pin_fit = FIT[role]
    C_k = C_k_fit + C_P * (PIN_ROLE[role] - Pin_fit)
    if role in STIFF_ROLES:
        C_k += STIFF_MARGIN_PCT / 100.0 * IMAX      # 모델을 그만큼 뻑뻑하게
    return C_k


def role_amax(ch, valve):
    return FIT[_role(ch, valve)][0]


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

    print(f"I_MAX = {IMAX} A (실측)   k_shape = {K_SHAPE}   alpha_shape = 1.0")
    print(f"밸브별 (자기 상류압 기준, 충전·배기에 +{STIFF_MARGIN_PCT}%p 여유폭):")
    for role in FIT:
        Pin = PIN_ROLE[role]
        ck = role_ck(0 if role[0] == 'pos' else 6, role[1])
        half = (ck - C_P * Pin) / IMAX * 100
        lo = (ck - C_P * Pin + math.log(0.1 / 0.9) / K_SHAPE) / IMAX * 100
        hi = (ck - C_P * Pin + math.log(0.9 / 0.1) / K_SHAPE) / IMAX * 100
        print(f"   {role[0]}/{role[1]:5s} Pin={Pin:6.1f}  A_max={role_amax(0 if role[0]=='pos' else 6, role[1]):.6f}  "
              f"C_k={ck:.8f}  반개 {half:5.1f}%  대역 {lo:.1f}~{hi:.1f}%")
    print()

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
            m = re.match(r'^(          )(A_max|k_shape|C_k|alpha_shape|I_MAX):\s*(\S+)\s*$', line)
            if m:
                ind, key, cur = m.group(1), m.group(2), float(m.group(3))
                if key == 'k_shape':      val = K_SHAPE
                elif key == 'C_k':        val = role_ck(ch, valve)
                elif key == 'alpha_shape': val = 1.0
                elif key == 'I_MAX':      val = IMAX
                else:                     val = role_amax(ch, valve)
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
