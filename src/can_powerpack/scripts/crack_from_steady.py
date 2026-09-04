#!/usr/bin/env python3
"""crack_from_steady.py — 로그에서 **실제 밸브 크래킹 지령**을 잰다 (정상 지령 기준)

  python3 crack_from_steady.py ~/result/<ts>/<ts>.csv [axis]

한쪽 밸브만 열려 있고 구동 차압이 충분한 표본만 골라, **지령 1 % 구간마다**
dP/dt 중앙값을 낸다. 유량이 시작되는 지령이 곧 크래킹이다.

crack_from_log.py 와 다른 점 — 그쪽은 "밸브가 켜지는 버스트"의 지령을 보므로
**진동에 오염된다** (세게 떠는 채널일수록 지령이 빨리 올라가 높게 잡힌다).
이 스크립트는 지령을 **구간별로 모아 중앙값**을 보므로 그 편향이 없다.
20260904 에 이 방법으로 atm 실 크래킹이 57 % 인데 모델이 46 % 인 것을 찾았다
(그 11 %p 차이가 하강 시 2.3 초 무반응의 원인이었다).

한계: 밸브가 세게 흐르면 팔이 움직여 부피가 변하고, 그 변화가 dP 에 섞인다.
그래서 **문턱 근처**(막 흐르기 시작하는 구간)가 가장 믿을 만하다. 완전 개방
구간의 절대 유량은 이 방법으로 못 잰다.

더 큰 한계 — 구간별 방식은 **챔버가 지령보다 늦다**는 것을 못 본다. 지령이
60 % 에서 내려오는 도중의 잔여 유량이 52 % 칸에 들어가, 낮은 지령이 실제보다
흐르는 것처럼 보인다. 그래서 아래 **정지창(steady window)** 표를 같이 낸다:
지령이 창 내내 ±1 %p 안에 머문 구간만 쓰므로 그 오염이 없다. 둘이 어긋나면
정지창을, 그중에서도 **가장 긴 창**을 믿을 것.

읽는 법 — 모델의 u_crack 은 `C_p·Pin` 항 때문에 **상류압에 따라 움직인다**.
micro 의 Pin 은 라인압, atm 의 Pin 은 챔버압이다. 그래서 이 스크립트가 낸
지령 %를 모델과 비교할 때는 반드시 **그 창의 실제 Pin** 에서 u_crack 을
계산해야 한다. 20260904 에 레일을 351 kPa 로 가정해 micro 가 10 %p 어긋난
것처럼 잘못 읽었다 — 실제 라인압은 139 kPa 였고 모델은 맞았다.
"""
import csv
import statistics
import sys

POS_BD = [5, 6, 7, 8, 9, 10]
NEG_BD = [11, 12, 13, 14, 15, 16]
DT = 0.01


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    ax = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    rd = csv.reader(open(sys.argv[1]))
    hdr = next(rd)
    C = {k: list(v) for k, v in
         zip(hdr, zip(*[[float(x) for x in r] for r in rd if r]))}
    n = len(C['time_sec'])
    bd, nb = POS_BD[ax], NEG_BD[ax]
    P = C[f'p_pos_actual_kpa_axis{ax}']
    LP = C['p_line_pos_kpa']
    ang = C[f'angle_deg_axis{ax}']
    mi = C[f'pwm_pct_pos_bd{bd}_v1micro_axis{ax}']
    at = C[f'pwm_pct_pos_bd{bd}_v2atm_axis{ax}']
    ma = C[f'pwm_pct_pos_bd{bd}_v3macro_axis{ax}']

    d = [0.0] * n
    for i in range(3, n - 3):
        d[i] = (P[i + 3] - P[i - 3]) / (6 * DT)

    def scan(sel, sgn, lbl):
        b = {}
        for i in range(5, n - 5):
            ok, cmd = sel(i)
            if ok:
                b.setdefault(int(cmd), []).append(sgn * d[i])
        print(f'\n  {lbl}')
        print('    지령%   표본   dP/dt 중앙값 [kPa/s]')
        # 표본이 적은 구간은 지령이 **스쳐 지나간** 순간들이다. 그때의 dP/dt 는
        # 이 밸브가 아니라 직전까지 흐르던 잔여 유량을 재는 것이라 부호까지 뒤집힌다.
        # 실제로 지령이 **머문** 구간만 남긴다 — 최대 표본의 10 % 를 기준으로.
        # (이 필터가 없어서 20260904 에 21 %/28 % 로 잘못 잡혔다.)
        if not b:
            print('    (표본 없음)')
            return None
        keep = max(30, int(0.10 * max(len(v) for v in b.values())))
        rows = [(k, len(b[k]), statistics.median(b[k]))
                for k in sorted(b) if len(b[k]) >= keep]
        print(f'    (지령이 머문 구간만: 표본 {keep} 개 이상)')
        # 크래킹을 넘으면 지령이 커질수록 유량은 **계속** 늘어난다 — 다시 0 으로
        # 돌아가지 않는다. 그래서 "여기서부터 끝까지 전부 문턱 초과" 인 첫 구간을
        # 찾는다. 한 구간이 우연히 튀는 것(뒤에서 음수로 돌아감)은 자동으로 걸러진다.
        NEED, THR = 3, 3.0
        first = None
        for i in range(len(rows) - NEED + 1):
            if all(r[2] > THR for r in rows[i:]):
                first = rows[i][0]
                break
        for k, cnt, m in rows:
            mark = '   ← 유량 시작' if k == first else ''
            print(f'    {k:>4}  {cnt:>6}   {m:8.2f}{mark}')
        if first:
            print(f'    → 실 크래킹 ≈ **{first} %**  '
                  f'(여기서부터 끝까지 {THR:.0f} kPa/s 초과 유지)')
        else:
            print(f'    → 유량 시작을 못 찾았다 (최소 {NEED} 구간 필요). '
                  f'그 밸브가 충분히 열린 표본이 없다는 뜻이다.')
        return first

    scan(lambda i: (mi[i] < 2 and ma[i] < 2 and at[i] >= 1 and P[i] > 108, at[i]),
         -1, f'axis{ax} atm 배기 (micro/macro 닫힘, 챔버 >108 kPa)')
    scan(lambda i: (at[i] < 2 and ma[i] < 2 and mi[i] >= 1 and LP[i] - P[i] > 20, mi[i]),
         +1, f'axis{ax} micro 충전 (atm/macro 닫힘, 라인−챔버 차압 >20 kPa)')
    report_windows(C, C['time_sec'], DT, [ax])
    print('\n  C_k 환산: Δu_crack[%p] × I_MAX/100 = ΔC_k   (I_MAX 0.2505 → ×0.002505)')
    print('  단, 비교 대상 u_crack 은 **그 창의 Pin** 에서 계산할 것 (모듈 주석 참고).')
    return 0


# ============================================================================
# 정지창(steady window) — 지령이 실제로 **멈춰 있던** 구간만 본다.
# 구간별 방식의 챔버 지연 오염이 없다. 창이 길수록 신뢰도가 높다.
# ============================================================================
WIN_MIN_S = 0.30      # 이보다 짧은 창은 안 쓴다
SETTLE_S = 0.15       # 창 앞부분은 직전 유량의 잔재라 버린다


def steady_windows(C, t, dt, ax, vname, other, sgn):
    bd = POS_BD[ax]
    a = C[f'pwm_pct_pos_bd{bd}_{vname}_axis{ax}']
    o = C[f'pwm_pct_pos_bd{bd}_{other}_axis{ax}']
    mac = C[f'pwm_pct_pos_bd{bd}_v3macro_axis{ax}']
    P = C[f'p_pos_actual_kpa_axis{ax}']
    line = C.get('p_line_pos_kpa', [0.0] * len(t))
    nwin, nset = max(3, int(WIN_MIN_S / dt)), int(SETTLE_S / dt)
    out, i = [], 0
    while i < len(t) - nwin:
        if o[i] > 2.0 or mac[i] > 2.0 or a[i] < 2.0:
            i += 1
            continue
        j = i
        while (j < len(t) - 1 and o[j] <= 2.0 and mac[j] <= 2.0
               and max(a[i:j + 1]) - min(a[i:j + 1]) <= 2.0):
            j += 1
        if j - i >= nwin + nset:
            s = i + nset
            dur = t[j - 1] - t[s]
            # micro 의 상류는 라인, atm 의 상류는 챔버다
            pin = (statistics.fmean(line[s:j]) if vname == 'v1micro'
                   else statistics.fmean(P[s:j]))
            out.append((statistics.fmean(a[s:j]), dur,
                        (P[j - 1] - P[s]) / dur * sgn, ax, t[s], pin))
        i = max(j, i + 1)
    return out


def report_windows(C, t, dt, axes):
    for vname, other, sgn, lbl in (('v1micro', 'v2atm', +1, 'micro 충전'),
                                   ('v2atm', 'v1micro', -1, 'atm 배기')):
        allw = []
        for ax in axes:
            allw += steady_windows(C, t, dt, ax, vname, other, sgn)
        print(f'\n  ══ {lbl} — 지령 정지창 (≥{WIN_MIN_S:.2f} s, ±1 %p) ══')
        if not allw:
            print('    (지령이 멈춰 있던 창이 없다 — 제어기가 계속 뱅뱅했다는 뜻)')
            continue
        print('     axis     t      길이     지령%    dP/dt      Pin')
        for u, dur, r, ax, t0, pin in sorted(allw, key=lambda w: -w[1])[:15]:
            print(f'      {ax}   {t0:7.2f}  {dur:6.2f}s   {u:6.1f}  {r:8.2f}  {pin:7.1f}')
        print('    → 가장 긴 창에서 dP/dt 가 누설 수준(±2 kPa/s)이면 그 지령은 **닫힘**이다.')
        print('      닫힘으로 확인된 최대 지령과, 유량이 확인된 최소 지령 사이가 크래킹이다.')


if __name__ == '__main__':
    sys.exit(main())
