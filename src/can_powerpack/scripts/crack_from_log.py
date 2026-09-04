#!/usr/bin/env python3
"""crack_from_log.py — 폐루프 로그에서 밸브 크래킹 지령을 추정한다.

  python3 crack_from_log.py ~/result/<ts>/<ts>.csv

크래킹 지령을 여러 검출 문턱에서 재고 dP/dt → 0 으로 외삽한다.

단일 문턱은 항상 **상한**이다 (검출 문턱만큼 + 밸브 지연만큼 늦게 잡힌다).
문턱을 여러 개 두고 (문턱, 지령) 을 직선으로 맞춰 문턱 0 으로 외삽하면
그 두 편향이 같이 제거된다.

**한계 — 반드시 알고 쓸 것.** 이 추정은 **진동 자체에 오염된다.** 세게 진동하는
채널일수록 지령이 빠르게 올라가므로, 검출 문턱을 넘을 때의 지령이 높게 잡힌다.
그래서 **채널 사이 비교에는 쓰면 안 된다** (20260904: 가장 세게 진동한 ch4 가
가장 높은 41 % 로, 가장 조용한 ch2 가 가장 낮은 14 % 로 나왔다 — 기전과 반대다).

쓸 수 있는 것은 **집계 결론** 하나다: 전 채널이 모델보다 낮게 나오면 모델의
u_crack 이 확실히 과대평가된 것이다 (편향이 전부 같은 방향이므로).
채널별 참값이 필요하면 개루프 스윕(RUNBOOK 1절 valve_fit_record.py)을 쓸 것.
"""
import csv
import statistics
import sys

rd = csv.reader(open(sys.argv[1]))
hdr = next(rd)
C = {k: list(v) for k, v in zip(hdr, zip(*[[float(x) for x in r] for r in rd if r]))}
t = C['time_sec']
DT = 0.010
POS_BD = [5, 6, 7, 8, 9, 10]
NEG_BD = [11, 12, 13, 14, 15, 16]
THRESHOLDS = [4.0, 8.0, 15.0, 25.0, 40.0]


def dpdt(P):
    d = [0.0] * len(P)
    for i in range(2, len(P) - 2):
        d[i] = (P[i + 2] - P[i - 2]) / (4 * DT)
    return d


def bursts(act, oth, mac, d, sgn):
    """각 버스트에 대해 (지령궤적, dP/dt궤적) 을 돌려준다."""
    out = []
    i = 3
    while i < len(t) - 5:
        if not (act[i - 1] < 2.0 and act[i] >= 2.0):
            i += 1
            continue
        if d[i] * sgn > 2.0:
            i += 1
            continue
        j = i
        us, rs = [], []
        while j < len(t) - 3 and oth[j] < 2.0 and mac[j] < 2.0 and act[j] >= 1.0:
            us.append(act[j]); rs.append(d[j] * sgn)
            j += 1
            if len(us) > 80:
                break
        if len(us) >= 5:
            out.append((us, rs))
        i = max(j, i + 1)
    return out


def crack_at(bs, thr):
    """문턱 thr 을 처음 넘을 때까지의 최대 지령의 중앙값."""
    v = []
    for us, rs in bs:
        for k in range(len(rs)):
            if rs[k] > thr:
                v.append(max(us[:k + 1]))
                break
    return (statistics.median(v), len(v)) if len(v) >= 8 else (None, len(v))


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-12:
        return my, 0.0
    b = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / sxx
    return my - b * mx, b        # 절편(=문턱 0 외삽), 기울기


print('  크래킹 지령 [%] — 검출 문턱별 측정 + 문턱 0 외삽')
print()
res = {}
for side, BD, pre in (('양압', POS_BD, 'pos'),):
    for vname, vcol, ocol, sgn, mdl in (
            ('micro', 'v1micro', 'v2atm', +1, 51.0),
            ('atm', 'v2atm', 'v1micro', -1, 55.5)):
        print(f'  ── {side} {vname} (모델 u_crack {mdl:.1f} %) ──')
        head = '     axis  ' + ''.join(f'{th:>7.0f}' for th in THRESHOLDS)
        print(head + '   외삽(u_crack)   모델대비')
        for ax in range(6):
            act = C[f'pwm_pct_{pre}_bd{BD[ax]}_{vcol}_axis{ax}']
            oth = C[f'pwm_pct_{pre}_bd{BD[ax]}_{ocol}_axis{ax}']
            mac = C[f'pwm_pct_{pre}_bd{BD[ax]}_v3macro_axis{ax}']
            d = dpdt(C[f'p_{pre}_actual_kpa_axis{ax}'])
            bs = bursts(act, oth, mac, d, sgn)
            xs, ys, cells = [], [], ''
            for th in THRESHOLDS:
                m, n = crack_at(bs, th)
                cells += f'{m:7.1f}' if m is not None else '    ---'
                if m is not None:
                    xs.append(th); ys.append(m)
            if len(xs) >= 3:
                icpt, slope = linfit(xs, ys)
                res[(pre, vname, ax)] = icpt
                print(f'      {ax}  {cells}   {icpt:11.1f}   {icpt - mdl:+9.1f}')
            else:
                print(f'      {ax}  {cells}      (부족)')
        print()

print('  → 외삽값이 검출 문턱과 밸브 지연을 뺀 **실제 크래킹 지령** 추정이다.')
print()
IMAX = 0.2505
print('  C_k 보정량:  ΔC_k = Δu_crack × I_MAX/100 = Δu × 0.002505')
for (pre, vname, ax), v in sorted(res.items()):
    mdl = 51.0 if vname == 'micro' else 55.5
    print(f'    {pre} {vname} ch{ax}:  Δu {v - mdl:+6.1f} %p  →  ΔC_k {(v - mdl) * IMAX / 100:+.5f}')
