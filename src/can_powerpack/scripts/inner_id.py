#!/usr/bin/env python3
"""inner_id.py — 로그에서 **내층(압력 루프)** 을 동정한다.

  python3 inner_id.py ~/result/<ts>/<ts>.csv [axis]

외층(위치)을 튜닝하려면 내층이 얼마나 느린지를 숫자로 알아야 한다. 이 값
없이 게인을 고르면 안 된다 — 20260904 에 두 번 틀렸고 한 번은 폭주했다
(7e6f636 은 적분을 위상 계산에 안 넣었고, 그 앞은 대역 자체를 안 봤다).

내층을 `P_actual/P_ref = e^{-L·s}/(1+T·s)` 로 보고 (T, L) 을 격자 탐색한다.
그리고 그 (T, L) 로 외층 PID 의 **이득·위상 여유를 적분까지 넣어** 계산한다.

같이 내는 것:
  · 압력 스펙트럼의 6~8 Hz 성분 — cmd_lpf_hz 를 올릴 때 밸브 공진(ζ≈0.2,
    ωn 6.3~7.2 Hz)이 되살아나는지 보는 감시 지표다. 실기에서 이 공진이
    챔버를 p-p 210 kPa 로 흔든 적이 있다.
  · 팔이 정지한 구간의 압력 σ 와 편향 — 내층 자체의 조용함. 이게 작으면
    진동의 근원은 내층이 아니라 **외층이 흔드는 지령**이다.
"""
import csv
import math
import statistics
import sys

D2R = math.pi / 180.0


def load(path):
    rd = csv.reader(open(path))
    hdr = next(rd)
    cols = zip(*[[float(x) for x in r] for r in rd if r])
    return {k: list(v) for k, v in zip(hdr, cols)}


def fit_first_order(t, u, y, idx, dt):
    """e^{-L s}/(1+T s) 를 격자 탐색. (T[ms], L[ms], RMS) 를 돌려준다."""
    best = None
    for T_ms in range(10, 405, 5):
        a = 1.0 - math.exp(-dt / (T_ms / 1000.0))
        for L_ms in range(0, 205, 5):
            k = int(round((L_ms / 1000.0) / dt))
            v = y[idx[0]]
            se = 0.0
            for i in idx:
                v += a * (u[max(0, i - k)] - v)
                se += (v - y[i]) ** 2
            r = math.sqrt(se / len(idx))
            if best is None or r < best[2]:
                best = (T_ms, L_ms, r)
    return best


def band_power(sig, dt, f_lo, f_hi):
    """[f_lo, f_hi] 대역의 RMS. numpy 없이 Goertzel 합으로 낸다."""
    n = len(sig)
    m = statistics.fmean(sig)
    x = [v - m for v in sig]
    df = 1.0 / (n * dt)
    tot = 0.0
    k0, k1 = max(1, int(f_lo / df)), min(n // 2, int(f_hi / df) + 1)
    for k in range(k0, k1):
        w = 2.0 * math.pi * k / n
        c, s = 0.0, 0.0
        for i, v in enumerate(x):
            c += v * math.cos(w * i)
            s += v * math.sin(w * i)
        tot += (c * c + s * s) * 2.0 / (n * n)
    return math.sqrt(tot)


def margins(kp, ki, kd, T, L, J):
    """L(s) = (kp + ki/s + kd·s)·e^{-Ls}/((1+Ts)·J·s²) 의 여유.

    게인은 deg 단위(N·m/deg 계열)로 받아 rad 로 바꾼다. 위상은 언랩 문제를
    피하려고 해석식으로 쓴다 — cmath.phase 로 하면 −400° 가 +320° 로 감긴다.
    """
    def mag(w):
        return (math.hypot(kp / D2R, (kd / D2R) * w - (ki / D2R) / w)
                / (math.hypot(1.0, w * T) * J * w * w))

    def ph(w):
        return (math.degrees(math.atan2((kd / D2R) * w - (ki / D2R) / w, kp / D2R))
                - 180.0 - math.degrees(math.atan(w * T)) - math.degrees(w * L))

    lo, hi = 1e-3, 1e3
    for _ in range(300):
        w = 0.5 * (lo + hi)
        if mag(w) > 1.0:
            lo = w
        else:
            hi = w
    wc = 0.5 * (lo + hi)
    pm = 180.0 + ph(wc)
    wp, w = None, 0.05
    while w < 1e3:
        if ph(w) <= -180.0:
            wp = w
            break
        w *= 1.002
    gm = -20.0 * math.log10(mag(wp)) if wp else None
    return wc, pm, gm


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    ax = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    C = load(sys.argv[1])
    t = C['time_sec']
    n = len(t)
    dt = statistics.fmean([t[i + 1] - t[i] for i in range(min(300, n - 1))])
    pr = C[f'p_pos_ref_kpa_axis{ax}']
    pa = C[f'p_pos_actual_kpa_axis{ax}']
    ang = C[f'angle_deg_axis{ax}']
    vel = C.get(f'vel_filt_dps_axis{ax}')
    if vel is None:                       # schema 5 이하
        vel = [0.0] + [(ang[i] - ang[i - 1]) / dt for i in range(1, n)]

    print(f'  {sys.argv[1]}')
    print(f'  {t[-1]:.1f} s, dt {dt*1000:.1f} ms, axis{ax}')

    # ── 내층이 조용한가 (팔이 멈춘 구간) ──
    still = [i for i in range(n) if abs(vel[i]) < 2.0]
    if len(still) > 50:
        e = [pa[i] - pr[i] for i in still]
        print(f'\n  ── 팔 정지 {len(still)*dt:5.1f} s ──')
        print(f'     P 편향 {statistics.fmean(e):+6.2f} kPa,  σ {statistics.pstdev(e):5.2f}')
        print(f'     P 실측 σ {statistics.pstdev([pa[i] for i in still]):5.2f},  '
              f'지령 σ {statistics.pstdev([pr[i] for i in still]):5.2f} kPa')
        print('     → 지령 σ 가 실측 σ 만큼 크면 진동의 근원은 **외층**이다.')

    # ── 전달함수 동정 ── 마지막 절반(정착 후)을 쓴다
    idx = list(range(n // 2, n))
    T_ms, L_ms, rms = fit_first_order(t, pr, pa, idx, dt)
    sd = statistics.pstdev([pa[i] for i in idx])
    print(f'\n  ── 내층 전달함수  P_ref → P_actual ──')
    print(f'     T = {T_ms} ms,  무시간 L = {L_ms} ms   '
          f'(RMS {rms:.2f} kPa, 분산의 {100*(1-rms/max(sd,1e-9)):.0f} % 설명)')
    print(f'     대역폭 ≈ 1/T = {1000.0/T_ms:.2f} rad/s = {1000.0/T_ms/2/math.pi:.2f} Hz')

    # ── 밸브 공진 감시 ──
    seg = [pa[i] for i in idx]
    p68 = band_power(seg, dt, 6.0, 8.0)
    p02 = band_power(seg, dt, 0.2, 2.0)
    print(f'\n  ── 압력 스펙트럼 (밸브 공진 감시) ──')
    print(f'     6~8 Hz RMS {p68:6.3f} kPa   0.2~2 Hz RMS {p02:6.3f} kPa   '
          f'비 {p68/max(p02,1e-9):.3f}')
    print('     → cmd_lpf_hz 를 올린 뒤 6~8 Hz 가 커지면 밸브 공진(ζ≈0.2)이 되살아난 것이다.')

    # ── 이 내층에서 외층 게인이 안정한가 ──
    J = 2.0 * 0.15 * 0.15
    T, L = T_ms / 1000.0, L_ms / 1000.0
    print(f'\n  ── 이 내층에서 외층 PID 의 여유 (J {J:.4f} kg·m², **적분 포함**) ──')
    print('        kp      ki      kd    │  ωc rad/s   위상여유   이득여유   판정')
    for kp, ki, kd in ((0.0786, 0.20, 0.0200), (0.0400, 0.10, 0.0100),
                       (0.0200, 0.04, 0.0060), (0.0100, 0.02, 0.0040),
                       (0.0050, 0.01, 0.0030)):
        wc, pm, gm = margins(kp, ki, kd, T, L, J)
        v = '불안정' if pm <= 0 else ('위험' if pm < 25 else
                                    ('양호' if pm < 70 else '과감쇠'))
        gs = f'{gm:+6.1f} dB' if gm is not None else '     ---'
        print(f'     {kp:.4f}  {ki:.3f}  {kd:.4f}  │  {wc:6.2f}   {pm:+8.1f}°  {gs}   {v}')
    print('\n     PI 코너 ki/kp 는 ωc/5 이하여야 적분이 교차 근처 위상을 안 먹는다.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
