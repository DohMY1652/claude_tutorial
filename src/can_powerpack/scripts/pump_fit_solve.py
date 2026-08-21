#!/usr/bin/env python3
"""
pump_fit_solve.py — 기록된 레일 압력에서 펌프 파라미터를 피팅하고 config 로 내보낸다

  Step 1  Phase L: 지수 감쇠 시상수 → 이중 부피법으로 레일 부피 V⁺,V⁻ + 누설 k⁺,k⁻
  Step 2  Phase M: 펄스 구간 초기 dP/dt → 레일별 ṁ_pump + **질량보존 교차검증**
  Step 3  Phase F: 스톨점 → 측정 능력경계 (안전 상한 도달분은 '하한 경계'로 취급)
  Step 4  펌프 기하 피팅. 능력경계에 큰 가중치 — 컨트롤러가 쓰는 유일한 출력이므로
  Step 5  pump_params.yaml + pump_report.md + 플롯

사용:
  # 3단계 CSV 를 한 디렉터리에 모아 두고
  python3 pump_fit_solve.py results_pump/20260820_1700 --crank-m 0.02

  # 부피를 직접 실측했다면 이중 부피법 생략
  python3 pump_fit_solve.py <dir> --crank-m 0.02 --volume-pos-ml 500 --volume-neg-ml 500
"""

import argparse
import csv
import glob
import math
import os
import sys
from collections import defaultdict
from datetime import datetime

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pump_fit_model as pm      # noqa: E402
import valve_fit_model as vm     # noqa: E402


# ══════════════════════════════════════════════════════════════════════════
def load_runs(indir):
    runs = []
    for path in sorted(glob.glob(os.path.join(indir, '*.csv'))):
        meta = {}
        mp = path[:-4] + '.meta.yaml'
        if os.path.exists(mp):
            with open(mp) as f:
                meta = yaml.safe_load(f) or {}
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        runs.append(dict(path=path, meta=meta, rows=rows,
                         phase=meta.get('phase') or rows[0]['phase'],
                         extra_ml=float(meta.get('extra_volume_ml', 0.0) or 0.0),
                         extra_rail=meta.get('extra_volume_rail', 'none'),
                         atm=float(meta.get('atm', vm.P_ATM_KPA))))
        r = runs[-1]
        print(f'  {os.path.basename(path)}: {len(rows)} 행, phase={r["phase"]}, '
              f'ΔV={r["extra_ml"]:.0f} mL → {r["extra_rail"]}')
    if not runs:
        raise SystemExit(f'{indir} 에 CSV 가 없다')
    return runs


def seg(rows, **match):
    out = [r for r in rows if all(str(r[k]) == str(v) for k, v in match.items())]
    return out


def col(rows, key):
    return np.array([float(r[key]) for r in rows])


# ══════════════════════════════════════════════════════════════════════════
# Step 1 — 누설 + 부피
# ══════════════════════════════════════════════════════════════════════════
def solve_volumes(runs, args):
    """Phase L 감쇠에서 τ 를 뽑고 이중 부피법으로 V 와 leak 을 함께 푼다."""
    taus = {}      # (rail, extra_rail) → dict
    for run in runs:
        if run['phase'] != 'leak':
            continue
        rows = seg(run['rows'], phase='leak', sub='decay')
        if len(rows) < 50:
            print(f'  건너뜀 (감쇠 구간 부족): {os.path.basename(run["path"])}')
            continue
        t = col(rows, 't')
        t = t - t[0]
        span = t[-1] - t[0]
        for rail, key in (('pos', 'P_pos'), ('neg', 'P_neg')):
            p = col(rows, key)
            fit = pm.exp_decay_fit(t, p, p_inf_guess=run['atm'])
            tag = f'{rail} 레일 (ΔV→{run["extra_rail"]})'
            if not (fit and math.isfinite(fit['tau'])):
                print(f'    {tag}: 감쇠 피팅 실패 — 기각')
                continue
            # 이 τ 가 부피의 절대 스케일을 정하므로 품질 게이트가 필수다. 리허설에서
            # R²=-805(상수보다 나쁜 피팅)인 τ=3.6e16 s 가 그냥 통과했다 — 뒤쪽 τ 비
            # 검사에 우연히 걸렸을 뿐이고, R²~0.3 짜리는 조용히 통과해 전부를 오염시킨다.
            drop = abs(p[0] - p[-1])
            why = None
            if drop < args.decay_min_kpa:
                why = (f'기록 구간 압력 변화 {drop:.2f} < {args.decay_min_kpa:.1f} kPa '
                       f'— 감쇠가 없다 (펌프가 정말 꺼졌는지 확인)')
            elif fit['r2'] < args.decay_min_r2:
                why = f'R²={fit["r2"]:.4f} < {args.decay_min_r2:.2f} — 1차 감쇠 모델이 안 맞는다'
            elif not (0.02 * span < fit['tau'] < 50.0 * span):
                why = (f'τ={fit["tau"]:.3g} s 가 기록 길이 {span:.1f} s 대비 범위 밖 '
                       f'— 기록을 τ 의 0.1~5 배로 잡을 것')
            if why:
                print(f'    {tag}: 기각 — {why}')
                continue
            taus[(rail, run['extra_rail'])] = fit
            print(f'    {tag}: τ={fit["tau"]:.2f} s  R²={fit["r2"]:.4f}  '
                  f'n={fit["n"]}  ΔP={drop:.1f} kPa')

    vols, leaks, diag = {}, {}, {}
    for rail in ('pos', 'neg'):
        override = getattr(args, f'volume_{rail}_ml')
        if override:
            vols[rail] = override
            diag[rail] = dict(method='지정값')
        else:
            bare = taus.get((rail, 'none'))
            extra = taus.get((rail, rail))
            if not (bare and extra):
                print(f'  {rail} 레일: 이중 부피법 불가 (bare={bool(bare)}, extra={bool(extra)})')
                continue
            extra_ml = next((r['extra_ml'] for r in runs
                             if r['phase'] == 'leak' and r['extra_rail'] == rail), 0.0)
            v, r = pm.volume_from_tau(bare['tau'], extra['tau'], extra_ml)
            if v is None:
                print(f'  {rail} 레일: τ 비 {r} 가 1 이하 — ΔV 가 너무 작거나 잡음이 크다')
                continue
            vols[rail] = v
            diag[rail] = dict(method='이중 부피법', tau_bare=bare['tau'],
                              tau_extra=extra['tau'], ratio=r, extra_ml=extra_ml,
                              r2_bare=bare['r2'], r2_extra=extra['r2'])
        tau = taus.get((rail, 'none'))
        if tau:
            leaks[rail] = pm.leak_from_tau(tau['tau'], vols[rail])
        print(f'  {rail} 레일: V = {vols.get(rail, float("nan")):.1f} mL, '
              f'leak = {leaks.get(rail, float("nan")):.5f} LPM/kPa')
    return vols, leaks, diag


# ══════════════════════════════════════════════════════════════════════════
# Step 2 — 2D 유량 맵
# ══════════════════════════════════════════════════════════════════════════
def _windows(t, arrays, skip_s, win_s, stride_s):
    """LPF 과도를 건너뛴 뒤 슬라이딩 창으로 (기울기, 평균) 을 뽑는다."""
    dt = float(np.median(np.diff(t))) if t.size > 1 else 0.005
    i0 = int(round(skip_s / max(dt, 1e-6)))
    w = max(6, int(round(win_s / max(dt, 1e-6))))
    st = max(1, int(round(stride_s / max(dt, 1e-6))))
    out = []
    for a in range(i0, t.size - w + 1, st):
        b = a + w
        tw = t[a:b] - t[a]
        out.append([(float(np.polyfit(tw, arr[a:b], 1)[0]), float(np.mean(arr[a:b])))
                    for arr in arrays])
    return out


def solve_map(runs, vols, leaks, args):
    """닫힌 레일의 dP/dt → ṁ_pump.

    **펄스를 한 점이 아니라 램프로 본다.** 밸브를 닫으면 레일이 계속 움직이므로 그 구간
    전체가 (P⁺,P⁻) 공간의 궤적이고, 슬라이딩 창마다 한 점을 얻는다. 한 점만 뽑으면
    유량이 큰 동작점은 압력 변화 제한에 먼저 걸려 표본이 짧아 버려지고, 유량이 거의 없는
    점만 살아남는 **선택 편향**이 생긴다 (실측: 15점이 전부 대기압 근처로 뭉쳤다).

    두 종류의 구간을 쓴다:
      · map/pulse       양 밸브 폐쇄 → 두 레일 모두 유효 (질량보존 교차검증 가능)
      · frontier/ramp   릴리프만 폐쇄 → **양압 레일만** 유효 (admit 이 열려 있으므로).
                        대신 (P⁺,P⁻) 범위를 넓게 쓸어 준다.
    """
    pts = []
    for run in runs:
        atm = run['atm']
        for phase, sub, use_neg in (('map', 'pulse', True), ('frontier', 'ramp', False)):
            by_pt = defaultdict(list)
            for r in seg(run['rows'], phase=phase, sub=sub):
                by_pt[int(r['point'])].append(r)
            for pt, rows in sorted(by_pt.items()):
                if len(rows) < 20:
                    continue
                t = col(rows, 't'); t = t - t[0]
                pp, pn = col(rows, 'P_pos'), col(rows, 'P_neg')
                ur, ua = col(rows, 'u_relief'), col(rows, 'u_admit')
                for (sp, mp_), (sn, mn_), (_, ur_m), (_, ua_m) in _windows(
                        t, (pp, pn, ur, ua), args.pulse_skip_s, args.win_s, args.stride_s):
                    if ur_m > args.closed_pct:          # 릴리프가 열려 있으면 양압식이 깨진다
                        continue
                    m_pos = pm.mdot_from_rail(sp, vols['pos'] * 1e-6) \
                        + pm.leak_kgps(mp_ - atm, leaks.get('pos', 0.0))
                    m_neg = None
                    if use_neg and ua_m <= args.closed_pct:
                        m_neg = -pm.mdot_from_rail(sn, vols['neg'] * 1e-6) \
                            + pm.leak_kgps(atm - mn_, leaks.get('neg', 0.0))
                    if m_pos <= args.mdot_min:
                        continue
                    if m_neg is not None and m_neg > args.mdot_min:
                        m_use = 0.5 * (m_pos + m_neg)
                        resid = (m_pos - m_neg) / m_use
                    else:
                        m_use, resid = m_pos, float('nan')
                    # pump_fit_model 규약에 맞춰 **게이지 Pa** 로 저장한다 (kPa 로 두면
                    # pump_avg(.. + P_ATM) 에서 101325 Pa 를 kPa 값에 더하게 된다)
                    pts.append(dict(ppos_g=(mp_ - atm) * 1e3, pneg_g=(mn_ - atm) * 1e3,
                                    mdot=m_use,
                                    m_pos=m_pos, m_neg=m_neg, resid=resid,
                                    point=pt, src=f'{phase}/{sub}'))
    if pts:
        rr = np.array([abs(p['resid']) for p in pts if p['resid'] == p['resid']])
        both = rr.size
        print(f'  {len(pts)} 점 (양 레일 교차검증 가능 {both} 점)')
        print(f'    P⁺ 범위 {min(p["ppos_g"] for p in pts)/1e3:.0f}~'
              f'{max(p["ppos_g"] for p in pts)/1e3:.0f} kPa,  '
              f'P⁻ 범위 {min(p["pneg_g"] for p in pts)/1e3:.0f}~'
              f'{max(p["pneg_g"] for p in pts)/1e3:.0f} kPa,  '
              f'ṁ {min(p["mdot"] for p in pts)*1e3:.3f}~'
              f'{max(p["mdot"] for p in pts)*1e3:.3f} g/s')
        if both:
            print(f'    질량보존 잔차 |Δṁ|/ṁ: 중앙값 {np.median(rr)*100:.1f}%  '
                  f'최대 {rr.max()*100:.1f}%')
        # 너무 많으면 (P⁺,P⁻) 격자로 솎아 피팅 비용을 줄인다
        if len(pts) > args.max_map_points:
            keep, seen = [], set()
            binw = args.thin_bin_kpa * 1e3
            for p in sorted(pts, key=lambda q: -q['mdot']):
                key = (round(p['ppos_g'] / binw), round(p['pneg_g'] / binw))
                if key not in seen:
                    seen.add(key); keep.append(p)
            print(f'    → {args.thin_bin_kpa:.0f} kPa 격자로 솎아 {len(keep)} 점 사용')
            pts = keep
    return pts


# ══════════════════════════════════════════════════════════════════════════
# Step 3 — 능력경계
# ══════════════════════════════════════════════════════════════════════════
def solve_frontier(runs, args):
    out = []
    for run in runs:
        if run['phase'] != 'frontier':
            continue
        atm = run['atm']
        ceil_abs = float(run['meta'].get('ppos_ceiling_abs', 1e9))
        by_pt = defaultdict(list)
        for r in seg(run['rows'], phase='frontier', sub='stall'):
            by_pt[int(r['point'])].append(r)
        for pt, rows in sorted(by_pt.items()):
            if len(rows) < 5:
                continue
            pp = float(np.median(col(rows, 'P_pos')))
            pn = float(np.median(col(rows, 'P_neg')))
            is_lb = pp >= ceil_abs - args.ceiling_tol
            out.append(dict(pneg_g=(pn - atm) * 1e3, ppos_g=(pp - atm) * 1e3,
                            lower_bound=is_lb, point=pt))
            print(f'    음압 {pn-atm:+7.1f} kPa → 양압 {pp-atm:7.1f} kPa gauge'
                  + ('  (하한 경계 — 안전 상한 도달)' if is_lb else '  (스톨)'))
    return out


# ══════════════════════════════════════════════════════════════════════════
def estimate_rpm(runs, npis, args):
    """정상 구간의 레일 압력 리플에서 회전수를 추정 (best-effort).

    CanBridge 의 1차 LPF(α=0.2, 코너 ≈18 Hz)가 100 Hz 를 크게 감쇠시키고 board1 분해능이
    0.25 kPa/LSB 로 거칠어 검출을 보장할 수 없다. 실패하면 None 을 돌려주고 정격을 쓴다.
    """
    best = None
    for run in runs:
        rows = seg(run['rows'], sub='hold') or seg(run['rows'], sub='ramp')
        if len(rows) < 512:
            continue
        t = col(rows, 't')
        dt = float(np.median(np.diff(t)))
        if not (dt > 0):
            continue
        p = col(rows, 'P_pos')
        p = p - np.mean(p)
        n = 1 << int(math.floor(math.log2(len(p))))
        p = p[:n] * np.hanning(n)
        spec = np.abs(np.fft.rfft(p))
        freq = np.fft.rfftfreq(n, dt)
        band = (freq > args.rpm_fmin) & (freq < min(args.rpm_fmax, 0.45 / dt))
        if not np.any(band):
            continue
        idx = np.argmax(spec[band])
        f_peak = freq[band][idx]
        snr = spec[band][idx] / max(np.median(spec[band]), 1e-12)
        cand = dict(f_peak=f_peak, rpm=f_peak / npis * 60.0, snr=snr,
                    src=os.path.basename(run['path']))
        if best is None or snr > best['snr']:
            best = cand
    if best and best['snr'] >= args.rpm_min_snr and args.rpm_expect_min <= best['rpm'] <= args.rpm_expect_max:
        print(f'  리플 첨두 {best["f_peak"]:.1f} Hz (SNR {best["snr"]:.1f}) '
              f'→ {best["rpm"]:.0f} rpm  [{best["src"]}]')
        return best
    if best:
        print(f'  리플 첨두 SNR {best["snr"]:.1f} < {args.rpm_min_snr} — 미검출로 처리')
    else:
        print('  리플 추정 불가 (정상 구간 표본 부족)')
    return None


# ══════════════════════════════════════════════════════════════════════════
def plots(outdir, geom, map_pts, front_pts, vols):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return
    # 1) 맵: 측정 산점 + 모델 등고선
    if map_pts:
        fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
        pp = np.array([p['ppos_g'] for p in map_pts]) / 1e3
        pn = np.array([p['pneg_g'] for p in map_pts]) / 1e3
        md = np.array([p['mdot'] for p in map_pts]) * 1e3
        sc = ax[0].scatter(pp, pn, c=md, cmap='viridis', s=42, edgecolor='k', lw=0.3)
        plt.colorbar(sc, ax=ax[0], label='ṁ [g/s]')
        gx = np.linspace(max(0.0, pp.min()), pp.max() + 1.0, 24)
        gy = np.linspace(pn.min() - 1.0, min(0.0, pn.max() + 1.0), 24)
        GX, GY = np.meshgrid(gx, gy)
        MO, _ = pm.pump_avg(geom, GX * 1e3 + pm.P_ATM, GY * 1e3 + pm.P_ATM)
        cs = ax[0].contour(GX, GY, MO * 1e3, colors='w', linewidths=0.7)
        ax[0].clabel(cs, fontsize=7)
        ax[0].set_xlabel('P⁺ [kPa gauge]'); ax[0].set_ylabel('P⁻ [kPa gauge]')
        ax[0].set_title('유량 맵 — 측정(점) vs 모델(등고선)')

        r = np.array([p['resid'] for p in map_pts
                      if p['resid'] == p['resid']]) * 100
        mdr = np.array([p['mdot'] for p in map_pts
                        if p['resid'] == p['resid']]) * 1e3
        ax[1].plot(mdr, r, 'o', ms=4)
        ax[1].axhline(0, color='k', lw=0.6)
        ax[1].set_xlabel('ṁ [g/s]'); ax[1].set_ylabel('질량보존 잔차 [%]')
        ax[1].set_title('양 레일 추정 불일치')
        for a in ax:
            a.grid(alpha=0.25, lw=0.4)
        fig.tight_layout(); fig.savefig(os.path.join(outdir, 'pump_map.png'), dpi=110)
        plt.close(fig)

    # 2) 능력경계: 측정 vs 피팅
    fig, ax = plt.subplots(figsize=(7, 4.6))
    pn_grid = np.linspace(-95e3, -20e3, 16)
    ax.plot(pn_grid / 1e3, pm.frontier(geom, pn_grid) / 1e3, '-', lw=1.6, label='피팅 모델')
    ax.plot(pn_grid / 1e3, pm.frontier(pm.PumpGeom(), pn_grid) / 1e3, '--', lw=1.0,
            color='gray', label='기존 값(예전 펌프)')
    for p in front_pts:
        ax.plot(p['pneg_g'] / 1e3, p['ppos_g'] / 1e3,
                'v' if p['lower_bound'] else 'o', color='C3', ms=8, label=None)
    ax.set_xlabel('P⁻ [kPa gauge]'); ax.set_ylabel('P⁺ 상한 [kPa gauge]')
    ax.set_title('능력경계 — ○ 스톨 측정, ▽ 하한 경계')
    ax.grid(alpha=0.25, lw=0.4); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, 'pump_frontier.png'), dpi=110)
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════════════
def write_yaml(outdir, geom, vols, leaks, args, indir, front_pts=(), map_pts=()):
    """머신 생성. **두 소비자 모두**에 같은 값을 써서 조용히 어긋나는 것을 막는다
    (지금 생성기는 하드코딩, 시뮬만 yaml 이라 어긋날 수 있는 구조다)."""
    pump = {k: (float(v) if not isinstance(v, int) else v)
            for k, v in geom.as_dict().items()}
    # 측정 능력경계를 그대로 싣는다. 컨트롤러가 펌프를 쓰는 유일한 경로가 cap_ppos 이고,
    # 기하 피팅에서 데드헤드는 측정 범위 밖 **외삽**이라 신뢰도가 낮다 — 측정값이 우선이다.
    fr = sorted(((float(round(p['pneg_g'] / 1e3, 3)), float(round(p['ppos_g'] / 1e3, 3)),
                  bool(p['lower_bound'])) for p in front_pts), key=lambda a: a[0])
    doc = {
        '/pack2/pp_controller': {'ros__parameters': {
            'PressureRefGen': {
                'pump': dict(pump), 'pump_grid_n': args.grid_n,
                'pump_frontier_measured': {
                    'pneg_kpa_gauge': [a for a, _, _ in fr],
                    'ppos_max_kpa_gauge': [b for _, b, _ in fr],
                    'is_lower_bound': [c for _, _, c in fr],
                }}}},
        '/pack2/can_bridge': {'ros__parameters': {
            'Virtual': {'pump': dict(pump)}}},
    }
    # 이 파일은 powerpack_config.yaml **뒤에** 병합되는 오버레이다. 측정하지 못한 값은
    # 키를 아예 쓰지 않아야 기존 설정값이 그대로 유효하다. 예전에는 폴백 리터럴
    # (부피 500 mL, 누설 0.002 LPM/kPa) 을 적었는데, 그러면 미측정값이 **측정값처럼**
    # 파일에 박혀 이후에 구별이 불가능해진다.
    virt = doc['/pack2/can_bridge']['ros__parameters']['Virtual']
    # 중앙집중 MPPI(solver: mppi_system) 는 레일을 롤아웃 상태로 예측하므로 **컨트롤러도**
    # 레일 부피·누설을 알아야 한다. Phase L 이 구하는 값이 바로 그것이다.
    ctrl_mpc = doc['/pack2/pp_controller']['ros__parameters'].setdefault('MPC_parameters', {})
    missing = []
    for rail in ('pos', 'neg'):
        if rail in vols:
            virt[f'line_volume_{rail}_ml'] = float(round(vols[rail], 2))
            ctrl_mpc[f'rail_volume_{rail}_ml'] = float(round(vols[rail], 2))
        else:
            missing.append(f'line_volume_{rail}_ml')
        if rail in leaks and leaks[rail] == leaks[rail]:
            virt[f'line_leak_{rail}_lpm_per_kpa'] = float(f'{leaks[rail]:.6g}')
            ctrl_mpc[f'rail_leak_{rail}'] = float(f'{leaks[rail]:.6g}')
        else:
            missing.append(f'line_leak_{rail}_lpm_per_kpa')
    doc['/pack2/can_bridge']['ros__parameters']['Virtual']['pump_map_measured'] = {
        'ppos_kpa_gauge': [float(round(p['ppos_g'] / 1e3, 2)) for p in map_pts],
        'pneg_kpa_gauge': [float(round(p['pneg_g'] / 1e3, 2)) for p in map_pts],
        'mdot_gps': [float(round(p['mdot'] * 1e3, 5)) for p in map_pts],
    }
    header = (
        '# ============================================================\n'
        '# pump_params.yaml — pump_fit_solve.py 가 생성한 파일 (직접 편집하지 말 것)\n'
        f'# 생성: {datetime.now().isoformat(timespec="seconds")}\n'
        f'# 입력: {os.path.abspath(indir)}\n'
        f'# 크랭크 반경(고정) {geom.r:.5g} m,  RPM {geom.rpm:.0f},  피스톤 {geom.Npis}\n'
        f'# 소기량 {geom.v_swept*1e6:.2f} mL/피스톤/회전,  사구간 {geom.v_dead*1e6:.3f} mL,'
        f'  압축비 {geom.compression_ratio:.1f}\n'
        '#\n'
        '# powerpack_config.yaml **뒤에** 병합한다.\n'
        '#\n'
        '# ★ 신뢰도 순서 ★\n'
        '#   1) pump_frontier_measured  — 직접 측정. 컨트롤러가 쓰는 유일한 출력(cap_ppos)이다.\n'
        '#   2) pump_map_measured       — 직접 측정한 (P⁺,P⁻,ṁ) 점들. 시뮬 flow_out 용.\n'
        '#   3) pump: {...}             — 기하 피팅 산물. 소기량×Cb_in 축퇴가 남아 개별\n'
        '#      기하값은 배수로 틀릴 수 있고, 데드헤드는 측정 범위 밖 외삽이다\n'
        '#      (자기검증: 능력경계 ~15% 오차). 참고용으로 쓰고 능력경계는 위 측정 테이블을 쓸 것.\n'
        '# 생성기와 시뮬 **양쪽**에 같은 값을 쓴다 — 두 소비자가 다른 펌프를 가정하면\n'
        '# 생성기의 능력경계가 시뮬 하드웨어와 어긋나는데 아무 경고도 나지 않는다.\n'
        + ('' if not missing else
           '#\n# ⚠ 아래 키는 **측정하지 못해 이 파일에 없다** — powerpack_config.yaml 의\n'
           '#   기존 값이 그대로 쓰인다. Phase L 을 다시 기록해서 채울 것:\n'
           + ''.join(f'#     {k}\n' for k in missing))
        + '# ============================================================\n'
    )
    path = os.path.join(outdir, 'pump_params.yaml')
    with open(path, 'w') as f:
        f.write(header)
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False, default_flow_style=False)
    return path


def write_report(outdir, geom, base, vols, leaks, vdiag, map_pts, front_pts,
                 sens, rpm_est, args, indir):
    L = ['# 펌프 파라미터 피팅 리포트\n',
         f'- 생성: {datetime.now().isoformat(timespec="seconds")}',
         f'- 입력: `{os.path.abspath(indir)}`',
         f'- 고정: 크랭크 반경 {geom.r:.5g} m, 피스톤 {geom.Npis}개, '
         f'RPM {geom.rpm:.0f}' + ('  (리플 추정)' if rpm_est else '  (정격/지정)'),
         '']

    L.append('## 레일 부피 · 누설\n')
    L.append('| 레일 | V [mL] | leak [LPM/kPa] | 방법 | τ(맨몸) | τ(ΔV) | 비 |')
    L.append('|---|---|---|---|---|---|---|')
    for rail in ('pos', 'neg'):
        d = vdiag.get(rail, {})
        L.append(f'| {rail} | {vols.get(rail, float("nan")):.1f} | '
                 f'{leaks.get(rail, float("nan")):.5f} | {d.get("method","-")} | '
                 f'{d.get("tau_bare", float("nan")):.2f} | '
                 f'{d.get("tau_extra", float("nan")):.2f} | '
                 f'{d.get("ratio", float("nan")):.4f} |')
    L.append('\n> 이중 부피법: 누설이 ΔP 에 선형이면 감쇠가 지수이고 `τ = V·1000/(R·T·k)` 이므로'
             ' **ΔV 회차의 τ 비가 `(V+ΔV)/V`** 다. 펌프가 전혀 관여하지 않아 펌프 모델 오차가'
             ' 부피 추정에 섞이지 않는다.\n')

    L.append('## 유량 맵 (Phase M)\n')
    if map_pts:
        rr = np.array([abs(p['resid']) for p in map_pts if p['resid'] == p['resid']])
        if rr.size:
            L.append(f'{len(map_pts)} 점 (교차검증 가능 {rr.size} 점). **질량보존 잔차** '
                     f'|ṁ⁺−ṁ⁻|/ṁ: 중앙값 {np.median(rr)*100:.1f}%, 최대 {rr.max()*100:.1f}%\n')
        else:
            L.append(f'{len(map_pts)} 점 (양 레일 교차검증 가능 구간 없음)\n')
        L.append('양 레일에서 독립적으로 계산한 ṁ_pump 가 얼마나 일치하는지다. 크게 어긋나면'
                 ' 부피·누설 추정이나 채널 밸브 폐쇄(레일 누출 경로)를 의심해야 한다.\n')
        L.append('_ṁ⁻ 가 — 인 점은 프론티어 램프 유래다 (admit 이 열려 있어 음압식이 성립하지 않는다).'
                 ' 그 구간은 양압 레일만으로 추정한다._\n')
        L.append('| P⁺ [kPa g] | P⁻ [kPa g] | ṁ 측정 [g/s] | ṁ 모델 [g/s] | ṁ⁺ | ṁ⁻ | 잔차 |')
        L.append('|---|---|---|---|---|---|---|')
        mo, _ = pm.pump_avg(geom,
                            np.array([p['ppos_g'] for p in map_pts]) + pm.P_ATM,
                            np.array([p['pneg_g'] for p in map_pts]) + pm.P_ATM)
        for p, m in zip(map_pts, np.atleast_1d(mo)):
            mn = f'{p["m_neg"]*1e3:.4f}' if p['m_neg'] is not None else '—'
            rs = f'{p["resid"]*100:+.1f}%' if p['resid'] == p['resid'] else '—'
            L.append(f'| {p["ppos_g"]/1e3:.1f} | {p["pneg_g"]/1e3:.1f} | {p["mdot"]*1e3:.4f} '
                     f'| {m*1e3:.4f} | {p["m_pos"]*1e3:.4f} | {mn} | {rs} |')
    else:
        L.append('_Phase M 데이터가 없다._')

    L.append('\n## 능력경계 (Phase F) — **컨트롤러가 실제로 쓰는 유일한 출력**\n')
    L.append('`PressureRefGen::decide_rail_setpoint` 이 `ppos_sp = min(cap_ppos(pneg_sp), 목표)`'
             ' 로만 쓴다. `flow_out`/`flow_in` 은 생성기에서 호출조차 되지 않는다.\n')
    if front_pts:
        L.append('| P⁻ [kPa g] | 측정 P⁺ 상한 | 피팅 | 기존값 | 종류 |')
        L.append('|---|---|---|---|---|')
        for p in front_pts:
            f_new = pm.cap_ppos(geom, p['pneg_g']) / 1e3
            f_old = pm.cap_ppos(base, p['pneg_g']) / 1e3
            kind = '하한 경계' if p['lower_bound'] else '스톨'
            L.append(f'| {p["pneg_g"]/1e3:.1f} | {p["ppos_g"]/1e3:.1f} | {f_new:.1f} '
                     f'| {f_old:.1f} | {kind} |')
        L.append('\n> `▽ 하한 경계` 는 안전 상한에 먼저 닿아 스톨을 못 본 점이다. 목적함수에서'
                 ' "그 이상이면 벌점 0" 으로 다룬다.')
        L.append(f'\n**실용 판정**: 생성기의 `pos_sp_max_kpa` 는 250 kPa gauge 다. 경계가 그보다'
                 ' 위면 `cap_ppos` 는 상시 느슨해 컨트롤러 동작에 영향이 없다.')
    else:
        L.append('_Phase F 데이터가 없다 — 능력경계는 맵에서 외삽한 값이라 불확실하다._')

    L.append('\n> **경고 — 기하 피팅의 능력경계는 신뢰하지 말 것.** 5-파라미터 슬라이더-크랭크는'
             ' 유량 데이터로 다중 모드다: 측정 범위 안의 유량을 잘 맞추면서도 데드헤드(측정 범위'
             ' **밖** 외삽)가 크게 틀어진다. 자기검증에서 맵 RMS 19% 로 맞추면서 압축비가 469 까지'
             ' 올라가 경계가 1200 kPa 로 튀는 해가 나왔다. → `pump_frontier_measured` 테이블을 쓸 것.\n')
    L.append('\n## 피팅 결과 (참고용)\n')
    L.append('| 항목 | 기존 (예전 펌프) | 피팅 |')
    L.append('|---|---|---|')
    for label, f in (('소기량 [mL/피스톤/회전]', lambda g: g.v_swept * 1e6),
                     ('사구간 [mL]', lambda g: g.v_dead * 1e6),
                     ('압축비', lambda g: g.compression_ratio),
                     ('토출 체크밸브 [mm²]', lambda g: g.Cb_out * 1e6),
                     ('흡입 체크밸브 [mm²]', lambda g: g.Cb_in * 1e6),
                     ('l/r', lambda g: g.l / g.r),
                     ('피스톤 면적 [cm²]', lambda g: g.Spis * 1e4),
                     ('delta [mm]', lambda g: g.delta * 1e3)):
        L.append(f'| {label} | {f(base):.4g} | {f(geom):.4g} |')

    L.append('\n### 식별성 (±10% 섭동 시 목적함수 상대 변화)\n')
    L.append('0 에 가까우면 이 데이터로 결정되지 않는다 — **직접 실측을 권장**한다.\n')
    L.append('| 파라미터 | 민감도 | 판정 |')
    L.append('|---|---|---|')
    for k, v in sens.items():
        L.append(f'| {k} | {v:.2e} | {"낮음 — 실측 권장" if v < 1e-3 else "ok"} |')
    L.append('\n> `크랭크 반경 r` 은 소기량과 곱으로만 나타나 따로 갈리지 않으므로 실측값으로'
             ' 고정했다. `l/r` 은 부피 파형 형상만 살짝 바꿔 거의 식별되지 않는다.\n')

    L.append('## RPM\n')
    if rpm_est:
        L.append(f'리플 첨두 {rpm_est["f_peak"]:.1f} Hz (SNR {rpm_est["snr"]:.1f}, '
                 f'피스톤 {geom.Npis}개) → **{rpm_est["rpm"]:.0f} rpm**')
    else:
        L.append(f'리플 미검출 → 지정/정격 {geom.rpm:.0f} rpm 을 사용했다.')
    L.append('\n> CanBridge LPF(α=0.2, 코너 ≈18 Hz)가 100 Hz 를 크게 감쇠시키고 board1 분해능이'
             ' 0.25 kPa/LSB 로 거칠어 리플 검출은 best-effort 다. 유량 맵 전체가 ω 에 비례하므로'
             ' 태코미터 실측이 가능하면 그쪽이 낫다.\n')

    L.append('## 반영 방법\n')
    L.append('`pump_params.yaml` 을 `powerpack_config.yaml` 뒤에 병합한다.\n')
    L.append('**주의**: 지금 `PressureRefGen` 은 펌프 파라미터를 yaml 에서 읽지 않는다'
             ' (`Controller.cpp` 에 `gp.pump.*` 대입이 없어 `PistonPump.hpp` 하드코딩을 쓴다).'
             ' `PressureRefGen.pump.*` 읽기를 추가하지 않으면 이 결과가 컨트롤러에 도달하지 못한다.\n')
    with open(os.path.join(outdir, 'pump_report.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


# ══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('indir')
    ap.add_argument('--crank-m', type=float, required=True,
                    help='실측 크랭크 반경 [m]. 소기량과 곱으로만 나타나 따로 갈리지 않는다.')
    ap.add_argument('--rpm', type=float, default=None, help='지정 RPM. 없으면 리플 추정 → 정격')
    ap.add_argument('--n-piston', type=int, default=2)
    ap.add_argument('--decay-min-r2', type=float, default=0.90,
                    help='Phase L 감쇠 피팅 R² 하한. τ 가 모든 값의 절대 스케일을 '
                         '정하므로 낮추지 말 것.')
    ap.add_argument('--decay-min-kpa', type=float, default=3.0,
                    help='감쇠로 인정할 최소 압력 변화 [kPa]. 센서 잡음보다 커야 한다.')
    ap.add_argument('--volume-pos-ml', type=float, default=None, help='실측 부피로 이중 부피법 대체')
    ap.add_argument('--volume-neg-ml', type=float, default=None)
    ap.add_argument('--pulse-frac', type=float, default=0.7,
                    help='LPF 과도 이후 구간 중 기울기에 쓸 앞부분 비율')
    ap.add_argument('--pulse-skip-s', type=float, default=0.06,
                    help='구간 시작 후 버릴 시간 [s]. 센서 LPF 시상수(≈20 ms)의 3배.')
    ap.add_argument('--win-s', type=float, default=0.15, help='기울기 창 길이 [s]')
    ap.add_argument('--stride-s', type=float, default=0.10, help='창 이동 간격 [s]')
    ap.add_argument('--closed-pct', type=float, default=1.0,
                    help='이 개도 이하를 "닫힘" 으로 본다 [%%]')
    ap.add_argument('--mdot-min', type=float, default=2.0e-5,
                    help='이보다 작은 유량 추정은 버린다 [kg/s] (잡음 바닥)')
    ap.add_argument('--max-map-points', type=int, default=400)
    ap.add_argument('--thin-bin-kpa', type=float, default=15.0)
    ap.add_argument('--ceiling-tol', type=float, default=8.0,
                    help='안전 상한 도달 판정 여유 [kPa]')
    ap.add_argument('--samples', type=int, default=80)
    ap.add_argument('--starts', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--grid-n', type=int, default=13)
    ap.add_argument('--frontier-weight', type=float, default=3.0,
                    help='스톨점 가중치. 능력경계가 컨트롤러가 쓰는 유일한 출력이라 크게 준다.')
    ap.add_argument('--fit-dt', type=float, default=4e-4, help='피팅 중 적분 스텝 (성기게)')
    ap.add_argument('--fit-nrev', type=int, default=6)
    ap.add_argument('--rpm-fmin', type=float, default=30.0)
    ap.add_argument('--rpm-fmax', type=float, default=220.0)
    ap.add_argument('--rpm-min-snr', type=float, default=6.0)
    ap.add_argument('--rpm-expect-min', type=float, default=1000.0,
                    help='리플 추정 결과가 이 범위를 벗어나면 오검출로 버린다')
    ap.add_argument('--rpm-expect-max', type=float, default=6000.0)
    ap.add_argument('--out', default=None)
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()

    outdir = args.out or args.indir
    os.makedirs(outdir, exist_ok=True)

    print('Step 0 — CSV 로드')
    runs = load_runs(args.indir)

    print('\nStep 1 — 레일 부피 · 누설')
    vols, leaks, vdiag = solve_volumes(runs, args)
    if 'pos' not in vols or 'neg' not in vols:
        raise SystemExit('레일 부피를 확정하지 못했다. Phase L 을 맨몸/ΔV(pos)/ΔV(neg) 3회 '
                         '기록하거나 --volume-pos-ml/--volume-neg-ml 로 실측값을 넣을 것.\n'
                         '유량계가 없어 부피가 절대 스케일 앵커다 — 없이는 진행할 수 없다.')

    print('\nStep 2 — 2D 유량 맵')
    map_pts = solve_map(runs, vols, leaks, args)

    print('\nStep 3 — 능력경계')
    front_pts = solve_frontier(runs, args)

    print('\nStep 3b — RPM')
    rpm_est = estimate_rpm(runs, args.n_piston, args)
    rpm = args.rpm or (rpm_est['rpm'] if rpm_est else pm.PumpGeom().rpm)

    if not map_pts and not front_pts:
        raise SystemExit('Phase M/F 데이터가 모두 없다 — 피팅할 대상이 없다')

    print('\nStep 4 — 기하 피팅')
    # 스톨점은 "유량이 누설과 같은 맵 점" 이다 — 압력을 비교하면 모델의 경계 판정
    # 기준(>0.02 g/s)과 실측 정의(dP/dt≈0, 즉 누설과 균형)가 어긋나 계통 오차가 난다.
    flow_pts = [(p['ppos_g'], p['pneg_g'], p['mdot'], 1.0, 0.0) for p in map_pts]
    if 'pos' not in leaks:
        # leaks.get('pos', 0.0) 이 조용히 0 을 넣으면 "스톨에서 유량 0" 이 되고, 유량을
        # 전혀 안 내는 펌프가 그 항을 공짜로 만족시킨다 (리허설에서 실제로 그랬다).
        print('  !! 양압 누설 미측정 — 스톨 목표 유량이 0 이 된다. 기하 피팅 결과를 '
              '신뢰하지 말 것 (측정 맵·경계 테이블은 영향 없음).')
    for p in front_pts:
        tgt = float(pm.leak_kgps(p['ppos_g'] / 1e3, leaks.get('pos', 0.0)))
        flow_pts.append((p['ppos_g'], p['pneg_g'], tgt,
                         args.frontier_weight, 1.0 if p['lower_bound'] else 0.0))
        print(f'    스톨점 → 목표 유량 = 누설 {tgt*1e3:.4f} g/s '
              f'@ P⁺={p["ppos_g"]/1e3:.1f} kPa gauge')
    stall_pts = [(p['pneg_g'], p['ppos_g'],
                  float(pm.leak_kgps(p['ppos_g'] / 1e3, leaks.get('pos', 0.0))) * 1e3,
                  1.0 if p['lower_bound'] else 0.0) for p in front_pts]
    geom, cost, diag = pm.fit(flow_pts, r_fixed=args.crank_m, rpm=rpm, npis=args.n_piston,
                              stall_points=stall_pts, w_stall=args.frontier_weight,
                              n_samples=args.samples, n_starts=args.starts, seed=args.seed,
                              dt=args.fit_dt, nrev=args.fit_nrev)
    print(f'  cost={cost:.5g}  {diag}')
    print(f'  {geom}')
    print(f'  소기량 {geom.v_swept*1e6:.2f} mL, 사구간 {geom.v_dead*1e6:.3f} mL, '
          f'압축비 {geom.compression_ratio:.1f}')

    sens = pm.sensitivity(geom, flow_pts, r_fixed=args.crank_m, rpm=rpm,
                          npis=args.n_piston, dt=args.fit_dt, nrev=args.fit_nrev,
                          stall_points=stall_pts, w_stall=args.frontier_weight)
    weak = [k for k, v in sens.items() if v < 1e-3]
    if weak:
        print(f'  식별성 낮음 (실측 권장): {weak}')

    print('\nStep 5 — 저장')
    p = write_yaml(outdir, geom, vols, leaks, args, args.indir, front_pts, map_pts)
    write_report(outdir, geom, pm.PumpGeom(), vols, leaks, vdiag,
                 map_pts, front_pts, sens, rpm_est, args, args.indir)
    print(f'  {p}')
    print(f'  {os.path.join(outdir, "pump_report.md")}')
    if not args.no_plots:
        plots(outdir, geom, map_pts, front_pts, vols)
        print(f'  pump_map.png / pump_frontier.png')
    return 0


if __name__ == '__main__':
    sys.exit(main())
