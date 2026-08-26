#!/usr/bin/env python3
"""
valve_fit_solve.py — 기록된 CSV 에서 밸브 파라미터를 피팅하고 config 로 내보낸다

  Step 1  CSV/meta 로드, (모드, 채널, 밸브) 단위로 연속 기록 재구성
  Step 2  이중 부피법으로 채널별 챔버 부피 V 결정 — 유량계가 없어 A_max 와 V 가
          분리되지 않으므로 절대 스케일을 여기서 고정한다
  Step 3  A절 13-parameter 피팅 (밸브별). 레퍼런스 MATLAB 과 동일한 2단계 솔버
  Step 4  B절 오리피스 유효면적 → Cd·eta 산출
  Step 5  valve_params.yaml + report.md + 밸브별 재생 플롯 출력

사용:
  # 이중 부피법 (권장): 같은 모드를 ΔV 0 / ΔV 100 mL 로 두 번 기록한 뒤
  python3 valve_fit_solve.py results_fit/20260820_161230 --extra-volume-ml 100

  # 부피를 따로 실측해서 넣는 경우 (이중 부피법 생략)
  python3 valve_fit_solve.py <dir> --volume-ml 128.5

  # 단일 밸브만 빠르게 확인
  python3 valve_fit_solve.py <dir> --only-gid 0 --only-valve v1 --samples 40
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
import valve_fit_model as vm   # noqa: E402

# 각 밸브 경로의 기하 오리피스 지름 [mm] — PressureRefGen.orifice_mm 와 같은 출처
ORIFICE_MM = {
    ('pos_micro', 'v1'): 2.3,   # fill   레일 → 양압 챔버
    ('pos_micro', 'v2'): 2.3,   # vent   양압 챔버 → 대기 (실측 2.3 mm)
    ('pos_macro', 'v3'): 1.6,   # boost  탱크 → 양압 챔버
    ('pos_macro', 'v2'): 2.3,   # vent   pos_micro 의 v2 와 같은 물리 밸브다
    ('neg_micro', 'v1'): 4.0,   # suck   음압 챔버 → 음압 레일
    ('neg_micro', 'v2'): 4.0,   # admit  대기 → 음압 챔버
    ('neg_macro', 'v3'): 4.0,   # eject  음압 챔버 → 외부 진공
    ('neg_macro', 'v2'): 4.0,   # admit
}
ORIFICE_ROLE = {
    ('pos_micro', 'v1'): 'fill', ('pos_micro', 'v2'): 'vent',
    ('pos_macro', 'v3'): 'boost', ('pos_macro', 'v2'): 'vent',
    ('neg_micro', 'v1'): 'suck', ('neg_micro', 'v2'): 'admit',
    ('neg_macro', 'v3'): 'eject', ('neg_macro', 'v2'): 'admit',
}


def load_warm_start(path):
    """valve_params.yaml 형식에서 (gid, valve) → 13-parameter 벡터를 뽑는다.

    channel_config.chN.{micro,atm,macro} 의 값을 vm.PARAM_NAMES 순서로 재배열한다.
    없는 채널/키는 조용히 건너뛴다 — 호출부가 못 찾으면 BASE_INITIAL 단독으로 돈다.
    """
    valve_key = {'v1': 'micro', 'v2': 'atm', 'v3': 'macro'}
    with open(path) as f:
        d = yaml.safe_load(f)
    cc = d.get('/pack2/pp_controller', {}).get('ros__parameters', {}).get('channel_config', {})
    out = {}
    for ch_name, ch in cc.items():
        if not ch_name.startswith('ch'):
            continue
        try:
            gid = int(ch_name[2:])
        except ValueError:
            continue
        for valve, key in valve_key.items():
            block = ch.get(key)
            if not block:
                continue
            try:
                out[(gid, valve)] = np.array([float(block[n]) for n in vm.PARAM_NAMES])
            except KeyError:
                continue
    return out


# ══════════════════════════════════════════════════════════════════════════
# Step 1 — 로드
# ══════════════════════════════════════════════════════════════════════════
def load_runs(indir):
    """디렉터리의 CSV+meta 쌍을 모두 읽어 run 리스트로."""
    runs = []
    for path in sorted(glob.glob(os.path.join(indir, '*.csv'))):
        meta_path = path[:-4] + '.meta.yaml'
        meta = {}
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                meta = yaml.safe_load(f) or {}
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            print(f'  건너뜀 (빈 파일): {os.path.basename(path)}')
            continue
        runs.append(dict(path=path, meta=meta, rows=rows,
                         extra_ml=float(meta.get('extra_volume_ml',
                                                 rows[0].get('extra_vol_ml', 0.0) or 0.0))))
        print(f'  {os.path.basename(path)}: {len(rows)} 행, ΔV={runs[-1]["extra_ml"]:.0f} mL')
    if not runs:
        raise SystemExit(f'{indir} 에 CSV 가 없다')
    return runs


def reject_glitches(rate, window=9, k=10.0):
    """dP/dt 의 순간 글리치(짧은 튐)를 검출해 불리언 마스크로 반환한다 (True=정상).

    로컬 이동중앙값 대비 크게 벗어나는 점만 잡는다 — 진짜 빠른 밸브 전이는 이웃 샘플도
    같이 움직이므로 안 걸리고, 실기 CAN/센서 순간 글리치처럼 **한두 샘플만** 튀는 것만
    잡힌다. scipy 가 없어(레포 관례) 순수 파이썬 루프로 구현 — 채널당 수만 샘플이라
    피팅 1회 준비 단계에서 문제되지 않는다.
    """
    n = rate.size
    if n < window:
        return np.ones(n, dtype=bool)
    half = window // 2
    med = np.empty(n)
    for i in range(n):
        lo, hi = max(0, i - half), min(n, i + half + 1)
        med[i] = np.median(rate[lo:hi])
    resid = rate - med
    mad = np.median(np.abs(resid - np.median(resid))) * 1.4826
    if mad < 1e-9:
        return np.ones(n, dtype=bool)
    return np.abs(resid) <= k * mad


def build_record(rows, mode, gid, valve, decimate=1, fixed_volume_side='chamber'):
    """한 (모드, 채널, 밸브) 의 **연속** 기록을 만든다.

    히스테리시스 상태 z 가 레벨을 넘어 이어져야 하므로 세그먼트를 쪼개지 않고,
    첫 sweep 부터 마지막 sweep 까지 통째로 쓴다. 반대 밸브로 초기화하는 구간은
    mask 로 잔차에서 제외한다 (그 구간의 dP/dt 는 다른 밸브가 만든 것이므로).

    fixed_volume_side: 'chamber'(기본, 기존 전제 — 챔버가 고정부피) | 'line'
      (음압 v1 전용 대안: 펌프로 라인압을 만드는 대신, **라인/레일 쪽에 고정부피 탱크를
      붙이고 챔버는 외부 레귤레이터로 일정압을 유지**한다. 이러면 유량을 역산할 dP/dt 는
      챔버가 아니라 그 고정부피 탱크(라인)에서 나와야 한다 — 그래서 이 모드에서는
      `rate`/`p_ch` 필드에 **라인 압력**을 담는다(이름은 그대로 두되 의미만 바꿔, `_blocks`/
      `solve_volume`/`orifice_coeff` 등 "고정부피 쪽 압력"을 쓰는 다운스트림 코드를
      그대로 재사용한다). v2(대기↔챔버)는 이 방식으로 특성화가 안 된다 — 챔버가 레귤레이터로
      고정돼 있으면 v2 유량이 챔버압에 안 나타난다. v1/v3 (라인↔챔버) 에만 쓸 것.
    """
    sel = [r for r in rows if r['mode'] == mode and int(r['gid']) == gid]
    if not sel:
        return None
    idx = [i for i, r in enumerate(sel) if r['phase'] == 'sweep' and r['valve'] == valve]
    if len(idx) < 50:
        return None
    lo, hi = idx[0], idx[-1] + 1
    seg = sel[lo:hi]

    t = np.array([float(r['t']) for r in seg])
    p_ch_raw = np.array([float(r['P_ch_abs']) for r in seg])
    p_atm = np.array([float(r['P_atm']) for r in seg])
    mask = np.array([r['phase'] == 'sweep' and r['valve'] == valve for r in seg])

    # 이 밸브의 실측 전류 — 태그와 무관하게 항상 자기 컬럼에서 읽는다
    cur = np.array([float(r[f'I_{valve}']) for r in seg])
    ucmd = np.array([float(r[f'u_{valve}']) for r in seg])

    # 상/하류압은 (모드, 밸브) 로 유일하게 정해진다
    line_key = {'pos_micro': 'P_line_pos', 'pos_macro': 'P_macro',
                'neg_micro': 'P_line_neg', 'neg_macro': 'P_macro_neg'}[mode]
    p_line_raw = np.array([float(r[line_key]) for r in seg])
    sign = +1 if mode.startswith('pos') else -1

    if fixed_volume_side == 'line':
        if valve not in ('v1', 'v3'):
            raise ValueError("fixed_volume_side='line' 은 v1/v3(라인↔챔버) 에만 쓸 수 있다 "
                             f"(valve={valve})")
        # 압력 신호를 맞바꾼다: "p_ch"(고정부피 쪽) = 실제 라인/레일 압력,
        # "p_line"(레귤레이터 쪽) = 실제 챔버 압력(외부 레귤레이터로 일정압).
        p_ch, p_line = p_line_raw, p_ch_raw
    else:
        p_ch, p_line = p_ch_raw, p_line_raw

    if valve in ('v1', 'v3'):                       # 라인 ↔ 챔버
        p_in, p_out = (p_line, p_ch) if sign > 0 else (p_ch, p_line)
    else:                                           # 챔버 ↔ 대기
        p_in, p_out = (p_ch, p_atm) if sign > 0 else (p_atm, p_ch)

    lvl = np.array([float(r['level_pct']) for r in seg])
    dirs = np.array([r['sweep_dir'] for r in seg])

    # dP/dt 는 **원본 해상도**에서 계산한다. 부피 산출과 오리피스 계수는 이 전해상도 값을 쓰고,
    # 피팅만 데시메이션한다 (밸브 동특성 wn≈40 rad/s 대비 200 Hz 는 과잉).
    # (fixed_volume_side='line' 이면 위에서 이미 p_ch 가 라인압으로 바뀌어 있어 그대로 맞다.)
    rate_full = vm.dpdt(t, p_ch)
    # full['mask'] 는 그대로 둔다 — 이중부피법(_blocks/solve_volume)과 B절 오리피스 계산이
    # 이걸 써서 연속구간을 찾는데, 여기에 글리치 마스크를 섞으면 그 연속성이 끊겨 부피·
    # 크래킹 임계 추정이 같이 나빠진다(실측: 부피오차 2.9%→18.4%로 회귀 확인). 글리치
    # 제거는 **13-parameter 피팅용** mask 에만 적용한다.
    full = dict(t=t, p_ch=p_ch, p_in=p_in, p_out=p_out, rate=rate_full,
                mask=mask, level=lvl, sweep=dirs)
    fit_mask = mask & reject_glitches(rate_full)
    if decimate > 1:
        sl = slice(None, None, decimate)
        t, p_ch, p_in, p_out = t[sl], p_ch[sl], p_in[sl], p_out[sl]
        cur, ucmd, lvl, dirs = cur[sl], ucmd[sl], lvl[sl], dirs[sl]
        mask, rate = fit_mask[sl], rate_full[sl]
    else:
        mask, rate = fit_mask, rate_full
    return dict(t=t, p_ch=p_ch, p_in=p_in, p_out=p_out, I=cur, u=ucmd, rate=rate,
                mask=mask, level=lvl, sweep=dirs, gid=gid, mode=mode, valve=valve,
                full=full)


# ══════════════════════════════════════════════════════════════════════════
# Step 2 — 이중 부피법
# ══════════════════════════════════════════════════════════════════════════
def _blocks(rec, min_span):
    """(레벨, 방향) 별 연속 스윕 블록을 (t, p) 로 뽑는다."""
    f = rec['full']
    key = list(zip(f['level'], f['sweep']))
    out, i, n = {}, 0, len(key)
    while i < n:
        j = i
        while j < n and key[j] == key[i] and f['mask'][j]:
            j += 1
        if j > i + 20 and f['mask'][i]:
            p, t = f['p_ch'][i:j], f['t'][i:j]
            if abs(p[-1] - p[0]) >= min_span:
                out[key[i]] = (t, p)
        i = max(j, i + 1)
    return out


def _transit_time(t, p, lo, hi, rising):
    """압력이 [lo, hi] 를 지나는 데 걸린 시간. 선형 보간으로 창 경계를 정확히 맞춘다."""
    if rising:
        pm = np.maximum.accumulate(p)
        if pm[0] > lo or pm[-1] < hi:
            return None
        return float(np.interp(hi, pm, t) - np.interp(lo, pm, t))
    pm = np.minimum.accumulate(p)
    if pm[0] < hi or pm[-1] > lo:
        return None
    pr, tr = pm[::-1], t[::-1]
    return float(np.interp(lo, pr, tr) - np.interp(hi, pr, tr))


def solve_volume(rec_a, rec_b, extra_ml, margin=0.12, min_span=3.0):
    """**통과시간법**으로 챔버 부피를 구한다.

    같은 레벨에서 챔버압이 **공통 절대 구간** [P1, P2] 를 지나는 데 걸린 시간은 부피에 비례한다:
        Δt = ∫ V·dP/(Q(P)·K)   →   Δt_B/Δt_A = V_B/V_A = (V + ΔV)/V   →   V = ΔV/(r − 1)
    미분 비를 점별로 쓰는 것보다 훨씬 튼튼하다 — 밸브 과도 첨두가 시간 적분으로 상쇄되고,
    두 회차의 시점을 맞출 필요도 없다.

    핵심: 창을 **두 회차 압력 구간의 교집합**에서 잡아야 한다. 각 회차 자기 구간의
    상대 위치로 잡으면 ΔV 회차가 더 느려 덜 올라가므로 서로 다른 절대 구간을 비교하게 되고
    r 이 과소평가된다 (실측: 그 오류로 V 가 128.5 → 168 mL 로 30% 부풀었다).
    """
    ba, bb = _blocks(rec_a, min_span), _blocks(rec_b, min_span)
    ratios, used = [], []
    for k in sorted(set(ba) & set(bb)):
        (ta, pa), (tb, pb) = ba[k], bb[k]
        rising = pa[-1] > pa[0]
        if rising != (pb[-1] > pb[0]):
            continue
        if rising:
            lo = max(pa.min(), pb.min())
            hi = min(pa.max(), pb.max())
        else:
            lo = max(pa.min(), pb.min())
            hi = min(pa.max(), pb.max())
        if hi - lo < min_span:
            continue
        pad = (hi - lo) * margin              # 과도·정착 구간을 앞뒤로 잘라낸다
        lo, hi = lo + pad, hi - pad
        dta = _transit_time(ta, pa, lo, hi, rising)
        dtb = _transit_time(tb, pb, lo, hi, rising)
        if dta and dtb and dta > 1e-6:
            ratios.append(dtb / dta)
            used.append(k)
    if len(ratios) < 3:
        return None, None, len(ratios), None
    r = float(np.median(ratios))
    if r <= 1.0 + 1e-6:
        return None, r, len(ratios), float(np.std(ratios))
    return extra_ml / (r - 1.0), r, len(ratios), float(np.std(ratios))


# ══════════════════════════════════════════════════════════════════════════
# Step 4 — B절 오리피스 계수
# ══════════════════════════════════════════════════════════════════════════
def orifice_coeff(rec, volume_m3, n_poly, d_mm):
    """완전개방(level=100) 구간에서 물리 유효면적과 Cd·eta 를 구한다.

    ṁ = dP/dt · V · 1000 / (n·R·T)          [kg/s]   ← LPM 환산 상수가 상쇄돼 단위 모호성 없음
    A_SI = ṁ · √(R·T) / (P_up[Pa] · Φ)      [m²]
    Cd·eta = A_SI / (π d²/4)
    """
    f = rec['full']
    rate = f['rate']
    ph = vm.phi(f['p_in'], f['p_out'])
    sel = f['mask'] & (f['level'] >= 99.0) & (ph > 1e-6) & (np.abs(rate) > 1.0)
    if np.count_nonzero(sel) < 10:
        return None
    mdot = np.abs(rate[sel]) * volume_m3 * 1000.0 / (n_poly * vm.RGAS * vm.TEMP_K)
    a_si = mdot * math.sqrt(vm.RGAS * vm.TEMP_K) / (f['p_in'][sel] * 1000.0 * ph[sel])
    a_geo = math.pi * (d_mm * 1e-3) ** 2 / 4.0
    return dict(a_si_mm2=float(np.median(a_si)) * 1e6,
                a_geo_mm2=a_geo * 1e6,
                cd_eta=float(np.median(a_si)) / a_geo,
                spread=float(np.std(a_si / a_geo)),
                n=int(np.count_nonzero(sel)))


# ══════════════════════════════════════════════════════════════════════════
def plot_fit(rec, params, seg, outpath):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    pred = vm.predict(params, seg)
    t = seg['t'] - seg['t'][0]
    fig, ax = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
    ax[0].plot(t, seg['Q'], lw=0.8, label='실측 (dP/dt 유래)')
    ax[0].plot(t, pred, lw=0.9, label='모델')
    bad = ~seg['mask']
    if np.any(bad):
        ax[0].fill_between(t, *ax[0].get_ylim(), where=bad, alpha=0.12,
                           color='gray', step='mid', label='잔차 제외 구간')
    ax[0].set_ylabel('Q [LPM]'); ax[0].legend(fontsize=8, loc='upper right')
    ax[0].set_title(f'{rec["mode"]}  gid {rec["gid"]}  {rec["valve"]}')
    ax[1].plot(t, rec['p_ch'], lw=0.8, label='P_chamber')
    ax[1].plot(t, rec['p_in'], lw=0.6, label='P_up')
    ax[1].plot(t, rec['p_out'], lw=0.6, label='P_dn')
    ax[1].set_ylabel('kPa abs'); ax[1].legend(fontsize=8, loc='upper right')
    ax[2].plot(t, rec['I'] / vm.I_MAX * 100.0, lw=0.8, color='C3')
    ax[2].set_ylabel('실측 전류 [%]'); ax[2].set_xlabel('t [s]')
    for a in ax:
        a.grid(alpha=0.25, lw=0.4)
    fig.tight_layout()
    fig.savefig(outpath, dpi=110)
    plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('indir', help='valve_fit_record.py 출력 디렉터리')
    ap.add_argument('--extra-volume-ml', type=float, default=None,
                    help='이중 부피법: 2회차에서 추가한 알려진 부피 [mL]')
    ap.add_argument('--volume-ml', type=float, default=None,
                    help='이중 부피법을 쓰지 않고 실측 챔버 부피를 직접 지정 [mL]')
    ap.add_argument('--n-poly', type=float, default=1.0,
                    help='챔버 폴리트로픽 지수. 기본 1.0(등온) = Controller/VirtualPowerpack 과 일치')
    ap.add_argument('--samples', type=int, default=200, help='1단계 난수 탐색 샘플 수')
    ap.add_argument('--decimate', type=int, default=2,
                    help='피팅 전 데시메이션 배율. dP/dt 는 원본 해상도에서 먼저 계산한다.')
    ap.add_argument('--fixed-volume-side', default='chamber', choices=['chamber', 'line'],
                    help="'chamber'(기본, 기존 전제) | 'line' — 음압 v1/v3 전용: 펌프 대신 "
                         "라인/레일 쪽에 고정부피 탱크, 챔버는 외부 레귤레이터로 일정압. "
                         "target 밸브(v1/v3)에만 적용되고 neutral(v2)은 항상 'chamber' 로 "
                         "돈다 — v2 는 이 방식으로 특성화가 안 된다.")
    ap.add_argument('--starts', type=int, default=3)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--warm-start-yaml', default=None,
                    help='valve_params.yaml 형식 파일. 있으면 (gid,valve) 가 일치하는 '
                         '13-parameter 를 BASE_INITIAL 과 함께 추가 탐색 시작점으로 쓴다 '
                         '(채널별 국소해 실패 완화용 웜스타트).')
    ap.add_argument('--only-gid', type=int, default=None)
    ap.add_argument('--only-valve', default=None, choices=['v1', 'v2', 'v3'])
    ap.add_argument('--out', default=None, help='기본 <indir>')
    ap.add_argument('--no-plots', action='store_true')
    args = ap.parse_args()

    outdir = args.out or args.indir
    os.makedirs(outdir, exist_ok=True)

    warm_start = {}
    if args.warm_start_yaml:
        warm_start = load_warm_start(args.warm_start_yaml)
        print(f'웜스타트: {args.warm_start_yaml} 에서 (gid,valve) {len(warm_start)}개 로드')

    print('Step 1 — CSV 로드')
    runs = load_runs(args.indir)

    # (mode, gid, valve) → {extra_ml: record}
    print('\nStep 1b — 기록 재구성')
    recs = defaultdict(dict)
    for run in runs:
        mode = run['meta'].get('mode') or run['rows'][0]['mode']
        target = run['meta'].get('target') or 'v1'
        neutral = run['meta'].get('neutral') or 'v2'
        gids = sorted({int(r['gid']) for r in run['rows'] if int(r['gid']) >= 0})
        for gid in gids:
            if args.only_gid is not None and gid != args.only_gid:
                continue
            for valve in (target, neutral):
                if args.only_valve and valve != args.only_valve:
                    continue
                fv_side = args.fixed_volume_side if valve == target else 'chamber'
                rec = build_record(run['rows'], mode, gid, valve, args.decimate, fv_side)
                if rec is None:
                    continue
                recs[(mode, gid, valve)][run['extra_ml']] = rec
                print(f'  {mode} gid{gid} {valve}: {len(rec["t"])} 샘플 '
                      f'(유효 {int(rec["mask"].sum())}), ΔV={run["extra_ml"]:.0f}')
    if not recs:
        raise SystemExit('재구성할 기록이 없다 — CSV 의 phase/valve 태그를 확인할 것')

    # ── Step 2: 부피 ──
    print('\nStep 2 — 챔버 부피')
    volumes = {}      # gid → mL
    vol_diag = {}
    if args.volume_ml is not None:
        for (mode, gid, valve) in recs:
            volumes[gid] = args.volume_ml
        print(f'  지정값 사용: {args.volume_ml:.2f} mL (이중 부피법 생략)')
    elif args.extra_volume_ml:
        for (mode, gid, valve), by_v in recs.items():
            if valve not in ('v1', 'v3') or gid in volumes:
                continue
            if 0.0 in by_v and args.extra_volume_ml in by_v:
                v_ml, r, n, sd = solve_volume(by_v[0.0], by_v[args.extra_volume_ml],
                                              args.extra_volume_ml)
                if v_ml:
                    volumes[gid] = v_ml
                    vol_diag[gid] = dict(ratio=r, n_bins=n, ratio_std=sd)
                    print(f'  gid{gid}: V = {v_ml:.2f} mL  (r={r:.4f}, 빈 {n}개, σ={sd:.4f})')
                else:
                    print(f'  gid{gid}: 이중 부피법 실패 (r={r}, 빈 {n}개)')
        if not volumes:
            raise SystemExit('이중 부피법이 성립한 채널이 없다 — ΔV 회차 CSV 가 같은 디렉터리에 있는지, '
                             '--extra-volume-ml 값이 meta 와 일치하는지 확인할 것')
    else:
        raise SystemExit('--extra-volume-ml (이중 부피법) 또는 --volume-ml 중 하나가 필요하다.\n'
                         '유량계가 없어 A_max 와 V 가 분리되지 않으므로 절대 스케일 앵커가 반드시 필요하다.')

    # ── Step 3/4 ──
    print('\nStep 3 — 13-parameter 피팅')
    fits, orif = {}, {}
    for (mode, gid, valve), by_v in sorted(recs.items()):
        if gid not in volumes:
            print(f'  {mode} gid{gid} {valve}: 부피 미확정 — 건너뜀')
            continue
        rec = by_v.get(0.0) or next(iter(by_v.values()))
        v_m3 = volumes[gid] * 1e-6

        rate = rec['rate']
        sign = +1 if mode.startswith('pos') else -1
        flow_sign = sign if valve in ('v1', 'v3') else -sign   # 중립밸브는 반대 방향
        q_meas = vm.q_from_dpdt(rate * flow_sign, v_m3, args.n_poly)

        seg = vm.prepare_segment(rec['t'], rec['I'], rec['p_in'], rec['p_out'], q_meas)
        seg['mask'] = rec['mask']
        seg['q0'] = 0.0                                     # 스윕 시작 시 밸브 폐쇄
        seg['q_ref'] = float(np.mean(np.abs(seg['Q'][rec['mask']]))) if rec['mask'].any() else 0.0

        warm_vec = warm_start.get((gid, valve))
        base = [vm.BASE_INITIAL, warm_vec] if warm_vec is not None else None

        print(f'\n  {mode} gid{gid} {valve} (V={volumes[gid]:.2f} mL)'
              + ('  [웜스타트 있음]' if warm_vec is not None else ''))
        params, err, r2, diag = vm.fit([seg], base=base, n_samples=args.samples,
                                       n_starts=args.starts, seed=args.seed)
        sens = vm.sensitivity(params, [seg])
        weak = [k for k, v in sens.items() if v < 1e-4]
        print(f'    SSE={err:.4g}  R²={r2:.4f}' + (f'  식별성 낮음: {weak}' if weak else ''))

        fits[(mode, gid, valve)] = dict(params=params, sse=err, r2=r2,
                                        sens=sens, weak=weak, diag=diag,
                                        volume_ml=volumes[gid])

        d_mm = ORIFICE_MM.get((mode, valve))
        if d_mm:
            oc = orifice_coeff(rec, v_m3, args.n_poly, d_mm)
            if oc:
                oc['d_mm'] = d_mm
                oc['role'] = ORIFICE_ROLE[(mode, valve)]
                orif[(mode, valve)] = oc

        if not args.no_plots:
            png = os.path.join(outdir, f'fit_{mode}_ch{gid}_{valve}.png')
            plot_fit(rec, params, seg, png)

    # ── Step 5: 출력 ──
    print('\nStep 5 — 결과 저장')
    write_yaml(outdir, fits, volumes, orif, args)
    write_report(outdir, fits, volumes, vol_diag, orif, args, runs)
    print(f'  {os.path.join(outdir, "valve_params.yaml")}')
    print(f'  {os.path.join(outdir, "report.md")}')
    if not args.no_plots:
        print(f'  fit_*.png ({len(fits)}개)')
    return 0


def write_yaml(outdir, fits, volumes, orif, args):
    """머신 생성 파일 — powerpack_config.yaml 뒤에 병합해 덮어쓰는 용도.

    C++ 로더가 `channel_config.chN.{micro,atm,macro}.*` 를 읽는다 (없으면 평면
    `chN.*` 로 폴백). 즉 이 파일을 config/ 에 넣고 다시 빌드하면 **밸브별 값이 실제로
    쓰인다** — 피드포워드 역모델·크래킹 임계·Bouc-Wen·MPPI 롤아웃·QP 선형화 전부.
    기동 로그의 "밸브별 파라미터: N/M 채널" 로 로드 여부를 확인할 수 있다.
    """
    ch = defaultdict(dict)
    valve_role = {'v1': 'micro', 'v2': 'atm', 'v3': 'macro'}
    for (mode, gid, valve), fit in sorted(fits.items()):
        entry = {n: float(round(v, 8)) for n, v in zip(vm.PARAM_NAMES, fit['params'])}
        entry['I_MAX'] = vm.I_MAX
        entry['_fit'] = dict(mode=mode, r2=float(round(fit['r2'], 5)),
                             sse=float(f"{fit['sse']:.6g}"),
                             volume_ml=float(round(fit['volume_ml'], 3)),
                             weak_params=fit['weak'])
        ch[f'ch{gid}'][valve_role[valve]] = entry
    for gid, v in volumes.items():
        ch[f'ch{gid}']['chamber_volume_ml'] = float(round(v, 3))

    doc = {
        '/pack2/pp_controller': {
            'ros__parameters': {
                'channel_config': dict(sorted(ch.items())),
                'PressureRefGen': {
                    'orifice_measured': {
                        o['role']: dict(d_mm=o['d_mm'],
                                        a_eff_mm2=float(round(o['a_si_mm2'], 5)),
                                        a_geo_mm2=float(round(o['a_geo_mm2'], 5)),
                                        cd_eta=float(round(o['cd_eta'], 5)),
                                        spread=float(round(o['spread'], 5)),
                                        n=o['n'])
                        for o in orif.values()
                    }
                },
            }
        }
    }
    header = (
        '# ============================================================\n'
        '# valve_params.yaml — valve_fit_solve.py 가 생성한 파일 (직접 편집하지 말 것)\n'
        f'# 생성: {datetime.now().isoformat(timespec="seconds")}\n'
        f'# 입력: {os.path.abspath(args.indir)}\n'
        f'# n_poly={args.n_poly}  seed={args.seed}  samples={args.samples}\n'
        '#\n'
        '# powerpack_config.yaml **뒤에** 병합해 덮어쓰는 용도다.\n'
        '# C++ 로더가 chN.{micro,atm,macro}.* 를 읽는다 (없으면 평면 chN.* 폴백).\n'
        '# config/valve_params.yaml 로 넣고 재빌드하면 밸브별 값이 실제로 쓰인다.\n'
        '# 기동 로그의 "밸브별 파라미터: N/M 채널" 로 확인할 것.\n'
        '# ============================================================\n'
    )
    with open(os.path.join(outdir, 'valve_params.yaml'), 'w') as f:
        f.write(header)
        yaml.safe_dump(doc, f, allow_unicode=True, sort_keys=False, default_flow_style=False)


def write_report(outdir, fits, volumes, vol_diag, orif, args, runs):
    L = []
    L.append('# 밸브 파라미터 피팅 리포트\n')
    L.append(f'- 생성: {datetime.now().isoformat(timespec="seconds")}')
    L.append(f'- 입력: `{os.path.abspath(args.indir)}`')
    L.append(f'- 기록 파일 {len(runs)}개, 피팅된 밸브 {len(fits)}개')
    L.append(f'- `n_poly = {args.n_poly}` (등온). '
             '**PressureRefGen 은 `n_ch = 1.4`(단열)를 쓰므로 불일치가 있다** — '
             '이 모델의 소비 측(Controller 피드포워드, VirtualPowerpack 챔버)이 등온이라 '
             '그쪽에 맞췄다.\n')

    L.append('## 챔버 부피\n')
    if vol_diag:
        L.append('이중 부피법: `V = ΔV/(r−1)`, `r = (dP/dt)_ΔV0 / (dP/dt)_ΔV`\n')
        L.append('| gid | V [mL] | r | 매칭 빈 | σ(r) |')
        L.append('|---|---|---|---|---|')
        for gid in sorted(volumes):
            d = vol_diag.get(gid, {})
            L.append(f'| {gid} | {volumes[gid]:.2f} | {d.get("ratio", float("nan")):.4f} '
                     f'| {d.get("n_bins", 0)} | {d.get("ratio_std", float("nan")):.4f} |')
    else:
        L.append(f'지정값 `--volume-ml {args.volume_ml}` 사용 (이중 부피법 생략).')
    L.append('\n> 유량계가 없어 `A_max` 와 `V` 는 분리되지 않는다 — 지배식에 `A_max/V` 곱만 '
             '나타나므로 `V` 가 절대 스케일 앵커다. `V` 가 틀리면 `A_max` 가 그 비율대로 틀린다.\n')

    L.append('## A절 — 13-parameter (밸브별)\n')
    L.append('| 모드 | gid | 밸브 | R² | SSE | ' + ' | '.join(vm.PARAM_NAMES) + ' |')
    L.append('|---|---|---|---|---|' + '---|' * len(vm.PARAM_NAMES))
    for (mode, gid, valve), fit in sorted(fits.items()):
        vals = ' | '.join(f'{v:.5g}' for v in fit['params'])
        L.append(f'| {mode} | {gid} | {valve} | {fit["r2"]:.4f} | {fit["sse"]:.4g} | {vals} |')

    L.append('\n### 식별성 (±10% 섭동 시 SSE 상대 변화)\n')
    L.append('값이 0 에 가까우면 그 파라미터는 이 데이터로 결정되지 않는다 — 초기값이 그대로 남는다.\n')
    L.append('| 모드 | gid | 밸브 | ' + ' | '.join(vm.PARAM_NAMES) + ' |')
    L.append('|---|---|---|' + '---|' * len(vm.PARAM_NAMES))
    for (mode, gid, valve), fit in sorted(fits.items()):
        vals = ' | '.join(f'{fit["sens"][n]:.2e}' for n in vm.PARAM_NAMES)
        L.append(f'| {mode} | {gid} | {valve} | {vals} |')
    weak_all = {k: f['weak'] for k, f in fits.items() if f['weak']}
    if weak_all:
        L.append('\n**식별성 낮은 파라미터:**\n')
        for (mode, gid, valve), w in sorted(weak_all.items()):
            L.append(f'- `{mode} gid{gid} {valve}`: {", ".join(w)}')

    L.append('\n## v2 교차검증\n')
    L.append('v2(대기 밸브)는 micro 모드와 macro 모드에 모두 등장해 독립 2회 피팅된다. '
             '두 결과가 크게 다르면 실험 조건이나 부피 추정에 문제가 있다는 뜻이다.\n')
    v2 = defaultdict(dict)
    for (mode, gid, valve), fit in fits.items():
        if valve == 'v2':
            v2[gid][mode] = fit
    if any(len(m) > 1 for m in v2.values()):
        L.append('| gid | 파라미터 | ' + ' | '.join(sorted({m for d in v2.values() for m in d}))
                 + ' | 상대차 |')
        L.append('|---|---|' + '---|' * (len(sorted({m for d in v2.values() for m in d})) + 1))
        for gid, bym in sorted(v2.items()):
            if len(bym) < 2:
                continue
            modes = sorted(bym)
            for i, name in enumerate(vm.PARAM_NAMES):
                vals = [bym[m]['params'][i] for m in modes]
                span = (max(vals) - min(vals)) / max(1e-12, abs(np.mean(vals)))
                L.append(f'| {gid} | {name} | ' + ' | '.join(f'{v:.5g}' for v in vals)
                         + f' | {span*100:.1f}% |')
    else:
        L.append('_두 모드가 모두 기록된 채널이 없어 비교 불가._')

    L.append('\n## B절 — 오리피스 유효면적 / Cd·eta\n')
    L.append('완전개방(level 100%) 구간에서 `ṁ = dP/dt·V·1000/(n·R·T)`, '
             '`A_eff = ṁ·√(R·T)/(P_up·Φ)`, `Cd·eta = A_eff / (πd²/4)`.\n')
    L.append('LPM 환산 상수가 상쇄되므로 이 계산에는 단위 모호성이 없다.\n')
    if orif:
        L.append('| 경로 | 지름 [mm] | A_geo [mm²] | A_eff [mm²] | Cd·eta | σ | 표본 |')
        L.append('|---|---|---|---|---|---|---|')
        for (mode, valve), o in sorted(orif.items()):
            L.append(f'| {o["role"]} ({mode} {valve}) | {o["d_mm"]:.1f} | {o["a_geo_mm2"]:.3f} '
                     f'| {o["a_si_mm2"]:.3f} | {o["cd_eta"]:.4f} | {o["spread"]:.4f} | {o["n"]} |')
        L.append('\n반영 방법: `PressureRefGen.Cd` 와 `valve_open_eta` 의 곱이 위 `Cd·eta` 다. '
                 '`Cd` 를 제조사값으로 고정하면 `valve_open_eta = Cd·eta / Cd` 로 나뉜다.')
        cds = [o['cd_eta'] for o in orif.values()]
        if max(cds) > 1.0:
            L.append(f'\n> **경고**: `Cd·eta` 최대 {max(cds):.3f} > 1 — 유효면적이 기하 면적을 '
                     '넘는다. 오리피스 지름이나 챔버 부피 추정을 다시 확인할 것.')
    else:
        L.append('_완전개방 구간 표본이 부족해 산출하지 못했다._')

    L.append('\n## 소비 측 반영\n')
    L.append('현재 `ChannelConfig` 는 채널당 13-parameter 세트가 **하나**뿐이고 '
             'micro/atm/macro 에 같은 값을 쓴다 (`Controller.cpp:1250-1263`). '
             '위 밸브 단위 결과를 실제로 로드하려면 로더를 '
             '`channel_config.chN.{micro,atm,macro}.*` 3세트로 확장해야 한다.\n')
    with open(os.path.join(outdir, 'report.md'), 'w') as f:
        f.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    sys.exit(main())
