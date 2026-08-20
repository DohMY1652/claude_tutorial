#!/usr/bin/env python3
"""
pump_fit_selftest.py — 하드웨어 없이 펌프 피팅 파이프라인을 검증한다

알려진 `PumpGeom` · 레일 부피 · 누설로 3단계 실험을 합성(레일 ODE + 라인 밸브 13-parameter
+ 센서 양자화/LPF)해 `pump_fit_record.py` 와 **같은 형식**의 CSV 를 만들고,
`pump_fit_solve.py` 로 피팅해서 원래 값을 복원하는지 본다. 실기 전에 반드시 통과시킬 것.

판정은 밸브 때와 같은 철학이다 — 개별 기하값이 아니라 **컨트롤러가 실제로 쓰는 것**으로:
  1. 레일 부피 (유량계가 없어 절대 스케일 앵커다)
  2. 능력경계 (`cap_ppos` — 생성기가 펌프를 쓰는 유일한 경로)
  3. 유량 맵 정확도

사용:
  python3 pump_fit_selftest.py               # 기본
  python3 pump_fit_selftest.py --keep        # 합성 데이터/결과 보존
"""

import argparse
import csv
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pump_fit_model as pm                              # noqa: E402
import valve_fit_model as vm                             # noqa: E402
from pump_fit_record import CSV_HEADER, IDX_RELIEF, IDX_ADMIT  # noqa: E402,F401
from valve_fit_selftest import ValveSim                  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))

# ── 진짜 값 (기존 하드코딩과 일부러 크게 다르게 — 초기값 반환을 통과로 오판하지 않도록) ──
TRUE_R = 0.02                     # 크랭크 반경 [m] (실측으로 고정하는 값)
TRUE_RPM = 2800.0
TRUE_NPIS = 2
TRUE_V_SWEPT = 14.0e-6            # 14 mL/피스톤/회전  (기존 153.9 mL)
TRUE_V_DEAD = 1.4e-6              # 1.4 mL             (기존 3.85 mL) → 압축비 11
TRUE_CB_OUT = 4.0e-6              # 4.0 mm²            (기존 1.46 mm²)
TRUE_CB_IN = 9.0e-6               # 9.0 mm²            (기존 33.47 mm²)
TRUE_L_OVER_R = 3.5

TRUE_V_POS_ML = 420.0
TRUE_V_NEG_ML = 380.0
TRUE_K_POS = 0.0025               # LPM/kPa
TRUE_K_NEG = 0.0018
EXTRA_ML = 250.0

# 라인 밸브 (board1/2 v1) — 시뮬 기본값과 같은 13-parameter. 피팅에는 쓰이지 않는다.
LINE_VALVE_PARAMS = vm.BASE_INITIAL

SENSOR_LPF_ALPHA = 0.2
CALIB = {1: (1112.0, 0.250), 2: (1020.0, -0.02525),
         3: (1064.0, 0.250), 4: (1020.0, -0.02525), 'atm': vm.P_ATM_KPA}


def true_geom():
    spis = TRUE_V_SWEPT / (2.0 * TRUE_R)
    delta = TRUE_V_DEAD / spis + 2.0 * TRUE_R
    return pm.PumpGeom(delta=delta, r=TRUE_R, l=TRUE_L_OVER_R * TRUE_R, rpm=TRUE_RPM,
                       Spis=spis, Npis=TRUE_NPIS, Cb_out=TRUE_CB_OUT, Cb_in=TRUE_CB_IN)


class Rig:
    """레일 2개 + 라인 밸브 2개 + 펌프 + 누설 + 센서."""

    def __init__(self, geom, v_pos_ml, v_neg_ml, seed=1):
        self.map = pm.PumpMap(geom)
        self.vr = ValveSim(LINE_VALVE_PARAMS)      # board1 v1 : 양압 → 대기
        self.va = ValveSim(LINE_VALVE_PARAMS)      # board2 v1 : 대기 → 음압
        self.atm = CALIB['atm']
        self.v_pos = v_pos_ml * 1e-6
        self.v_neg = v_neg_ml * 1e-6
        self.p_pos = self.p_neg = self.atm
        self.meas_pos = self.meas_neg = self.atm
        self.rng = np.random.RandomState(seed)
        self.t = 0.0

    def _sense(self, board, kpa, prev):
        off, gain = CALIB[board]
        mv = round((kpa - self.atm) / gain + off)          # ADC 양자화
        mv = min(max(mv, 0.0), 5000.0)
        raw = (mv - off) * gain + self.atm
        return SENSOR_LPF_ALPHA * raw + (1.0 - SENSOR_LPF_ALPHA) * prev

    def step(self, u_relief, u_admit, pump_on, dt):
        m_pump = self.map.flow(self.p_pos * 1000.0, self.p_neg * 1000.0) if pump_on else 0.0
        q_r = self.vr.step(u_relief / 100.0 * vm.I_MAX, self.p_pos, self.atm, dt)   # LPM
        q_a = self.va.step(u_admit / 100.0 * vm.I_MAX, self.atm, self.p_neg, dt)
        leak_p = TRUE_K_POS * max(0.0, self.p_pos - self.atm)
        leak_n = TRUE_K_NEG * max(0.0, self.atm - self.p_neg)

        md_pos = m_pump - (q_r + leak_p) * vm.LPM_TO_KGPS
        md_neg = (q_a + leak_n) * vm.LPM_TO_KGPS - m_pump
        k = vm.RGAS * vm.TEMP_K / 1000.0
        self.p_pos = max(1.0, self.p_pos + dt * k * md_pos / self.v_pos)
        self.p_neg = max(1.0, self.p_neg + dt * k * md_neg / self.v_neg)

        self.meas_pos = self._sense(1, self.p_pos, self.meas_pos)
        self.meas_neg = self._sense(2, self.p_neg, self.meas_neg)
        self.t += dt
        return self.meas_pos, self.meas_neg


class Writer:
    def __init__(self, extra_ml, extra_rail):
        self.rows = []
        self.extra_ml = extra_ml
        self.extra_rail = extra_rail

    def add(self, rig, phase, sub, point, pump_on, u_r, u_a):
        self.rows.append([round(rig.t, 5), phase, sub, point, int(pump_on),
                          round(u_r, 3), round(u_a, 3),
                          round(rig.meas_pos, 4), round(rig.meas_neg, 4),
                          round(rig.atm, 4), round(rig.atm, 4), rig.atm,
                          self.extra_ml, self.extra_rail])


def run_until(rig, w, phase, sub, point, u_r, u_a, pump_on, dt, max_s,
              settle_eps=None, ceiling=None, dp_limit=None):
    """조건 만족까지 적분하며 기록. 반환: 'settled'|'limit'|'dp'|'timeout'"""
    n = int(max_s / dt)
    p0p, p0n = rig.meas_pos, rig.meas_neg
    calm = 0
    prev = (rig.meas_pos, rig.meas_neg)
    for _ in range(n):
        rig.step(u_r, u_a, pump_on, dt)
        w.add(rig, phase, sub, point, pump_on, u_r, u_a)
        rp = (rig.meas_pos - prev[0]) / dt
        rn = (rig.meas_neg - prev[1]) / dt
        prev = (rig.meas_pos, rig.meas_neg)
        if ceiling is not None and rig.meas_pos >= ceiling:
            return 'limit'
        if dp_limit is not None and (abs(rig.meas_pos - p0p) >= dp_limit
                                     or abs(rig.meas_neg - p0n) >= dp_limit):
            return 'dp'
        if settle_eps is not None and abs(rp) <= settle_eps and abs(rn) <= settle_eps:
            calm += 1
            if calm * dt >= 0.3:
                return 'settled'
        else:
            calm = 0
    return 'timeout'


# ══════════════════════════════════════════════════════════════════════════
def synth_leak(geom, extra_ml, extra_rail, dt, args):
    v_pos = TRUE_V_POS_ML + (extra_ml if extra_rail == 'pos' else 0.0)
    v_neg = TRUE_V_NEG_ML + (extra_ml if extra_rail == 'neg' else 0.0)
    rig = Rig(geom, v_pos, v_neg)
    w = Writer(extra_ml, extra_rail)
    run_until(rig, w, 'leak', 'charge', 0, 0.0, 0.0, True, dt, 40.0,
              ceiling=rig.atm + args.charge_gauge)
    run_until(rig, w, 'leak', 'decay', 0, 0.0, 0.0, False, dt, args.leak_seconds)
    return w, dict(phase='leak', extra_volume_ml=extra_ml, extra_volume_rail=extra_rail,
                   atm=rig.atm, rows=len(w.rows), synthetic=True)


def synth_map(geom, dt, args):
    rig = Rig(geom, TRUE_V_POS_ML, TRUE_V_NEG_ML)
    w = Writer(0.0, 'none')
    grid = [(a, b) for a in args.u_grid for b in args.u_grid]
    ceil = rig.atm + args.ppos_ceiling
    for k, (u1, u2) in enumerate(grid):
        run_until(rig, w, 'map', 'hold', k, u1, u2, True, dt, 8.0,
                  settle_eps=1.5, ceiling=ceil)
        run_until(rig, w, 'map', 'pulse', k, 0.0, 0.0, True, dt, args.pulse_s,
                  dp_limit=args.pulse_dp, ceiling=ceil)
        run_until(rig, w, 'map', 'reopen', k, 100.0, 100.0, True, dt, 0.5)
    return w, dict(phase='map', extra_volume_ml=0.0, extra_volume_rail='none',
                   atm=rig.atm, ppos_ceiling_abs=ceil, pulse_s=args.pulse_s,
                   pulse_dp_kpa=args.pulse_dp, rows=len(w.rows), synthetic=True)


def synth_frontier(geom, dt, args):
    """record 의 ramp_with_pneg_hold 와 같은 구조 — admit 밸브로 P⁻ 를 목표에 잡고 P⁺ 램프."""
    rig = Rig(geom, TRUE_V_POS_ML, TRUE_V_NEG_ML)
    w = Writer(0.0, 'none')
    ceil = rig.atm + args.ppos_ceiling
    for k, tgt_g in enumerate(args.pneg_targets):
        tgt = rig.atm + tgt_g
        run_until(rig, w, 'frontier', 'reset', k, 100.0, 100.0, True, dt, 6.0, settle_eps=1.5)
        u_admit = args.admit_start
        calm, prev = 0, rig.meas_pos
        for _ in range(int(args.frontier_timeout / dt)):
            u_admit = min(max(u_admit + args.admit_kp * (tgt - rig.meas_neg) * dt, 0.0), 100.0)
            rig.step(0.0, u_admit, True, dt)
            w.add(rig, 'frontier', 'ramp', k, 1, 0.0, u_admit)
            rp = (rig.meas_pos - prev) / dt
            prev = rig.meas_pos
            if rig.meas_pos >= ceil:
                break
            if abs(rp) <= 0.4:
                calm += 1
                if calm * dt >= 0.5:
                    break
            else:
                calm = 0
        run_until(rig, w, 'frontier', 'stall', k, 0.0, u_admit, True, dt, 1.0)
        print(f'    P⁻ 목표 {tgt_g:+6.1f} → 실제 {rig.meas_neg - rig.atm:+6.1f}, '
              f'P⁺ {rig.meas_pos - rig.atm:7.1f} kPa gauge (u_admit {u_admit:.1f}%)')
    return w, dict(phase='frontier', extra_volume_ml=0.0, extra_volume_rail='none',
                   atm=rig.atm, ppos_ceiling_abs=ceil,
                   pneg_targets=list(args.pneg_targets), rows=len(w.rows), synthetic=True)


def write(outdir, tag, w, meta):
    path = os.path.join(outdir, f'{tag}.csv')
    with open(path, 'w', newline='') as f:
        c = csv.writer(f)
        c.writerow(CSV_HEADER)
        c.writerows(w.rows)
    with open(path[:-4] + '.meta.yaml', 'w') as f:
        yaml.safe_dump(meta, f, allow_unicode=True, sort_keys=False)
    return path


# ══════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dt', type=float, default=0.005, help='합성 적분/기록 스텝 [s] (200 Hz)')
    ap.add_argument('--u-grid', type=float, nargs='*', default=[40.0, 60.0, 80.0, 100.0])
    ap.add_argument('--pneg-targets', type=float, nargs='*',
                    default=[-80.0, -65.0, -50.0, -35.0], help='[kPa gauge]')
    ap.add_argument('--admit-kp', type=float, default=3.0)
    ap.add_argument('--admit-start', type=float, default=70.0)
    ap.add_argument('--leak-seconds', type=float, default=120.0)
    ap.add_argument('--charge-gauge', type=float, default=200.0)
    ap.add_argument('--pulse-s', type=float, default=0.6)
    ap.add_argument('--pulse-dp', type=float, default=70.0)
    ap.add_argument('--ppos-ceiling', type=float, default=500.0, help='[kPa gauge]')
    ap.add_argument('--frontier-timeout', type=float, default=60.0)
    ap.add_argument('--samples', type=int, default=50)
    ap.add_argument('--vol-tol', type=float, default=0.15)
    ap.add_argument('--frontier-tol', type=float, default=0.20)
    ap.add_argument('--keep', action='store_true')
    args = ap.parse_args()

    g = true_geom()
    wd = tempfile.mkdtemp(prefix='pump_fit_selftest_')
    print('=' * 74)
    print('펌프 피팅 자기검증')
    print(f'  진짜 {g}')
    print(f'  소기량 {g.v_swept*1e6:.2f} mL, 사구간 {g.v_dead*1e6:.3f} mL, '
          f'압축비 {g.compression_ratio:.1f}')
    mo, _ = pm.pump_avg(g, pm.P_ATM, pm.P_ATM - 60e3)
    print(f'  무부하 토출 {float(mo)*1e3:.4f} g/s   '
          f'능력경계(−74.3 kPa) {pm.cap_ppos(g, -74.3e3)/1e3:.1f} kPa gauge')
    print(f'  레일 V⁺={TRUE_V_POS_ML} V⁻={TRUE_V_NEG_ML} mL, '
          f'leak⁺={TRUE_K_POS} leak⁻={TRUE_K_NEG} LPM/kPa, ΔV={EXTRA_ML} mL')
    print(f'  작업 디렉터리 {wd}')
    print('  주의: 합성은 보간 테이블을 쓰므로 **크랭크 리플이 없다** → RPM 리플 추정은'
          ' 이 검증으로 확인할 수 없다 (--rpm 으로 진짜 값을 넘긴다).')
    print('=' * 74)

    try:
        for extra, rail in ((0.0, 'none'), (EXTRA_ML, 'pos'), (EXTRA_ML, 'neg')):
            print(f'\n합성: leak (ΔV={extra:.0f} → {rail})...')
            w, m = synth_leak(g, extra, rail, args.dt, args)
            print(f'  {len(w.rows)} 행')
            write(wd, f'leak_{rail}{int(extra)}', w, m)

        print('\n합성: map...')
        w, m = synth_map(g, args.dt, args)
        print(f'  {len(w.rows)} 행 ({len(args.u_grid)**2} 점)')
        write(wd, 'map_none0', w, m)

        print('\n합성: frontier (P⁻ 유지 램프)...')
        w, m = synth_frontier(g, args.dt, args)
        print(f'  {len(w.rows)} 행 ({len(args.pneg_targets)} 점)')
        write(wd, 'frontier_none0', w, m)

        cmd = [sys.executable, os.path.join(HERE, 'pump_fit_solve.py'), wd,
               '--crank-m', str(TRUE_R), '--n-piston', str(TRUE_NPIS),
               '--rpm', str(TRUE_RPM), '--samples', str(args.samples), '--no-plots']
        print(f'\n피팅 실행: {" ".join(cmd[1:])}\n' + '-' * 74)
        rc = subprocess.call(cmd)
        print('-' * 74)
        if rc != 0:
            print(f'!! pump_fit_solve.py 실패 (rc={rc})')
            return 1

        with open(os.path.join(wd, 'pump_params.yaml')) as f:
            doc = yaml.safe_load(f)
        virt = doc['/pack2/can_bridge']['ros__parameters']['Virtual']
        fitted = virt['pump']
        gf = pm.PumpGeom(delta=fitted['delta_m'], r=fitted['crank_m'], l=fitted['rod_m'],
                         rpm=fitted['rpm'], Spis=fitted['piston_area_m2'],
                         Npis=fitted['n_piston'], Cb_out=fitted['cb_out_m2'],
                         Cb_in=fitted['cb_in_m2'])

        ok = True
        print('\n[1] 레일 부피 · 누설 — 유량계가 없어 절대 스케일 앵커다')
        for rail, truth, key in (('pos', TRUE_V_POS_ML, 'line_volume_pos_ml'),
                                 ('neg', TRUE_V_NEG_ML, 'line_volume_neg_ml')):
            est = virt[key]
            err = abs(est - truth) / truth
            good = err < args.vol_tol
            ok = ok and good
            print(f'    V_{rail}: 진짜 {truth:6.1f} → 복원 {est:6.1f} mL  '
                  f'({err*100:4.1f}%)  {"PASS" if good else "FAIL"}')
        for rail, truth, key in (('pos', TRUE_K_POS, 'line_leak_pos_lpm_per_kpa'),
                                 ('neg', TRUE_K_NEG, 'line_leak_neg_lpm_per_kpa')):
            est = virt[key]
            err = abs(est - truth) / truth
            good = err < args.vol_tol
            ok = ok and good
            print(f'    leak_{rail}: 진짜 {truth:.5f} → 복원 {est:.5f} LPM/kPa  '
                  f'({err*100:4.1f}%)  {"PASS" if good else "FAIL"}')

        # ── 측정 산출물 검증 ── (컨트롤러/시뮬이 실제로 쓰는 것)
        ctrl = doc['/pack2/pp_controller']['ros__parameters']['PressureRefGen']
        fr = ctrl['pump_frontier_measured']
        mm = virt['pump_map_measured']

        print('\n[2] 측정 맵 — 추출 사슬(dP/dt·부피·누설·슬라이딩 창) 전체 검증')
        pp = np.array(mm['ppos_kpa_gauge']) * 1e3
        pn = np.array(mm['pneg_kpa_gauge']) * 1e3
        md = np.array(mm['mdot_gps']) / 1e3
        mt, _ = pm.pump_avg(g, pp + pm.P_ATM, pn + pm.P_ATM)
        mt = np.atleast_1d(mt)
        rel = np.abs(md - mt) / np.maximum(mt, 1e-9)
        good = float(np.median(rel)) < 0.10
        ok = ok and good
        print(f'    {len(md)} 점.  |측정−진짜|/진짜: 중앙값 {np.median(rel)*100:.1f}%  '
              f'90분위 {np.percentile(rel,90)*100:.1f}%  {"PASS" if good else "FAIL"} (중앙값<10%)')
        print(f'    범위 P⁺ {pp.min()/1e3:.0f}~{pp.max()/1e3:.0f} kPa, '
              f'P⁻ {pn.min()/1e3:.0f}~{pn.max()/1e3:.0f} kPa, ṁ {md.min()*1e3:.3f}~{md.max()*1e3:.3f} g/s')

        print('\n[3] 측정 능력경계 — 컨트롤러가 펌프를 쓰는 유일한 경로 (cap_ppos)')
        print('    측정 스톨은 ṁ_pump = leak⁺ 인 지점이다. 진짜 모델로 그 균형이 맞는지 본다.')
        errs = []
        for pn_k, pp_k, lb in zip(fr['pneg_kpa_gauge'], fr['ppos_max_kpa_gauge'],
                                  fr['is_lower_bound']):
            mo, _ = pm.pump_avg(g, pp_k * 1e3 + pm.P_ATM, pn_k * 1e3 + pm.P_ATM)
            leak = float(pm.leak_kgps(pp_k, TRUE_K_POS))
            e = abs(float(mo) - leak) / max(leak, 1e-9)
            errs.append(e)
            print(f'    P⁻={pn_k:+7.1f} P⁺={pp_k:7.1f} kPa gauge: ṁ_진짜={float(mo)*1e3:.4f} vs '
                  f'leak={leak*1e3:.4f} g/s  (불일치 {e*100:5.1f}%)'
                  + ('  [하한 경계]' if lb else ''))
        good = bool(errs) and float(np.median(errs)) < args.frontier_tol
        ok = ok and good
        print(f'    중앙 불일치 {np.median(errs)*100:.1f}%  '
              f'{"PASS" if good else "FAIL"} (<{args.frontier_tol*100:.0f}%)')

        print('\n[4] 기하 피팅 — 참고용 (신뢰도 낮음, 판정에 넣지 않는다)')
        pns = np.array([-90.0, -74.3, -60.0, -40.0]) * 1e3
        f_true, f_fit = pm.frontier(g, pns), pm.frontier(gf, pns)
        for q, ft, ff in zip(pns, f_true, f_fit):
            print(f'    P⁻={q/1e3:+6.1f}: 진짜 경계 {ft/1e3:7.1f} → 피팅 {ff/1e3:7.1f} kPa gauge')
        print('    5-파라미터 슬라이더-크랭크는 유량 데이터로 다중 모드다 — 측정 범위 안을')
        print('    잘 맞추면서도 데드헤드(범위 밖 외삽)가 크게 틀어진다. 능력경계는 [3] 의')
        print('    측정 테이블을 쓰는 것이 정답이다.')
        for label, f in (('소기량 [mL]', lambda x: x.v_swept * 1e6),
                         ('사구간 [mL]', lambda x: x.v_dead * 1e6),
                         ('압축비', lambda x: x.compression_ratio),
                         ('Cb_out [mm²]', lambda x: x.Cb_out * 1e6),
                         ('Cb_in [mm²]', lambda x: x.Cb_in * 1e6)):
            print(f'    {label:<14} 진짜 {f(g):>10.4g}   피팅 {f(gf):>10.4g}')

        print('\n' + '=' * 74)
        print('자기검증 ' + ('PASS' if ok else 'FAIL — 위 FAIL 항목 확인'))
        print('판정은 **측정 산출물**(부피·누설·맵·능력경계)로 한다 — 컨트롤러와 시뮬이')
        print('실제로 쓰는 것이 그것이고, 기하 피팅의 데드헤드는 외삽이라 믿을 수 없다.')
        print('=' * 74)
        rep = os.path.join(wd, 'pump_report.md')
        if os.path.exists(rep):
            print(f'\n상세 리포트: {rep}')
        return 0 if ok else 1
    finally:
        if args.keep:
            print(f'\n(--keep) 유지: {wd}')
        else:
            shutil.rmtree(wd, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
