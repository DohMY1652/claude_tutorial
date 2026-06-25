#!/usr/bin/env python3
"""
Open-loop pneumatic force simulation.

Valve openings are prescribed as a time schedule (no feedback).
Physics and force model are identical to pneumatic_sim_standalone.py.
Goal: observe how valve → pressure → BiPAM output force chain behaves.

Schedule (T_TOTAL = 50 s):
  Phase 0  0– 3 s   idle at ATM
  Phase 1  3–10 s   positive micro-valve  30 %  → low contraction force
  Phase 2 10–17 s   positive micro-valve  70 %  → high contraction force
  Phase 3 17–20 s   positive exhaust      80 %  → release positive pressure
  Phase 4 20–23 s   idle
  Phase 5 23–30 s   negative micro-valve  50 %  → moderate extension force
  Phase 6 30–37 s   negative micro+macro  80/60 % → high extension force
  Phase 7 37–40 s   negative vent (atm)   80 %  → release negative pressure
  Phase 8 40–47 s   both channels active  pos 50 %, neg 50 % → antagonistic
  Phase 9 47–50 s   exhaust both          → back to ATM
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv, os

# ── Physical constants (same as pneumatic_sim_standalone.py) ─────────────────
RGAS    = 287.0
TEMP_K  = 293.15
LPM2KG  = 0.0002155
ATM_KPA = 101.325

P_LINE_POS   = 400.0
P_LINE_NEG   =  50.0
P_LINE_MACRO = 400.0
P_MACRO_NEG  =  50.0

DT      = 0.002   # 500 Hz
N_SUB   = 10
T_TOTAL = 50.0

# ── Valve flow model parameters (BASE_POS / BASE_NEG from standalone) ────────
K0_P, K1_P, K2_P = 0.00022, 0.010, 0.75
K0_N, K1_N, K2_N = 0.00022, 0.008, 0.70
VOL_M3 = 1e-6     # 1 mL chamber

# ── BiPAM force model ─────────────────────────────────────────────────────────
# Source: Park, J.H. et al., Mechatronics 97, 2023 (DOI 10.1016/j.mechatronics.2023.103099)
# Virtual work: F = (P_gauge) × (dV/dL) = P_gauge × π r²
#
# Positive pressure → bellows contraction:
#   F_pos = (P - P_atm) × 1e3 × π r_m²      r_m back-calc from F_max = 124.46 N @ 400 kPa
#
# Negative pressure → inner-tube extension:
#   F_neg = (P_atm - P) × 1e3 × π r_i²      r_i back-calc from F_max = 122.80 N @  50 kPa
#
# Length correction (polynomial):  F(ε) = F₀ × (1 + c₁ε + c₂ε²)
#   ε_pos = (L₀−L)/L₀  ∈ [0, 0.751]   (paper max contraction 75.1 %)
#   ε_neg = (L−L₀)/L₀  ∈ [0, 4.021]   (paper max extension   402.1 %)

_BIPAM = dict(
    L0          = 0.15,      # m  max extension per chamber (50 mm dia spec)
    r_m         = 0.025,     # m  effective radius = 50 mm / 2  (positive)
    r_i         = 0.025,     # m  effective radius = 50 mm / 2  (negative)
    c_pos       = (-0.80, 0.30),
    c_neg       = ( 0.50, -0.10),
    eps_max_pos = 0.751,
    eps_max_neg = 4.021,
)

def bipam_force_pos(P_kPa, L_m=None):
    dP = max(0.0, P_kPa - ATM_KPA) * 1e3
    F0 = dP * np.pi * _BIPAM['r_m'] ** 2
    if L_m is None or F0 == 0.0:
        return F0
    eps = np.clip((_BIPAM['L0'] - L_m) / _BIPAM['L0'], 0.0, _BIPAM['eps_max_pos'])
    c1, c2 = _BIPAM['c_pos']
    return max(0.0, F0 * (1.0 + c1 * eps + c2 * eps ** 2))

def bipam_force_neg(P_kPa, L_m=None):
    dP = max(0.0, ATM_KPA - P_kPa) * 1e3
    F0 = dP * np.pi * _BIPAM['r_i'] ** 2
    if L_m is None or F0 == 0.0:
        return F0
    eps = np.clip((L_m - _BIPAM['L0']) / _BIPAM['L0'], 0.0, _BIPAM['eps_max_neg'])
    c1, c2 = _BIPAM['c_neg']
    return max(0.0, F0 * (1.0 + c1 * eps + c2 * eps ** 2))

def bipam_net_force(P_pos_kPa, P_neg_kPa, L_m=None):
    return bipam_force_pos(P_pos_kPa, L_m) + bipam_force_neg(P_neg_kPa, L_m)

# ── Open-loop valve schedule ──────────────────────────────────────────────────

def valve_schedule(t):
    """
    Returns (u_mi_p, u_ma_p, u_at_p, u_mi_n, u_ma_n, u_at_n) in [%].
    Positive ch: mi=micro-fill, ma=macro-fill, at=exhaust-to-atm.
    Negative ch: mi=micro-exhaust-to-suction, ma=macro-exhaust, at=vent-from-atm.

    Dead-zone thresholds (from calc_flow model params):
      pos fill      u > 66.2 %   (P_LINE_POS=400 kPa)
      pos exhaust   u > ~70  %   (P_in = built-up pressure)
      neg suction   u > 84.7 %   (P_in = chamber ≈ ATM)
      neg vent      u > 84.7 %   (P_in = ATM)
    """
    u_mi_p = u_ma_p = u_at_p = 0.0
    u_mi_n = u_ma_n = u_at_n = 0.0

    if   t < 3.0:   # Phase 0: idle at ATM
        pass
    elif t < 10.0:  # Phase 1: pos 70% (just above dead zone → slow fill)
        u_mi_p = 70.0
    elif t < 17.0:  # Phase 2: pos 90% → fast fill, high contraction force
        u_mi_p = 90.0
    elif t < 20.0:  # Phase 3: exhaust positive (80% > dead zone at P_pos>150 kPa)
        u_at_p = 80.0
    elif t < 23.0:  # Phase 4: idle
        pass
    elif t < 30.0:  # Phase 5: neg 90% (above 84.7% dead zone → moderate suction)
        u_mi_n = 90.0
    elif t < 37.0:  # Phase 6: neg micro+macro 100% → deep suction, high extension force
        u_mi_n = 100.0
        u_ma_n = 100.0
    elif t < 40.0:  # Phase 7: vent negative back to ATM (90% > dead zone)
        u_at_n = 90.0
    elif t < 47.0:  # Phase 8: both active → antagonistic (pos 85%, neg 90%)
        u_mi_p = 85.0
        u_mi_n = 90.0
    else:           # Phase 9: exhaust both to ATM
        u_at_p = 80.0
        u_at_n = 90.0

    return u_mi_p, u_ma_p, u_at_p, u_mi_n, u_ma_n, u_at_n

# ── Flow model (identical to v1) ─────────────────────────────────────────────

def calc_flow(u_pct, P_in, P_out, k0, k1, k2):
    u_pct = max(0.0, min(100.0, u_pct))
    if P_in <= P_out:
        return 0.0
    root = (2.0 * (P_in - P_out) * P_in) ** 0.5
    return max(0.0, min(100.0, (k0 * P_in + k1 * u_pct - k2) * root))

def dPdt_pos(P, u_mi, u_ma, u_at):
    net = (calc_flow(u_mi, P_LINE_POS,  P, K0_P, K1_P, K2_P)
         + calc_flow(u_ma, P_LINE_MACRO, P, K0_P, K1_P, K2_P)
         - calc_flow(u_at, P, ATM_KPA,   K0_P, K1_P, K2_P))
    return (RGAS * TEMP_K / VOL_M3) * LPM2KG * net / 1000.0

def dPdt_neg(P, u_mi, u_ma, u_at):
    net = (calc_flow(u_at, ATM_KPA,    P, K0_N, K1_N, K2_N)
         - calc_flow(u_mi, P, P_LINE_NEG,  K0_N, K1_N, K2_N)
         - calc_flow(u_ma, P, P_MACRO_NEG, K0_N, K1_N, K2_N))
    return (RGAS * TEMP_K / VOL_M3) * LPM2KG * net / 1000.0

# ── Simulation ────────────────────────────────────────────────────────────────

def run():
    N  = int(T_TOTAL / DT)
    dt_sub = DT / N_SUB

    P_pos = ATM_KPA
    P_neg = ATM_KPA

    t_arr   = np.empty(N)
    Pp_arr  = np.empty(N); Pn_arr  = np.empty(N)
    ump_arr = np.empty(N); uap_arr = np.empty(N); umap_arr = np.empty(N)
    umn_arr = np.empty(N); uan_arr = np.empty(N); uman_arr = np.empty(N)
    Fp_arr  = np.empty(N); Fn_arr  = np.empty(N); Fnet_arr = np.empty(N)

    for k in range(N):
        t = k * DT
        u_mi_p, u_ma_p, u_at_p, u_mi_n, u_ma_n, u_at_n = valve_schedule(t)

        for _ in range(N_SUB):
            dp = dPdt_pos(P_pos, u_mi_p, u_ma_p, u_at_p)
            dn = dPdt_neg(P_neg, u_mi_n, u_ma_n, u_at_n)
            P_pos = max(50.0,  min(800.0, P_pos + dp * dt_sub))
            P_neg = max(10.0,  min(110.0, P_neg + dn * dt_sub))

        t_arr[k]   = t
        Pp_arr[k]  = P_pos;  Pn_arr[k]  = P_neg
        ump_arr[k] = u_mi_p; uap_arr[k] = u_at_p; umap_arr[k] = u_ma_p
        umn_arr[k] = u_mi_n; uan_arr[k] = u_at_n; uman_arr[k] = u_ma_n
        Fp_arr[k]  = bipam_force_pos(P_pos)
        Fn_arr[k]  = bipam_force_neg(P_neg)
        Fnet_arr[k]= bipam_net_force(P_pos, P_neg)

    return dict(t=t_arr,
                P_pos=Pp_arr,  P_neg=Pn_arr,
                u_mi_p=ump_arr, u_ma_p=umap_arr, u_at_p=uap_arr,
                u_mi_n=umn_arr, u_ma_n=uman_arr, u_at_n=uan_arr,
                F_pos=Fp_arr, F_neg=Fn_arr, F_net=Fnet_arr)

# ── Plot ──────────────────────────────────────────────────────────────────────

PHASE_EDGES = [3, 10, 17, 20, 23, 30, 37, 40, 47]
PHASE_LABELS = [
    'P0\nidle', 'P1\npos\n70%', 'P2\npos\n90%', 'P3\nexh+\n80%',
    'P4\nidle', 'P5\nneg\n90%', 'P6\nneg\n100+\n100%', 'P7\nvent−\n90%',
    'P8\nboth\n85/90%', 'P9\nexh\nboth',
]

def _phase_bands(ax, ymin, ymax):
    edges = [0.0] + PHASE_EDGES + [T_TOTAL]
    colors = ['#f0f4ff', '#dff0df', '#fff0d0', '#f0f4ff',
              '#f0f4ff', '#ffe0e0', '#ffc0c0', '#f0f4ff',
              '#e8d8ff', '#f0f4ff']
    for i in range(len(edges) - 1):
        ax.axvspan(edges[i], edges[i + 1], alpha=0.25,
                   color=colors[i % len(colors)], lw=0)
    for xv in PHASE_EDGES:
        ax.axvline(xv, color='#aaaaaa', lw=0.7, ls=':')

def plot(d, path):
    t = d['t']
    fig, axes = plt.subplots(4, 1, figsize=(14, 16), sharex=True)
    fig.suptitle(
        'BiPAM Open-Loop Simulation  —  valve schedule → pressure → output force\n'
        '(no feedback; parameters from pneumatic_sim_standalone.py)',
        fontsize=12, fontweight='bold')

    # ── Row 0: valve inputs (positive channel) ───────────────────
    ax = axes[0]
    ax.plot(t, d['u_mi_p'],  'b-',  lw=1.4, label='u_mi (fill micro)')
    ax.plot(t, d['u_ma_p'],  'b--', lw=1.2, label='u_ma (fill macro)', alpha=0.7)
    ax.plot(t, d['u_at_p'],  'b:',  lw=1.2, label='u_at (exhaust)',    alpha=0.9)
    ax.plot(t, d['u_mi_n'],  'r-',  lw=1.4, label='u_mi (suction)')
    ax.plot(t, d['u_ma_n'],  'r--', lw=1.2, label='u_ma (macro suc)', alpha=0.7)
    ax.plot(t, d['u_at_n'],  'r:',  lw=1.2, label='u_at (vent atm)',  alpha=0.9)
    _phase_bands(ax, 0, 100)
    ax.set_ylabel('Valve opening [%]')
    ax.set_title('Valve Inputs  (blue = positive ch, red = negative ch)')
    ax.set_ylim(-5, 105)
    ax.legend(loc='upper right', fontsize=8, ncol=3)
    ax.grid(True, alpha=0.25)
    # phase labels
    edges = [0.0] + PHASE_EDGES + [T_TOTAL]
    for i, lbl in enumerate(PHASE_LABELS):
        mid = (edges[i] + edges[i + 1]) / 2
        ax.text(mid, 96, lbl, ha='center', va='top', fontsize=6.5,
                color='#444444', multialignment='center')

    # ── Row 1: chamber pressures ─────────────────────────────────
    ax = axes[1]
    ax.plot(t, d['P_pos'], 'b-', lw=1.6, label='P_pos (positive ch)')
    ax.plot(t, d['P_neg'], 'r-', lw=1.6, label='P_neg (negative ch)')
    ax.axhline(ATM_KPA, color='gray', lw=0.9, ls='--', label=f'ATM = {ATM_KPA:.1f} kPa')
    _phase_bands(ax, 0, 500)
    ax.set_ylabel('Pressure [kPa abs]')
    ax.set_title('Chamber Pressures')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.25)

    # ── Row 2: individual forces ──────────────────────────────────
    ax = axes[2]
    ax.plot(t, d['F_pos'], 'b-', lw=1.6, label='F_pos (contraction force)')
    ax.plot(t, d['F_neg'], 'r-', lw=1.6, label='F_neg (extension  force)')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    _phase_bands(ax, 0, 150)
    ax.set_ylabel('Force [N]')
    ax.set_title('BiPAM Output Forces  (F_pos: positive ch,  F_neg: negative ch)')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.25)

    # ── Row 3: net antagonistic force ─────────────────────────────
    ax = axes[3]
    ax.fill_between(t, d['F_net'], 0,
                    where=np.array(d['F_net']) > 0, color='blue', alpha=0.15,
                    label='output force (both active)')
    ax.plot(t, d['F_net'], 'k-', lw=1.8, label='F_net = F_pos + F_neg')
    ax.axhline(0, color='gray', lw=0.9, ls='--')
    _phase_bands(ax, 0, 300)
    ax.set_ylabel('Net Force [N]')
    ax.set_title('Series Cooperative Net Force  (pos + neg → same direction)')
    ax.set_xlabel('Time [s]')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.25)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  saved → {path}')

# ── CSV ───────────────────────────────────────────────────────────────────────

def save_csv(d, path):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s',
                    'P_pos_kPa', 'P_neg_kPa',
                    'u_mi_pos', 'u_ma_pos', 'u_at_pos',
                    'u_mi_neg', 'u_ma_neg', 'u_at_neg',
                    'F_pos_N', 'F_neg_N', 'F_net_N'])
        for i in range(len(d['t'])):
            w.writerow([
                f"{d['t'][i]:.4f}",
                f"{d['P_pos'][i]:.4f}", f"{d['P_neg'][i]:.4f}",
                f"{d['u_mi_p'][i]:.1f}", f"{d['u_ma_p'][i]:.1f}", f"{d['u_at_p'][i]:.1f}",
                f"{d['u_mi_n'][i]:.1f}", f"{d['u_ma_n'][i]:.1f}", f"{d['u_at_n'][i]:.1f}",
                f"{d['F_pos'][i]:.3f}", f"{d['F_neg'][i]:.3f}", f"{d['F_net'][i]:.3f}",
            ])
    print(f'  saved → {path}')

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    out = 'results_openloop'
    os.makedirs(out, exist_ok=True)

    print('Running open-loop simulation ...')
    d = run()

    # ── per-phase force summary ───────────────────────────────────
    t = d['t']
    phases = [
        ('P0 idle',          0.0,  3.0),
        ('P1 pos 70%',       3.0, 10.0),
        ('P2 pos 90%',      10.0, 17.0),
        ('P3 exhaust+ 80%', 17.0, 20.0),
        ('P4 idle',         20.0, 23.0),
        ('P5 neg 90%',      23.0, 30.0),
        ('P6 neg 100+100%', 30.0, 37.0),
        ('P7 vent− 90%',    37.0, 40.0),
        ('P8 both 85/90%',  40.0, 47.0),
        ('P9 exhaust all',  47.0, 50.0),
    ]

    print(f"\n{'Phase':<18} {'P_pos_end':>10} {'P_neg_end':>10} "
          f"{'F_pos_end':>10} {'F_neg_end':>10} {'F_net_end':>10}")
    print('-' * 72)
    for name, t0, t1 in phases:
        mask = (t >= t0) & (t < t1)
        if not mask.any():
            continue
        idx = np.where(mask)[0][-1]
        print(f"{name:<18} "
              f"{d['P_pos'][idx]:>10.2f} {d['P_neg'][idx]:>10.2f} "
              f"{d['F_pos'][idx]:>10.2f} {d['F_neg'][idx]:>10.2f} "
              f"{d['F_net'][idx]:>10.2f}")

    peak_pos = d['F_pos'].max()
    peak_neg = d['F_neg'].max()
    peak_net_pos = d['F_net'].max()
    peak_net_neg = d['F_net'].min()
    print(f"\nPeak F_pos : {peak_pos:.2f} N")
    print(f"Peak F_neg : {peak_neg:.2f} N")
    print(f"Peak F_net : {peak_net_pos:.2f} N  (series cooperative, both chambers same direction)")

    save_csv(d, f'{out}/openloop.csv')
    plot(d, f'{out}/openloop.png')
    print('\nDone.')
