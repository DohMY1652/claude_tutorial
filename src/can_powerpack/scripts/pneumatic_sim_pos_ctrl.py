#!/usr/bin/env python3
"""
Position control simulation — vertical series-cooperative BiPAM actuator.

Actuator:  50 mm diameter, 150 mm max extension per chamber
Load:      10 kg hanging mass (mg = 98.1 N)
Mount:     Actuator top-fixed, load hangs below.
           Contraction (positive pressure+vacuum) lifts the load upward.

Control structure:
  [y_ref] → PID → F_req → equal split (F_pos=F_neg=F_req/2)
           → P_pos_sp, P_neg_sp  → 1st-order pressure dynamics
           → F_pos, F_neg → F_net = F_pos + F_neg (series cooperative)
           → mechanical dynamics → y

Position range:  Y_LOW = 15 mm  (near bottom, 10 % margin)
                 Y_HIGH = 135 mm (near top,   10 % margin)

Trajectory:  3 s settle at Y_LOW → 5 s/5 s step cycle (3.5 cycles) → 40 s total
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv, os

# ─── Physical constants ───────────────────────────────────────────────────────
ATM_KPA    = 101.325   # kPa  atmospheric pressure
P_LINE_POS = 400.0     # kPa  positive supply (compressed air)
P_LINE_NEG =  50.0     # kPa  negative supply (vacuum)
GRAVITY    = 9.81      # m/s²

# ─── Actuator geometry (50 mm diameter, 150 mm per chamber) ──────────────────
R_ACT = 0.025                    # m   effective radius (50 mm / 2)
L_CH  = 0.150                    # m   max extension length per chamber
A_EFF = np.pi * R_ACT ** 2      # m²  effective piston area  ≈ 1.963 × 10⁻³ m²

# Theoretical force limits at supply pressures
F_POS_MAX = (P_LINE_POS - ATM_KPA) * 1e3 * A_EFF   # ≈ 586 N
F_NEG_MAX = (ATM_KPA  - P_LINE_NEG) * 1e3 * A_EFF  # ≈ 101 N

# With EQUAL distribution (F_pos = F_neg = F/2), negative chamber is limiting:
F_NET_MAX = 2.0 * min(F_POS_MAX, F_NEG_MAX)          # ≈ 202 N

# ─── Load ─────────────────────────────────────────────────────────────────────
MASS   = 10.0               # kg
WEIGHT = MASS * GRAVITY     # N   ≈ 98.1 N  (< F_NET_MAX → liftable with good margin)
DAMP_B = 40.0               # N·s/m  viscous friction (bellows + seal)

# ─── Position setpoints ───────────────────────────────────────────────────────
Y_LOW  = 0.015   # m   target low  (physical range 0 – L_CH)
Y_HIGH = 0.135   # m   target high

# ─── Pressure dynamics (first-order lag representing valve + line) ────────────
TAU_P = 0.12     # s   pressure time constant (≈ 120 ms)

# ─── PID gains ────────────────────────────────────────────────────────────────
# Plant (linear approx): m ẍ + b ẋ = ΔF_net
# Closed-loop: m ẍ + (b + Kd) ẋ + Kp x = 0
#   ωn = √(Kp/m) = √(400/10) ≈ 6.32 rad/s → T ≈ 1.0 s
#   ζ  = (b + Kd) / (2 m ωn) = (40+100)/126.4 ≈ 1.11  (slightly over-damped)
KP = 400.0   # N/m
KI =  25.0   # N/(m·s)
KD = 100.0   # N·s/m  (acts on measured velocity, avoids derivative kick)

# ─── Simulation parameters ────────────────────────────────────────────────────
DT      = 0.002    # s   500 Hz
T_TOTAL = 40.0     # s
OUT_DIR = 'results_pos_ctrl'

# ─── Force ↔ Pressure mappings ───────────────────────────────────────────────
def p_sp_pos(F_N):
    """F_pos [N] → P_pos setpoint [kPa abs].  Clamped to [ATM, P_LINE_POS]."""
    return float(np.clip(F_N / A_EFF / 1e3 + ATM_KPA, ATM_KPA, P_LINE_POS))

def p_sp_neg(F_N):
    """F_neg [N] → P_neg setpoint [kPa abs].  Clamped to [P_LINE_NEG, ATM]."""
    return float(np.clip(ATM_KPA - F_N / A_EFF / 1e3, P_LINE_NEG, ATM_KPA))

def f_pos(P_kPa):
    """P_pos [kPa abs] → contraction force [N] (≥ 0)."""
    return max(0.0, (P_kPa - ATM_KPA) * 1e3 * A_EFF)

def f_neg(P_kPa):
    """P_neg [kPa abs] → suction force [N] (≥ 0)."""
    return max(0.0, (ATM_KPA - P_kPa) * 1e3 * A_EFF)

def f_net(P_pos, P_neg):
    """Series-cooperative net force [N] (both chambers same direction)."""
    return f_pos(P_pos) + f_neg(P_neg)

# ─── Reference trajectory ─────────────────────────────────────────────────────
def y_ref(t):
    """Step between Y_LOW and Y_HIGH.  3 s warm-up at Y_LOW, then 5 s / 5 s."""
    if t < 3.0:
        return Y_LOW
    return Y_HIGH if (t - 3.0) % 10.0 < 5.0 else Y_LOW

# ─── Simulation ───────────────────────────────────────────────────────────────
def run():
    N = int(T_TOTAL / DT)

    # Initial state: at equilibrium holding at Y_LOW
    F0    = WEIGHT / 2.0          # each chamber provides half of gravity load
    P_pos = p_sp_pos(F0)          # ≈ 126 kPa
    P_neg = p_sp_neg(F0)          # ≈  77 kPa
    y     = Y_LOW
    v     = 0.0
    int_e = 0.0                   # integral starts at 0 (feedforward handles gravity)

    # Anti-windup bounds for integral
    int_lo = (0.0       - WEIGHT) / KI
    int_hi = (F_NET_MAX - WEIGHT) / KI

    # Pre-allocate output arrays
    t_a   = np.empty(N); y_a   = np.empty(N); yr_a  = np.empty(N)
    v_a   = np.empty(N); a_a   = np.empty(N)
    Pp_a  = np.empty(N); Pn_a  = np.empty(N)
    Ppsp_a= np.empty(N); Pnsp_a= np.empty(N)
    Fp_a  = np.empty(N); Fn_a  = np.empty(N); Fnet_a= np.empty(N)
    Freq_a= np.empty(N); Fe_a  = np.empty(N)

    for k in range(N):
        t  = k * DT
        yr = y_ref(t)
        e  = yr - y

        # ── PID position controller ───────────────────────────────────────────
        int_e  = np.clip(int_e + e * DT, int_lo, int_hi)
        F_req  = float(np.clip(
            WEIGHT + KP * e + KI * int_e - KD * v,   # feedforward + PID
            0.0, F_NET_MAX
        ))
        F_each = F_req / 2.0    # equal distribution: F_pos = F_neg = F_req / 2

        # ── Pressure setpoints from force targets ────────────────────────────
        Pp_sp = p_sp_pos(F_each)
        Pn_sp = p_sp_neg(F_each)

        # ── Pressure dynamics (1st-order lag) ────────────────────────────────
        P_pos += (Pp_sp - P_pos) * DT / TAU_P
        P_neg += (Pn_sp - P_neg) * DT / TAU_P
        P_pos  = float(np.clip(P_pos, ATM_KPA,    P_LINE_POS))
        P_neg  = float(np.clip(P_neg, P_LINE_NEG, ATM_KPA))

        # ── Actual forces ─────────────────────────────────────────────────────
        Fp   = f_pos(P_pos)
        Fn   = f_neg(P_neg)
        Fnet = Fp + Fn

        # ── Mechanical dynamics ───────────────────────────────────────────────
        acc  = (Fnet - WEIGHT - DAMP_B * v) / MASS
        v   += acc * DT
        y   += v   * DT

        # Physical end-stops
        if y <= 0.0:
            y = 0.0;  v = max(0.0, v)
        elif y >= L_CH:
            y = L_CH; v = min(0.0, v)

        # ── Store ─────────────────────────────────────────────────────────────
        t_a[k]    = t;     y_a[k]    = y;      yr_a[k]   = yr
        v_a[k]    = v;     a_a[k]    = acc
        Pp_a[k]   = P_pos; Pn_a[k]   = P_neg
        Ppsp_a[k] = Pp_sp; Pnsp_a[k] = Pn_sp
        Fp_a[k]   = Fp;    Fn_a[k]   = Fn;     Fnet_a[k] = Fnet
        Freq_a[k] = F_req; Fe_a[k]   = F_each

    return dict(
        t=t_a, y=y_a, y_ref=yr_a, v=v_a, acc=a_a,
        P_pos=Pp_a,  P_neg=Pn_a,
        P_pos_sp=Ppsp_a, P_neg_sp=Pnsp_a,
        F_pos=Fp_a, F_neg=Fn_a, F_net=Fnet_a,
        F_req=Freq_a, F_each=Fe_a,
    )

# ─── Valve command proxy (derived from pressure error for visualization) ───────
def valve_cmds(d):
    P_rng_pos = P_LINE_POS - ATM_KPA     # kPa  fill range
    P_rng_neg = ATM_KPA   - P_LINE_NEG   # kPa  suction range
    dp_pos = np.array(d['P_pos_sp']) - np.array(d['P_pos'])
    dp_neg = np.array(d['P_neg_sp']) - np.array(d['P_neg'])
    u_pos_fill = np.clip( dp_pos / P_rng_pos * 100,  0, 100)   # fill  positive ch
    u_pos_ex   = np.clip(-dp_pos / P_rng_pos * 100,  0, 100)   # exhaust positive ch
    u_neg_suck = np.clip(-dp_neg / P_rng_neg * 100,  0, 100)   # suction negative ch
    u_neg_vent = np.clip( dp_neg / P_rng_neg * 100,  0, 100)   # vent   negative ch
    return u_pos_fill, u_pos_ex, u_neg_suck, u_neg_vent

# ─── Plot ─────────────────────────────────────────────────────────────────────
def plot(d, path):
    t  = d['t']
    yr = np.array(d['y_ref'])   * 1e3   # mm
    y  = np.array(d['y'])       * 1e3   # mm
    e  = (yr - y)                        # mm  error

    u_pf, u_pe, u_ns, u_nv = valve_cmds(d)

    fig, axes = plt.subplots(5, 1, figsize=(12, 18), sharex=True)
    fig.suptitle(
        'BiPAM Position Control  (50 mm diam., 150 mm/ch, 10 kg load)\n'
        'Controller: PID + equal force split + 1st-order pressure lag',
        fontsize=11
    )

    # ── Row 0: Position tracking ──────────────────────────────────────────────
    ax = axes[0]
    ax.plot(t, yr, 'k--', lw=1.4, label='y_ref')
    ax.plot(t, y,  'b-',  lw=1.8, label='y (actual)')
    ax.fill_between(t, yr, y, alpha=0.18, color='orange', label='error band')
    ax.axhline(Y_LOW  * 1e3, color='gray', lw=0.7, ls=':')
    ax.axhline(Y_HIGH * 1e3, color='gray', lw=0.7, ls=':')
    ax.set_ylabel('Position [mm]')
    ax.set_title('Position Tracking')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    ax2.plot(t, e, 'r-', lw=0.9, alpha=0.6, label='error [mm]')
    ax2.set_ylabel('Error [mm]', color='r')
    ax2.tick_params(axis='y', colors='r')
    ax2.legend(loc='lower right', fontsize=9)

    # ── Row 1: Forces ─────────────────────────────────────────────────────────
    ax = axes[1]
    ax.plot(t, d['F_req'],  'k--', lw=1.4, label='F_req (controller output)')
    ax.plot(t, d['F_net'],  'g-',  lw=1.8, label='F_net = F_pos + F_neg (actual)')
    ax.plot(t, d['F_pos'],  'b-',  lw=1.0, alpha=0.7, label='F_pos (positive ch)')
    ax.plot(t, d['F_neg'],  'r-',  lw=1.0, alpha=0.7, label='F_neg (negative ch)')
    ax.axhline(WEIGHT,      color='purple', lw=1.0, ls=':', label=f'Weight ({WEIGHT:.1f} N)')
    ax.axhline(F_NET_MAX,   color='gray',   lw=0.8, ls='--', label=f'F_max ({F_NET_MAX:.0f} N)')
    ax.set_ylabel('Force [N]')
    ax.set_title('Force: Required vs Actual  (F_pos = F_neg = F_req/2)')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.25)

    # ── Row 2: Pressures ──────────────────────────────────────────────────────
    ax = axes[2]
    ax.plot(t, d['P_pos_sp'], 'b--', lw=1.2, alpha=0.7, label='P_pos setpoint')
    ax.plot(t, d['P_pos'],    'b-',  lw=1.8,             label='P_pos (actual)')
    ax.plot(t, d['P_neg_sp'], 'r--', lw=1.2, alpha=0.7, label='P_neg setpoint')
    ax.plot(t, d['P_neg'],    'r-',  lw=1.8,             label='P_neg (actual)')
    ax.axhline(ATM_KPA,    color='gray',  lw=0.8, ls=':', label=f'ATM ({ATM_KPA:.1f} kPa)')
    ax.axhline(P_LINE_POS, color='blue',  lw=0.6, ls=':', alpha=0.4, label='Supply+ (400 kPa)')
    ax.axhline(P_LINE_NEG, color='red',   lw=0.6, ls=':', alpha=0.4, label='Supply− (50 kPa)')
    ax.set_ylabel('Pressure [kPa abs]')
    ax.set_title('Chamber Pressures  (setpoint vs actual)')
    ax.legend(loc='upper right', fontsize=8, ncol=3)
    ax.grid(True, alpha=0.25)

    # ── Row 3: Valve commands ─────────────────────────────────────────────────
    ax = axes[3]
    ax.plot(t, u_pf,  'b-',  lw=1.4, label='pos fill   (%)')
    ax.plot(t, -u_pe, 'b--', lw=1.0, alpha=0.7, label='pos exhaust (−%)')
    ax.plot(t, u_ns,  'r-',  lw=1.4, label='neg suction (%)')
    ax.plot(t, -u_nv, 'r--', lw=1.0, alpha=0.7, label='neg vent   (−%)')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.set_ylabel('Valve activity [%]')
    ax.set_title('Valve Commands  (derived from pressure error)')
    ax.legend(loc='upper right', fontsize=8, ncol=2)
    ax.grid(True, alpha=0.25)

    # ── Row 4: Velocity ───────────────────────────────────────────────────────
    ax = axes[4]
    ax.plot(t, np.array(d['v']) * 1e3, 'g-', lw=1.4, label='velocity [mm/s]')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.set_ylabel('Velocity [mm/s]')
    ax.set_title('Load Velocity')
    ax.set_xlabel('Time [s]')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.25)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'  saved → {path}')

# ─── CSV ──────────────────────────────────────────────────────────────────────
def save_csv(d, path):
    keys = ['t', 'y_ref', 'y', 'v', 'acc',
            'P_pos_sp', 'P_pos', 'P_neg_sp', 'P_neg',
            'F_req', 'F_each', 'F_pos', 'F_neg', 'F_net']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(keys)
        for i in range(len(d['t'])):
            w.writerow([f'{d[k][i]:.6g}' for k in keys])
    print(f'  saved → {path}')

# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    print('Running position control simulation ...')
    print(f'  Actuator: dia={R_ACT*2*1000:.0f} mm, L_ch={L_CH*1000:.0f} mm')
    print(f'  A_eff   = {A_EFF*1e6:.2f} mm²   ({A_EFF*1e4:.2f} cm²)')
    print(f'  F_neg_max = {F_NEG_MAX:.1f} N,  F_pos_max = {F_POS_MAX:.1f} N')
    print(f'  F_net_max (equal split) = {F_NET_MAX:.1f} N')
    print(f'  Load    : {MASS:.1f} kg  (mg = {WEIGHT:.1f} N)')
    print(f'  y_low={Y_LOW*1e3:.0f} mm,  y_high={Y_HIGH*1e3:.0f} mm\n')

    d = run()

    # Print summary — measure SS error only in the settled middle of each hold
    y_mm  = np.array(d['y']) * 1e3
    yr_mm = np.array(d['y_ref']) * 1e3
    e_mm  = yr_mm - y_mm
    t_arr = d['t']
    # Check SS error during t=5.5–7.5s (Y_HIGH hold) and t=10.5–12.5s (Y_LOW hold)
    def ss_err(t0, t1):
        mask = (t_arr >= t0) & (t_arr < t1)
        return np.max(np.abs(e_mm[mask]))
    ss_high = ss_err(5.5, 7.5)
    ss_low  = ss_err(10.5, 12.5)
    # Rise time: first time position crosses 90% of step size after t=3
    step = (Y_HIGH - Y_LOW) * 1e3  # mm
    thr  = Y_LOW * 1e3 + 0.9 * step
    idx  = np.where((t_arr > 3.0) & (y_mm >= thr))[0]
    t_rise = (t_arr[idx[0]] - 3.0) if len(idx) > 0 else float('nan')
    print(f"  Rise time (10→90 %) : {t_rise:.2f} s")
    print(f"  SS error @ Y_HIGH   : {ss_high:.2f} mm  (t=5.5–7.5 s)")
    print(f"  SS error @ Y_LOW    : {ss_low:.2f} mm   (t=10.5–12.5 s)")
    print(f"  Peak overshoot      : {np.max(y_mm) - Y_HIGH*1e3:.2f} mm above Y_HIGH")
    print(f"  F_net range         : {np.min(d['F_net']):.1f} N – {np.max(d['F_net']):.1f} N")
    print(f"  P_pos range         : {np.min(d['P_pos']):.1f} – {np.max(d['P_pos']):.1f} kPa")
    print(f"  P_neg range         : {np.min(d['P_neg']):.1f} – {np.max(d['P_neg']):.1f} kPa")

    save_csv(d, f'{OUT_DIR}/pos_ctrl.csv')
    plot(d, f'{OUT_DIR}/pos_ctrl.png')
    print('\nDone.')

if __name__ == '__main__':
    main()
