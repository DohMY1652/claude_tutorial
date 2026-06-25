#!/usr/bin/env python3
"""
Reel angle control simulation — wire-coupled BiPAM actuator.

Mechanism:
  BiPAM contraction → wire tension → reel (R=50mm) rotates → lifts rod loads
  Two rods rigidly attached to reel axis, loads at their ends.

Control structure:
  [theta_ref] → PID → tau_req → F_req = tau_req / R_REEL
              → equal split (F_pos = F_neg = F_req/2)
              → P_sp  → 1st-order pressure lag
              → F_net → tau_net = F_net * R_REEL
              → rotational dynamics → theta

Sensor (real hardware):
  AS5600 I2C (Arduino) → USB serial → AngleSerialReader
  Run with: USE_REAL_SENSOR = True  and set SERIAL_PORT below.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv, os

# ─── Hardware toggle ──────────────────────────────────────────────────────────
USE_REAL_SENSOR = False          # True = read from Arduino via USB serial
SERIAL_PORT     = '/dev/ttyUSB0'
SERIAL_BAUD     = 115200

# ─── Physical constants ───────────────────────────────────────────────────────
ATM_KPA    = 101.325
P_LINE_POS = 400.0
P_LINE_NEG =  50.0
GRAVITY    = 9.81

# ─── Actuator geometry (50 mm diameter) ──────────────────────────────────────
R_ACT  = 0.025                       # m  effective piston radius
A_EFF  = np.pi * R_ACT ** 2         # m²  ≈ 1.963e-3 m²
F_POS_MAX = (P_LINE_POS - ATM_KPA) * 1e3 * A_EFF   # ≈ 586 N
F_NEG_MAX = (ATM_KPA   - P_LINE_NEG) * 1e3 * A_EFF  # ≈ 101 N
F_NET_MAX  = 2.0 * min(F_POS_MAX, F_NEG_MAX)         # ≈ 202 N (equal split limit)

# ─── Reel geometry ────────────────────────────────────────────────────────────
R_REEL  = 0.050   # m  reel drum radius (50 mm)

# ─── Rod + load parameters  ← SET FROM HARDWARE ──────────────────────────────
L_ROD1  = 0.050   # m  50 mm rod
L_ROD2  = 0.100   # m  100 mm rod
M_LOAD1 = 0.500   # kg  load at 50 mm rod end  ← 측정 후 수정
M_LOAD2 = 0.500   # kg  load at 100 mm rod end ← 측정 후 수정

# Moment of inertia (point masses on rods + reel disc estimate)
J_REEL  = 1e-4    # kg·m²  reel body (estimate; measure or compute from CAD)
J_TOTAL = M_LOAD1 * L_ROD1**2 + M_LOAD2 * L_ROD2**2 + J_REEL

# Rotational damping (bearing + seal friction)
B_ROT   = 0.05    # N·m·s/rad  (tune from step response)

# ─── Gravity torque ───────────────────────────────────────────────────────────
# theta = 0: rods hanging straight down (stable equilibrium).
# theta increases as wire lifts the rods upward.
# Gravity always opposes positive rotation:
#   tau_grav(theta) = (M1*g*L1 + M2*g*L2) * sin(theta)
#
# If rods are mounted at an angular offset from the wire direction, add PHI_OFFSET.
PHI_OFFSET = 0.0  # rad  adjust from hardware geometry (usually 0)

_GRAV_COEFF = M_LOAD1 * GRAVITY * L_ROD1 + M_LOAD2 * GRAVITY * L_ROD2

def gravity_torque(theta):
    """Gravity restoring torque [N·m] acting against upward rotation (positive theta)."""
    return _GRAV_COEFF * np.sin(theta + PHI_OFFSET)

# ─── Angle range (limited by actuator stroke: 120 mm / R_REEL = 2.4 rad) ─────
STROKE_M    = 0.120           # m  actuator max stroke
THETA_MIN   = 0.0             # rad  mechanical lower stop
THETA_MAX   = STROKE_M / R_REEL   # rad  ≈ 2.4 rad ≈ 137.5°
THETA_LOW   = np.deg2rad(10)  # rad  lower setpoint (near hanging, away from singularity)
THETA_HIGH  = np.deg2rad(90)  # rad  upper setpoint (horizontal, max gravity torque)

# ─── Pressure dynamics ────────────────────────────────────────────────────────
TAU_P = 0.12   # s  pressure lag (valve + line)

# ─── PID gains ───────────────────────────────────────────────────────────────
# Tuned for: J_TOTAL ≈ 6.35e-3 kg·m², tau_grav_max ≈ 0.74 N·m, tau_act_max ≈ 10 N·m
# Target closed-loop bandwidth ≈ 15 rad/s (ωn = √(Kp/J) = √(1.5/6.35e-3) ≈ 15.4)
KP = 1.500   # N·m/rad
KI = 0.200   # N·m/(rad·s)
KD = 0.180   # N·m·s/rad

# ─── Simulation parameters ────────────────────────────────────────────────────
DT      = 0.002    # s  500 Hz
T_TOTAL = 40.0     # s
OUT_DIR = 'results_reel_ctrl'

# ─── Pressure ↔ force mappings (same as pos_ctrl) ────────────────────────────
def p_sp_pos(F_N):
    return float(np.clip(F_N / A_EFF / 1e3 + ATM_KPA, ATM_KPA, P_LINE_POS))

def p_sp_neg(F_N):
    return float(np.clip(ATM_KPA - F_N / A_EFF / 1e3, P_LINE_NEG, ATM_KPA))

def f_pos(P_kPa):
    return max(0.0, (P_kPa - ATM_KPA) * 1e3 * A_EFF)

def f_neg(P_kPa):
    return max(0.0, (ATM_KPA - P_kPa) * 1e3 * A_EFF)

# ─── Reference trajectory ─────────────────────────────────────────────────────
def theta_ref(t):
    """Step between THETA_LOW and THETA_HIGH. 3 s settle, then 5 s / 5 s."""
    if t < 3.0:
        return THETA_LOW
    return THETA_HIGH if (t - 3.0) % 10.0 < 5.0 else THETA_LOW

# ─── Simulation ───────────────────────────────────────────────────────────────
def run(sensor=None):
    """
    sensor: AngleSerialReader instance for real hardware, None for simulation.
    """
    N = int(T_TOTAL / DT)

    # Initial equilibrium: hold at THETA_LOW against gravity
    tau_grav_init = gravity_torque(THETA_LOW)
    F0     = tau_grav_init / R_REEL / 2.0   # each chamber
    P_pos  = p_sp_pos(F0)
    P_neg  = p_sp_neg(F0)
    theta  = THETA_LOW
    omega  = 0.0
    int_e  = 0.0

    int_lo = -F_NET_MAX * R_REEL / KI
    int_hi =  F_NET_MAX * R_REEL / KI

    # Pre-allocate
    t_a    = np.empty(N); th_a   = np.empty(N); thr_a  = np.empty(N)
    om_a   = np.empty(N); al_a   = np.empty(N)
    Pp_a   = np.empty(N); Pn_a   = np.empty(N)
    Ppsp_a = np.empty(N); Pnsp_a = np.empty(N)
    Fp_a   = np.empty(N); Fn_a   = np.empty(N); Fnet_a = np.empty(N)
    Freq_a = np.empty(N); tgrav_a= np.empty(N); tact_a = np.empty(N)

    for k in range(N):
        t  = k * DT

        # Read sensor: real hardware or simulation
        if sensor is not None:
            theta_meas = np.deg2rad(sensor.get_angle_deg())
        else:
            theta_meas = theta

        tr = theta_ref(t)
        e  = tr - theta_meas

        # ── PID (outputs torque demand [N·m]) ─────────────────────────────────
        int_e  = float(np.clip(int_e + e * DT, int_lo, int_hi))
        tau_pid = KP * e + KI * int_e - KD * omega

        # Gravity feedforward (cancel static load)
        tau_ff  = gravity_torque(theta_meas)
        tau_req = float(np.clip(tau_pid + tau_ff, 0.0, F_NET_MAX * R_REEL))

        # ── Force demand and pressure setpoints ───────────────────────────────
        F_req   = tau_req / R_REEL
        F_each  = F_req / 2.0

        Pp_sp   = p_sp_pos(F_each)
        Pn_sp   = p_sp_neg(F_each)

        # ── Pressure dynamics (1st-order lag) ────────────────────────────────
        P_pos  += (Pp_sp - P_pos) * DT / TAU_P
        P_neg  += (Pn_sp - P_neg) * DT / TAU_P
        P_pos   = float(np.clip(P_pos, ATM_KPA,    P_LINE_POS))
        P_neg   = float(np.clip(P_neg, P_LINE_NEG, ATM_KPA))

        # ── Actual forces and net torque ──────────────────────────────────────
        Fp      = f_pos(P_pos)
        Fn      = f_neg(P_neg)
        Fnet    = Fp + Fn
        tau_act = Fnet * R_REEL
        tau_grav= gravity_torque(theta)

        # ── Rotational dynamics ───────────────────────────────────────────────
        alpha  = (tau_act - tau_grav - B_ROT * omega) / J_TOTAL
        omega += alpha * DT
        theta += omega * DT

        # Mechanical end stops
        if theta <= THETA_MIN:
            theta = THETA_MIN; omega = max(0.0, omega)
        elif theta >= THETA_MAX:
            theta = THETA_MAX; omega = min(0.0, omega)

        # ── Store ─────────────────────────────────────────────────────────────
        t_a[k]    = t;      th_a[k]   = theta;   thr_a[k]  = tr
        om_a[k]   = omega;  al_a[k]   = alpha
        Pp_a[k]   = P_pos;  Pn_a[k]   = P_neg
        Ppsp_a[k] = Pp_sp;  Pnsp_a[k] = Pn_sp
        Fp_a[k]   = Fp;     Fn_a[k]   = Fn;      Fnet_a[k] = Fnet
        Freq_a[k] = F_req;  tgrav_a[k]= tau_grav; tact_a[k] = tau_act

    return dict(
        t=t_a, theta=th_a, theta_ref=thr_a, omega=om_a, alpha=al_a,
        P_pos=Pp_a,    P_neg=Pn_a,
        P_pos_sp=Ppsp_a, P_neg_sp=Pnsp_a,
        F_pos=Fp_a,    F_neg=Fn_a,    F_net=Fnet_a,
        F_req=Freq_a,  tau_grav=tgrav_a, tau_act=tact_a,
    )

# ─── Plot ─────────────────────────────────────────────────────────────────────
def plot(d, path):
    t    = d['t']
    thr  = np.rad2deg(d['theta_ref'])
    th   = np.rad2deg(d['theta'])
    e_deg = thr - th

    fig, axes = plt.subplots(5, 1, figsize=(12, 18), sharex=True)
    fig.suptitle(
        'BiPAM Reel Angle Control  (R_reel=50mm, rods: 50mm+100mm)\n'
        'PID + gravity feedforward + 1st-order pressure lag',
        fontsize=11
    )

    ax = axes[0]
    ax.plot(t, thr, 'k--', lw=1.4, label='theta_ref')
    ax.plot(t, th,  'b-',  lw=1.8, label='theta (actual)')
    ax.fill_between(t, thr, th, alpha=0.18, color='orange')
    ax.set_ylabel('Angle [deg]'); ax.set_title('Angle Tracking')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.25)
    ax2 = ax.twinx()
    ax2.plot(t, e_deg, 'r-', lw=0.9, alpha=0.6)
    ax2.set_ylabel('Error [deg]', color='r')
    ax2.tick_params(axis='y', colors='r')

    ax = axes[1]
    ax.plot(t, d['tau_act'],  'g-',  lw=1.8, label='tau_act = F_net × R')
    ax.plot(t, d['tau_grav'], 'r--', lw=1.2, label='tau_gravity')
    ax.set_ylabel('Torque [N·m]'); ax.set_title('Torques')
    ax.legend(fontsize=9); ax.grid(True, alpha=0.25)

    ax = axes[2]
    ax.plot(t, d['P_pos_sp'], 'b--', lw=1.2, alpha=0.7, label='P_pos setpoint')
    ax.plot(t, d['P_pos'],    'b-',  lw=1.8,             label='P_pos actual')
    ax.plot(t, d['P_neg_sp'], 'r--', lw=1.2, alpha=0.7, label='P_neg setpoint')
    ax.plot(t, d['P_neg'],    'r-',  lw=1.8,             label='P_neg actual')
    ax.axhline(ATM_KPA, color='gray', lw=0.8, ls=':')
    ax.set_ylabel('Pressure [kPa]'); ax.set_title('Chamber Pressures')
    ax.legend(fontsize=8, ncol=2); ax.grid(True, alpha=0.25)

    ax = axes[3]
    ax.plot(t, d['F_req'],  'k--', lw=1.4, label='F_req')
    ax.plot(t, d['F_net'],  'g-',  lw=1.8, label='F_net')
    ax.plot(t, d['F_pos'],  'b-',  lw=1.0, alpha=0.7, label='F_pos')
    ax.plot(t, d['F_neg'],  'r-',  lw=1.0, alpha=0.7, label='F_neg')
    ax.set_ylabel('Force [N]'); ax.set_title('Forces')
    ax.legend(fontsize=8, ncol=2); ax.grid(True, alpha=0.25)

    ax = axes[4]
    ax.plot(t, np.rad2deg(d['omega']), 'g-', lw=1.4, label='omega [deg/s]')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.set_ylabel('Angular velocity [deg/s]'); ax.set_title('Angular Velocity')
    ax.set_xlabel('Time [s]'); ax.legend(fontsize=9); ax.grid(True, alpha=0.25)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()

# ─── CSV ──────────────────────────────────────────────────────────────────────
def save_csv(d, path):
    keys = ['t', 'theta_ref', 'theta', 'omega', 'alpha',
            'P_pos_sp', 'P_pos', 'P_neg_sp', 'P_neg',
            'F_req', 'F_pos', 'F_neg', 'F_net', 'tau_act', 'tau_grav']
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        # Convert theta/omega to degrees for readability
        header = keys[:]
        header[1] = 'theta_ref_deg'; header[2] = 'theta_deg'; header[3] = 'omega_degps'
        w.writerow(header)
        for i in range(len(d['t'])):
            row = []
            for k in keys:
                v = d[k][i]
                if k in ('theta_ref', 'theta'):
                    v = np.rad2deg(v)
                elif k == 'omega':
                    v = np.rad2deg(v)
                row.append(f'{v:.6g}')
            w.writerow(row)

# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    sensor = None
    if USE_REAL_SENSOR:
        from serial_angle_reader import AngleSerialReader
        sensor = AngleSerialReader(SERIAL_PORT, SERIAL_BAUD)
        print(f'Sensor connected: {SERIAL_PORT}')

    print(f'R_REEL={R_REEL*1000:.0f}mm  '
          f'J_TOTAL={J_TOTAL*1e3:.2f}g·m²  '
          f'F_NET_MAX={F_NET_MAX:.1f}N  '
          f'tau_max={F_NET_MAX*R_REEL:.2f}N·m')
    print(f'THETA range: {np.rad2deg(THETA_LOW):.1f}° – {np.rad2deg(THETA_HIGH):.1f}°  '
          f'(actuator stroke limit: {np.rad2deg(THETA_MAX):.1f}°)')
    tau_grav_hi = gravity_torque(THETA_HIGH)
    print(f'tau_gravity @ THETA_HIGH: {tau_grav_hi:.3f} N·m  '
          f'(F_wire needed: {tau_grav_hi/R_REEL:.1f} N)\n')

    d = run(sensor)

    th_deg  = np.rad2deg(d['theta'])
    thr_deg = np.rad2deg(d['theta_ref'])
    e_deg   = thr_deg - th_deg
    t_arr   = d['t']

    def ss_err(t0, t1):
        mask = (t_arr >= t0) & (t_arr < t1)
        return np.max(np.abs(e_deg[mask]))

    step_deg = np.rad2deg(THETA_HIGH - THETA_LOW)
    thr_90   = np.rad2deg(THETA_LOW) + 0.9 * step_deg
    idx      = np.where((t_arr > 3.0) & (th_deg >= thr_90))[0]
    t_rise   = (t_arr[idx[0]] - 3.0) if len(idx) > 0 else float('nan')

    print(f'  Rise time (10→90%)        : {t_rise:.2f} s')
    print(f'  SS error @ THETA_HIGH     : {ss_err(5.5, 7.5):.2f} deg  (t=5.5–7.5s)')
    print(f'  SS error @ THETA_LOW      : {ss_err(10.5, 12.5):.2f} deg  (t=10.5–12.5s)')
    print(f'  Peak overshoot            : {np.max(th_deg) - np.rad2deg(THETA_HIGH):.2f} deg')
    print(f'  P_pos range               : {np.min(d["P_pos"]):.1f} – {np.max(d["P_pos"]):.1f} kPa')
    print(f'  P_neg range               : {np.min(d["P_neg"]):.1f} – {np.max(d["P_neg"]):.1f} kPa')

    save_csv(d, f'{OUT_DIR}/reel_ctrl.csv')
    plot(d,       f'{OUT_DIR}/reel_ctrl.png')
    print(f'  saved → {OUT_DIR}/')

    if sensor is not None:
        sensor.close()

if __name__ == '__main__':
    main()
