#!/usr/bin/env python3
"""Open-loop valve characterization simulation.
   No feedback controller — valve command changes every 1 second.
   Records: u_pct [%], flow rate Q [LPM], chamber pressure P [kPa].
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv, os

# ── Physical constants (same as v2) ──────────────────────────────────────────
RGAS    = 287.0
TEMP_K  = 293.15
LPM2KG  = 0.0002155
ATM_KPA = 101.325
KAPPA   = 1.4
I_MAX   = 0.30

P_LINE_POS = 400.0   # supply pressure  [kPa]
P_LINE_NEG =  50.0   # suction pressure [kPa]

DT        = 0.002    # 500 Hz
N_SUB     = 10
VOLUME_M3 = 1e-6     # 1 mL

# ── MATLAB 13-param valve ─────────────────────────────────────────────────────
VP = [0.177485, 24.9354, 0.0918, 0.000251, 0.0,
      363318.0739, 1.6334, 0.1516, 83.5718,
      2.3474, 0.9792, 3.6058, 1.5719]

# ── Compressible flow ─────────────────────────────────────────────────────────
_PCR   = (2.0 / (KAPPA + 1.0)) ** (KAPPA / (KAPPA - 1.0))
_PHICK = np.sqrt(KAPPA * (2.0 / (KAPPA + 1.0)) ** ((KAPPA + 1.0) / (KAPPA - 1.0)))
_PHISC = np.sqrt((2.0 * KAPPA) / (KAPPA - 1.0))

def get_phi(P_in, P_out):
    if P_in < 1e-6 or P_out >= P_in:
        return 0.0
    Pr = min(1.0, P_out / P_in)
    if Pr <= _PCR:
        return _PHICK
    return _PHISC * np.sqrt(max(0.0, Pr ** (2.0/KAPPA) - Pr ** ((KAPPA+1.0)/KAPPA)))

def q_static(u_pct, z, P_in, P_out):
    if P_in <= P_out or u_pct < 0.5:
        return 0.0
    I = u_pct / 100.0 * I_MAX
    Force = max(-500.0, min(500.0, I + VP[4]*z + VP[3]*P_in - VP[2]))
    denom = max(1e-300, (1.0 + np.exp(-VP[1] * Force)) ** VP[8])
    return VP[0] / denom * P_in * get_phi(P_in, P_out)

class ValveState:
    __slots__ = ('Q', 'dQ', 'z', 'I_prev')
    def __init__(self):
        self.Q = 0.0; self.dQ = 0.0; self.z = 0.0; self.I_prev = 0.0

    def step(self, u_pct, P_in, P_out):
        # Hard check-valve: no flow if pressure differential is reversed
        if P_in <= P_out:
            self.Q = 0.0; self.dQ = 0.0
            self.I_prev = u_pct / 100.0 * I_MAX
            return 0.0
        u_pct = max(0.0, min(100.0, u_pct))
        I = u_pct / 100.0 * I_MAX
        dI = I - self.I_prev
        dz = VP[5]*dI - VP[6]*abs(dI)*self.z - VP[7]*dI*abs(self.z)
        self.z = max(-1e6, min(1e6, self.z + dz))
        Q_tgt = max(0.0, q_static(u_pct, self.z, P_in, P_out))
        wn   = VP[9]  if dI >= 0.0 else VP[11]
        zeta = VP[10] if dI >= 0.0 else VP[12]
        dt_s = DT / N_SUB
        for _ in range(N_SUB):
            self.Q  = max(0.0, self.Q  + dt_s * self.dQ)
            self.dQ = self.dQ + dt_s * (wn*wn*(Q_tgt - self.Q) - 2.0*zeta*wn*self.dQ)
        self.I_prev = I
        return self.Q

# ── Command sequences (each entry = 1 second) ─────────────────────────────────
SEQ = [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 0]   # 12 s per phase

coeff = (RGAS * TEMP_K / VOLUME_M3) * LPM2KG / 1000.0   # kPa/(LPM·s)

def run_seq(N, u_cmd, valve, P_in_fn, P_out_fn, P, P_min, P_max, sign=+1):
    """Generic single-valve step loop.
    sign=+1: flow enters chamber (fill), sign=-1: flow leaves chamber (exhaust).
    """
    t_arr = np.empty(N); P_arr = np.empty(N); Q_arr = np.empty(N); u_arr = np.empty(N)
    for k in range(N):
        t = k * DT
        u = float(u_cmd[min(int(t), len(u_cmd)-1)])
        Q = valve.step(u, P_in_fn(P), P_out_fn(P))
        P = max(P_min, min(P_max, P + sign * coeff * Q * DT))
        t_arr[k] = t; P_arr[k] = P; Q_arr[k] = Q; u_arr[k] = u
    return t_arr, P_arr, Q_arr, u_arr

def run_pos_channel():
    """
    Phase 1: fill valve 0→100→0% (supply 400 kPa → chamber, atm closed)
    Phase 2: exhaust valve 0→100→0% (chamber → atm, fill closed)
    """
    T1   = len(SEQ) * 1.0
    N1   = int(T1 / DT)
    N2   = int(T1 / DT)

    # Phase 1: fill
    v1 = ValveState()
    t1, P1, Qf1, u1 = run_seq(N1, SEQ, v1,
                               lambda P: P_LINE_POS, lambda P: P,
                               ATM_KPA, 50.0, P_LINE_POS)   # cannot exceed supply
    P_after1 = P1[-1]

    # Phase 2: exhaust from whatever pressure Phase 1 left
    v2 = ValveState()
    t2, P2, Qf2, u2 = run_seq(N2, SEQ, v2,
                               lambda P: P, lambda P: ATM_KPA,
                               P_after1, 50.0, P_LINE_POS, sign=-1)  # exhaust leaves chamber
    t2 = t2 + T1

    t   = np.concatenate([t1,  t2])
    P   = np.concatenate([P1,  P2])
    Qa  = np.concatenate([Qf1, np.zeros(N2)])   # Phase 1 valve (fill)
    Qb  = np.concatenate([np.zeros(N1), Qf2])   # Phase 2 valve (exhaust)
    ua  = np.concatenate([u1,  np.zeros(N2)])
    ub  = np.concatenate([np.zeros(N1), u2])
    return dict(t=t, P=P, Qa=Qa, Qb=Qb, ua=ua, ub=ub, T1=T1)

def run_neg_channel():
    """
    Phase 1: suction valve 0→100→0% (chamber → suction 50 kPa, recovery closed)
    Phase 2: recovery valve 0→100→0% (atm → chamber, suction closed)
    """
    T1 = len(SEQ) * 1.0
    N1 = int(T1 / DT)
    N2 = int(T1 / DT)

    # Phase 1: suction draws P below atm
    # P_in = P (chamber), P_out = P_LINE_NEG (50 kPa)
    v1 = ValveState()
    t1, P1, Qs1, u1 = run_seq(N1, SEQ, v1,
                               lambda P: P, lambda P: P_LINE_NEG,
                               ATM_KPA, 30.0, 200.0)
    # flow Qs1 DECREASES P → negate contribution
    # Redo with proper sign: Q flows chamber→suction, so dP/dt = -coeff*Q
    v1b = ValveState()
    P1b = np.empty(N1)
    Q1b = np.empty(N1)
    u1b = np.empty(N1)
    P = ATM_KPA
    for k in range(N1):
        t = k * DT
        u = float(SEQ[min(int(t), len(SEQ)-1)])
        Q = v1b.step(u, P, P_LINE_NEG)              # P_in=chamber, P_out=suction
        P = max(P_LINE_NEG, min(ATM_KPA, P - coeff * Q * DT))  # min=suction line
        P1b[k] = P; Q1b[k] = Q; u1b[k] = u
    t1b = np.arange(N1) * DT
    P_after1 = P1b[-1]

    # Phase 2: recover back to atm from low pressure
    v2 = ValveState()
    P2b = np.empty(N2)
    Q2b = np.empty(N2)
    u2b = np.empty(N2)
    P = P_after1
    for k in range(N2):
        t = k * DT
        u = float(SEQ[min(int(t), len(SEQ)-1)])
        Q = v2.step(u, ATM_KPA, P)              # P_in=atm, P_out=chamber
        P = max(P_LINE_NEG, min(ATM_KPA, P + coeff * Q * DT))  # max=atm
        P2b[k] = P; Q2b[k] = Q; u2b[k] = u
    t2b = np.arange(N2) * DT + T1

    t   = np.concatenate([t1b,  t2b])
    P   = np.concatenate([P1b,  P2b])
    Qa  = np.concatenate([Q1b,  np.zeros(N2)])
    Qb  = np.concatenate([np.zeros(N1), Q2b])
    ua  = np.concatenate([u1b,  np.zeros(N2)])
    ub  = np.concatenate([np.zeros(N1), u2b])
    return dict(t=t, P=P, Qa=Qa, Qb=Qb, ua=ua, ub=ub, T1=T1)

def save_csv(dp, dn, outdir):
    for name, d, lbl_a, lbl_b in [
        ('pos', dp, 'Q_fill_lpm',   'Q_exha_lpm'),
        ('neg', dn, 'Q_suction_lpm', 'Q_recovery_lpm'),
    ]:
        path = f'{outdir}/char_{name}.csv'
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'u_ph1_pct', 'u_ph2_pct', lbl_a, lbl_b, 'P_kPa'])
            for i in range(len(d['t'])):
                w.writerow([f"{d['t'][i]:.4f}",
                            f"{d['ua'][i]:.1f}", f"{d['ub'][i]:.1f}",
                            f"{d['Qa'][i]:.6f}", f"{d['Qb'][i]:.6f}",
                            f"{d['P'][i]:.3f}"])
        print(f'Saved CSV: {path}')

def plot_all(dp, dn, outdir):
    T1 = dp['T1']

    fig, axes = plt.subplots(3, 2, figsize=(15, 11), sharex='col')
    fig.suptitle(
        'Open-loop Valve Characterization  |  MATLAB 13-param valve  |  V = 1 mL chamber\n'
        'Phase 1 (0–12 s): sweep fill/suction valve  |  '
        'Phase 2 (12–24 s): sweep exhaust/recovery valve',
        fontsize=11, fontweight='bold')

    cfg = [
        dict(d=dp, ch='ch0  Positive',
             ca='steelblue', cb='seagreen',
             la='u_fill (supply→chamber)', lb='u_exha (chamber→atm)',
             lQa='Q fill [LPM]', lQb='Q exhaust [LPM]',
             P_lines=[(P_LINE_POS, 'orange', 'Supply 400 kPa'),
                      (ATM_KPA,   'gray',   'ATM 101 kPa')]),
        dict(d=dn, ch='ch6  Negative',
             ca='tomato', cb='darkorange',
             la='u_suct (chamber→50 kPa)', lb='u_fill (atm→chamber)',
             lQa='Q suction [LPM]', lQb='Q recovery [LPM]',
             P_lines=[(ATM_KPA,   'gray', 'ATM 101 kPa'),
                      (P_LINE_NEG, 'cyan', 'Suction 50 kPa')]),
    ]

    for col, c in enumerate(cfg):
        d  = c['d']
        t  = d['t']
        ax0, ax1, ax2 = axes[0, col], axes[1, col], axes[2, col]

        # Row 0: valve commands
        ax0.step(t, d['ua'], where='post', color=c['ca'], lw=1.4, label=c['la'])
        ax0.step(t, d['ub'], where='post', color=c['cb'], lw=1.4, label=c['lb'])
        ax0.axvline(T1, color='red', lw=1.2, ls='--', label='phase change')
        ax0.set_ylabel('Valve cmd [%]')
        ax0.set_title(f"{c['ch']} — Valve Command")
        ax0.legend(fontsize=8, loc='upper right')
        ax0.set_ylim(-5, 115)

        # Row 1: flow rate
        ax1.plot(t, d['Qa'], color=c['ca'], lw=1.1, label=c['lQa'])
        ax1.plot(t, d['Qb'], color=c['cb'], lw=1.1, label=c['lQb'])
        ax1.axvline(T1, color='red', lw=1.2, ls='--')
        ax1.set_ylabel('Flow rate [LPM]')
        ax1.set_title(f"{c['ch']} — Flow Rate Q")
        ax1.legend(fontsize=8, loc='upper right')

        # Row 2: pressure
        ax2.plot(t, d['P'], color=c['ca'], lw=1.5, label='P chamber')
        for P_ref, clr, lbl in c['P_lines']:
            ax2.axhline(P_ref, color=clr, lw=0.9, ls='--', label=lbl)
        ax2.axvline(T1, color='red', lw=1.2, ls='--')
        ax2.set_ylabel('Pressure [kPa]')
        ax2.set_xlabel('Time [s]')
        ax2.set_title(f"{c['ch']} — Chamber Pressure")
        ax2.legend(fontsize=8, loc='upper right')

        step_ticks = np.arange(0, t[-1] + 0.5, 1.0)
        for ax in (ax0, ax1, ax2):
            ax.grid(True, alpha=0.3)
            for xt in step_ticks:
                ax.axvline(xt, color='lightgray', lw=0.4, ls=':')

    plt.tight_layout()
    path = f'{outdir}/valve_char.png'
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved plot: {path}')

def print_summary(d, label, T_offset):
    Q_key = 'Qa' if T_offset == 0.0 else 'Qb'
    u_key = 'ua' if T_offset == 0.0 else 'ub'
    phase  = 'Phase 1 (fill/suction)' if T_offset == 0.0 else 'Phase 2 (exha/recovery)'
    print(f'\n{label}  {phase}:')
    print(f'  {"u_pct":>6}  {"Q_peak [LPM]":>14}  {"Q_ss [LPM]":>12}  {"P_end [kPa]":>12}')
    for i, u in enumerate(SEQ):
        t0 = T_offset + i + 0.2
        t1 = T_offset + i + 0.95
        mask = (d['t'] >= t0) & (d['t'] < t1)
        if mask.any():
            Q_peak = d[Q_key][mask].max()
            Q_ss   = float(np.mean(d[Q_key][mask][-50:]))  # last 100ms
            P_end  = d['P'][mask][-1]
            print(f'  {u:>6}%  {Q_peak:>14.4f}  {Q_ss:>12.4f}  {P_end:>12.1f}')

if __name__ == '__main__':
    outdir = 'results_char'
    os.makedirs(outdir, exist_ok=True)

    print('Running positive channel...')
    dp = run_pos_channel()

    print('Running negative channel...')
    dn = run_neg_channel()

    save_csv(dp, dn, outdir)
    plot_all(dp, dn, outdir)

    print_summary(dp, 'Positive', 0.0)
    print_summary(dp, 'Positive', dp['T1'])
    print_summary(dn, 'Negative', 0.0)
    print_summary(dn, 'Negative', dn['T1'])
