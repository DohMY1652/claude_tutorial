#!/usr/bin/env python3
"""
Valve characterization v2 — multiple step scenarios.
Chamber: 50 mL (MATLAB hardware range).
Shows micro / macro / combined valve steps every 1 s (or defined hold times).
No feedback controller — pure open-loop.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# ── Physical constants ────────────────────────────────────────────────────────
RGAS       = 287.0
TEMP_K     = 293.15
LPM2KG     = 0.0002155
ATM_KPA    = 101.325
KAPPA      = 1.4
I_MAX      = 0.30

P_LINE_POS    = 400.0    # positive supply  [kPa]
P_LINE_MACRO  = 400.0    # macro supply (same)
P_LINE_NEG    =  50.0    # suction line     [kPa]
P_LINE_MACRO_NEG = 50.0  # macro suction (same)

DT        = 0.002        # 500 Hz
N_SUB     = 10
VOLUME_M3 = 5e-5         # 50 mL  ← enlarged from 1 mL

coeff = (RGAS * TEMP_K / VOLUME_M3) * LPM2KG / 1000.0   # kPa/(LPM·s) ≈ 362.6

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
    return _PHISC * np.sqrt(max(0.0, Pr**(2.0/KAPPA) - Pr**((KAPPA+1.0)/KAPPA)))

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
        if P_in <= P_out:                        # hard check-valve: no reverse flow
            self.Q = 0.0; self.dQ = 0.0
            self.I_prev = u_pct / 100.0 * I_MAX
            return 0.0
        u_pct = max(0.0, min(100.0, u_pct))
        I     = u_pct / 100.0 * I_MAX
        dI    = I - self.I_prev
        dz    = VP[5]*dI - VP[6]*abs(dI)*self.z - VP[7]*dI*abs(self.z)
        self.z = max(-1e6, min(1e6, self.z + dz))
        Q_tgt = max(0.0, q_static(u_pct, self.z, P_in, P_out))
        wn    = VP[9]  if dI >= 0.0 else VP[11]
        zeta  = VP[10] if dI >= 0.0 else VP[12]
        dt_s  = DT / N_SUB
        for _ in range(N_SUB):
            self.Q  = max(0.0, self.Q  + dt_s * self.dQ)
            self.dQ = self.dQ + dt_s * (wn*wn*(Q_tgt - self.Q) - 2.0*zeta*wn*self.dQ)
        self.I_prev = I
        return self.Q

# ── Scenario runner ───────────────────────────────────────────────────────────
# Positive channel: (dt, u_mi, u_ma, u_at)
#   Q_mi: P_LINE_POS→P  (fill +)
#   Q_ma: P_LINE_MACRO→P (fill +)
#   Q_at: P→ATM_KPA     (exhaust −)
#
# Negative channel: (dt, u_mi, u_ma, u_at)
#   Q_mi: P→P_LINE_NEG  (suction −)
#   Q_ma: P→P_LINE_MACRO_NEG (suction −)
#   Q_at: ATM_KPA→P     (fill from atm +)

def run_pos(cmds, P0=ATM_KPA):
    N = sum(max(1, int(dt / DT)) for dt, *_ in cmds)
    t_arr  = np.empty(N); P_arr  = np.empty(N)
    Qmi    = np.empty(N); Qma    = np.empty(N); Qat = np.empty(N)
    umi    = np.empty(N); uma    = np.empty(N); uat = np.empty(N)
    vmi = ValveState(); vma = ValveState(); vat_v = ValveState()
    P = P0; k = 0
    for dt, u_mi, u_ma, u_at in cmds:
        for _ in range(max(1, int(dt / DT))):
            qmi = vmi.step(u_mi,  P_LINE_POS,   P)
            qma = vma.step(u_ma,  P_LINE_MACRO,  P)
            qat = vat_v.step(u_at, P,            ATM_KPA)
            Q_net = qmi + qma - qat
            P = max(ATM_KPA - 10, min(P_LINE_POS, P + coeff * Q_net * DT))
            t_arr[k] = k * DT; P_arr[k] = P
            Qmi[k] = qmi; Qma[k] = qma; Qat[k] = qat
            umi[k] = u_mi; uma[k] = u_ma; uat[k] = u_at
            k += 1
    return dict(t=t_arr[:k], P=P_arr[:k],
                Qmi=Qmi[:k], Qma=Qma[:k], Qat=Qat[:k],
                umi=umi[:k], uma=uma[:k], uat=uat[:k])

def run_neg(cmds, P0=ATM_KPA):
    N = sum(max(1, int(dt / DT)) for dt, *_ in cmds)
    t_arr  = np.empty(N); P_arr  = np.empty(N)
    Qmi    = np.empty(N); Qma    = np.empty(N); Qat = np.empty(N)
    umi    = np.empty(N); uma    = np.empty(N); uat = np.empty(N)
    vmi = ValveState(); vma = ValveState(); vat_v = ValveState()
    P = P0; k = 0
    for dt, u_mi, u_ma, u_at in cmds:
        for _ in range(max(1, int(dt / DT))):
            qmi = vmi.step(u_mi,  P,      P_LINE_NEG)
            qma = vma.step(u_ma,  P,      P_LINE_MACRO_NEG)
            qat = vat_v.step(u_at, ATM_KPA, P)
            Q_net = qat - qmi - qma          # suction removes, atm-fill adds
            P = max(P_LINE_NEG, min(ATM_KPA, P + coeff * Q_net * DT))
            t_arr[k] = k * DT; P_arr[k] = P
            Qmi[k] = qmi; Qma[k] = qma; Qat[k] = qat
            umi[k] = u_mi; uma[k] = u_ma; uat[k] = u_at
            k += 1
    return dict(t=t_arr[:k], P=P_arr[:k],
                Qmi=Qmi[:k], Qma=Qma[:k], Qat=Qat[:k],
                umi=umi[:k], uma=uma[:k], uat=uat[:k])

# ── Scenario definitions ──────────────────────────────────────────────────────
# Each entry: (title, runner, cmds, P0)
# Positive fill scenarios — 1 s per step

def pos_cmds_sweep(u_vals, valve='mi'):
    """Sweep u_vals, 1 s each, then 2 s close. valve: 'mi'|'ma'|'both'"""
    out = []
    for u in u_vals:
        umi = u if valve in ('mi', 'both') else 0
        uma = u if valve in ('ma', 'both') else 0
        out.append((1, umi, uma, 0))
    out.append((2, 0, 0, 0))
    return out

SWEEP_U = [0, 20, 25, 28, 30, 32, 35, 40, 50, 70, 100]

POS_SCENARIOS = [
    # ── fill valve tests ──────────────────────────────────────────────────────
    dict(title='① Micro valve only\n(0→20→25→28→30→32→35→40→50→70→100%, 1s each)',
         run=run_pos,
         cmds=pos_cmds_sweep(SWEEP_U, 'mi'),
         P0=ATM_KPA, Pref=P_LINE_POS),

    dict(title='② Macro valve only\n(same sweep as micro)',
         run=run_pos,
         cmds=pos_cmds_sweep(SWEEP_U, 'ma'),
         P0=ATM_KPA, Pref=P_LINE_POS),

    dict(title='③ Micro + Macro simultaneous\n(both get same u at each step)',
         run=run_pos,
         cmds=pos_cmds_sweep(SWEEP_U, 'both'),
         P0=ATM_KPA, Pref=P_LINE_POS),

    # ── combined timing tests ─────────────────────────────────────────────────
    dict(title='④ Micro 35% base, Macro added at t=5s\n'
               '(micro only 0→5s, micro+macro 5→10s, micro only 10→13s)',
         run=run_pos,
         cmds=[(1,0,0,0),(4,35,0,0),(5,35,35,0),(3,35,0,0),(2,0,0,0)],
         P0=ATM_KPA, Pref=P_LINE_POS),

    dict(title='⑤ Staggered steps: micro 30→35→40% + macro 0→30→40%\n'
               '(micro steps first, macro activates 2s later)',
         run=run_pos,
         cmds=[(1,0,0,0),
               (2,30,0,0),(2,35,0,0),(2,40,0,0),   # micro only
               (2,30,30,0),(2,35,30,0),(2,40,40,0), # add macro
               (2,0,0,0)],
         P0=ATM_KPA, Pref=P_LINE_POS),

    # ── exhaust test (start from near-max, sweep exhaust) ─────────────────────
    dict(title='⑥ Exhaust sweep\n(fill with mi+ma=35% for 4s, then exhaust 0→25→30→35→40→50→70%)',
         run=run_pos,
         cmds=[(4,35,35,0),(1,0,0,0)] +
              [(1,0,0,u) for u in [0,25,30,35,40,50,70,100]] +
              [(1,0,0,0)],
         P0=ATM_KPA, Pref=P_LINE_POS),
]

NEG_SCENARIOS = [
    # ── suction valve tests ───────────────────────────────────────────────────
    dict(title='① Micro suction only\n(0→40→45→50→55→60→65→70→80→100%, 1s each)',
         run=run_neg,
         cmds=[(1, u, 0, 0) for u in [0,40,45,50,55,60,65,70,80,100]] + [(2,0,0,0)],
         P0=ATM_KPA, Pref=P_LINE_NEG),

    dict(title='② Macro suction only\n(same sweep)',
         run=run_neg,
         cmds=[(1, 0, u, 0) for u in [0,40,45,50,55,60,65,70,80,100]] + [(2,0,0,0)],
         P0=ATM_KPA, Pref=P_LINE_NEG),

    dict(title='③ Micro 60% base, Macro added at t=4s',
         run=run_neg,
         cmds=[(1,0,0,0),(3,60,0,0),(4,60,60,0),(3,60,0,0),(2,0,0,0)],
         P0=ATM_KPA, Pref=P_LINE_NEG),

    dict(title='④ Recovery sweep\n(suction to min with mi+ma=60%, then fill from atm 0→50→60→70%)',
         run=run_neg,
         cmds=[(3,60,60,0),(1,0,0,0)] +
              [(1,0,0,u) for u in [0,40,50,55,60,65,70,80,100]] +
              [(1,0,0,0)],
         P0=ATM_KPA, Pref=ATM_KPA),
]

# ── Plot helper ───────────────────────────────────────────────────────────────
STEP_LW = 0.4

def plot_scenario_grid(scenarios, fname, sup_title, P_lines):
    ncols = len(scenarios)
    fig, axes = plt.subplots(3, ncols, figsize=(4.5*ncols, 10))
    if ncols == 1:
        axes = axes[:, None]
    fig.suptitle(sup_title, fontsize=11, fontweight='bold')

    for col, sc in enumerate(scenarios):
        d    = sc['run'](sc['cmds'], P0=sc['P0'])
        t    = d['t']
        clr  = f'C{col}'
        ax0, ax1, ax2 = axes[0, col], axes[1, col], axes[2, col]

        # Row 0: valve commands ─────────────────────────────────────────
        ax0.step(t, d['umi'], where='post', color='steelblue', lw=1.2, label='u_micro')
        ax0.step(t, d['uma'], where='post', color='seagreen',  lw=1.2, label='u_macro')
        ax0.step(t, d['uat'], where='post', color='tomato',    lw=1.2, label='u_exhaust')
        ax0.set_ylim(-5, 115)
        ax0.set_ylabel('Valve cmd [%]', fontsize=8)
        ax0.set_title(sc['title'], fontsize=8, loc='left', pad=3)
        ax0.legend(fontsize=7, ncol=3, loc='upper right',
                   handlelength=1.2, columnspacing=0.8)

        # Row 1: flow rates ─────────────────────────────────────────────
        Q_total = d['Qmi'] + d['Qma'] - d['Qat']
        ax1.plot(t, d['Qmi'], color='steelblue', lw=0.8, alpha=0.7, label='Q micro')
        ax1.plot(t, d['Qma'], color='seagreen',  lw=0.8, alpha=0.7, label='Q macro')
        ax1.plot(t, d['Qat'], color='tomato',    lw=0.8, alpha=0.7, label='Q exhaust')
        ax1.plot(t, Q_total, color='black',      lw=1.3, label='Q net', zorder=5)
        ax1.set_ylabel('Flow rate [LPM]', fontsize=8)
        ax1.legend(fontsize=7, ncol=2, loc='upper right',
                   handlelength=1.2, columnspacing=0.8)

        # Row 2: pressure ───────────────────────────────────────────────
        ax2.plot(t, d['P'], color=clr, lw=1.8, label='P chamber')
        for P_ref, lclr, llbl in P_lines:
            ax2.axhline(P_ref, color=lclr, lw=0.9, ls='--', label=llbl)
        ax2.set_ylabel('Pressure [kPa]', fontsize=8)
        ax2.set_xlabel('Time [s]', fontsize=8)
        ax2.legend(fontsize=7, loc='upper right')

        # Shared formatting ─────────────────────────────────────────────
        T_total = t[-1]
        tick_ts = np.arange(0, T_total + 0.5, 1.0)
        for ax in (ax0, ax1, ax2):
            ax.grid(True, alpha=0.3)
            ax.set_xlim(0, T_total)
            for xt in tick_ts:
                ax.axvline(xt, color='lightgray', lw=STEP_LW, ls=':')
            ax.tick_params(labelsize=8)

    plt.tight_layout()
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {fname}')

def plot_P_overlay(scenarios, fname, title, P_lines):
    """Overlay all scenario pressures on one axis for easy comparison."""
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(title, fontsize=11, fontweight='bold')

    ax_P, ax_Q = axes[0], axes[1]

    for i, sc in enumerate(scenarios):
        d = sc['run'](sc['cmds'], P0=sc['P0'])
        t = d['t']
        lbl = sc['title'].split('\n')[0]
        ax_P.plot(t, d['P'],                       color=f'C{i}', lw=1.5, label=lbl)
        ax_Q.plot(t, d['Qmi']+d['Qma']-d['Qat'], color=f'C{i}', lw=1.2, label=lbl)

    for ax, ylabel in [(ax_P, 'Chamber pressure [kPa]'), (ax_Q, 'Net flow rate [LPM]')]:
        for P_ref, clr, lbl in P_lines:
            ax.axhline(P_ref, color=clr, lw=0.9, ls='--', label=lbl)
        ax.set_ylabel(ylabel)
        ax.set_xlabel('Time [s]')
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, alpha=0.3)
        T_max = max(sc['cmds'][-1][0] if False else 1 for sc in scenarios)
        for xt in np.arange(0, 30, 1.0):
            ax.axvline(xt, color='lightgray', lw=STEP_LW, ls=':')

    plt.tight_layout()
    plt.savefig(fname, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {fname}')

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    outdir = 'results_char'
    os.makedirs(outdir, exist_ok=True)

    # ── Positive channel grid ─────────────────────────────────────────────────
    print('Plotting positive channel...')
    plot_scenario_grid(
        POS_SCENARIOS,
        f'{outdir}/char2_pos.png',
        f'Positive Channel Open-loop Characterization  |  MATLAB 13-param valve  |  V=50 mL\n'
        f'ch0: supply 400 kPa  |  micro + macro fill valves + exhaust valve  |  coeff≈363 kPa/(LPM·s)',
        [(ATM_KPA,    'gray',   'ATM 101 kPa'),
         (P_LINE_POS, 'orange', 'Supply 400 kPa')],
    )

    # ── Negative channel grid ─────────────────────────────────────────────────
    print('Plotting negative channel...')
    plot_scenario_grid(
        NEG_SCENARIOS,
        f'{outdir}/char2_neg.png',
        f'Negative Channel Open-loop Characterization  |  MATLAB 13-param valve  |  V=50 mL\n'
        f'ch6: suction 50 kPa  |  micro + macro suction valves + atm-fill valve  |  coeff≈363 kPa/(LPM·s)',
        [(P_LINE_NEG, 'cyan',  'Suction 50 kPa'),
         (ATM_KPA,   'gray',   'ATM 101 kPa')],
    )

    # ── Overlay comparison plots ──────────────────────────────────────────────
    print('Plotting overlay comparisons...')
    plot_P_overlay(
        POS_SCENARIOS[:5],   # fill scenarios only
        f'{outdir}/char2_pos_overlay.png',
        f'Positive Channel — All Fill Scenarios Overlaid  (V=50 mL)',
        [(ATM_KPA,    'gray',   'ATM 101'),
         (P_LINE_POS, 'orange', 'Supply 400')],
    )
    plot_P_overlay(
        NEG_SCENARIOS,
        f'{outdir}/char2_neg_overlay.png',
        f'Negative Channel — All Scenarios Overlaid  (V=50 mL)',
        [(P_LINE_NEG, 'cyan',  'Suction 50'),
         (ATM_KPA,   'gray',   'ATM 101')],
    )

    print(f'\ncoeff = {coeff:.1f} kPa/(LPM·s)  [V={VOLUME_M3*1e6:.0f} mL]')
    print('Done.')
