#!/usr/bin/env python3
"""
Standalone pneumatic control simulation v2 - New valve model.
Valve model: 13-parameter physical model (MATLAB port)
  - Bouc-Wen hysteresis + sigmoid effective area + 2nd-order dynamics + compressible flow
Controller: same feedforward + integral structure as v1, gains re-tuned.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import csv, os

# ── Physical constants ──────────────────────────────────────────────────────
RGAS    = 287.0
TEMP_K  = 293.15
LPM2KG  = 0.0002155
ATM_KPA = 101.325
KAPPA   = 1.4

P_LINE_POS   = 400.0
P_LINE_NEG   =  50.0
P_LINE_MACRO = 400.0
P_MACRO_NEG  =  50.0

DT      = 0.002     # 500 Hz
N_SUB   = 10        # valve 2nd-order ODE sub-steps per tick
T_TOTAL = 55.0   # extended for low-frequency sine evaluation
I_MAX   = 0.30      # max valve current [A] (u=1.0 → 0.3 A, matches MATLAB)
VOLUME_M3 = 1e-6    # chamber volume [m³] = 1 mL (sim_config.yaml)

# ── BiPAM Force Model ──────────────────────────────────────────────────────
# Source: "Cooperative antagonistic mechanism driven by bidirectional pneumatic
#          artificial muscles for soft robotic joints"
#          Park, J.H., Kim, K., Gong, Y.J., et al.
#          Mechatronics 97, 2023. DOI: 10.1016/j.mechatronics.2023.103099
#
# Derivation: virtual work  →  F = P_gauge × (dV / dL)
#
#   Positive pressure (P > P_atm):  bellows contracts.
#     F_pos = (P − P_atm) × 1e3 × π r_m²                 [N]
#
#   Negative pressure (P < P_atm):  inner tube extends.
#     F_neg = (P_atm − P) × 1e3 × π r_i²                 [N]
#
#   Length-dependent correction:
#     F(ε) = F₀ × (1 + c₁ε + c₂ε²)
#     ε_pos = (L₀ − L)/L₀  ∈ [0, 0.751]   (contraction, paper max 75.1 %)
#     ε_neg = (L − L₀)/L₀  ∈ [0, 4.021]   (extension,   paper max 402.1 %)
#
# Radii back-calculated from paper's peak force data:
#   F_pos_max = 124.46 N @ ΔP = 298.7 kPa  → r_m ≈ 11.5 mm
#   F_neg_max = 122.80 N @ ΔP =  51.3 kPa  → r_i ≈ 27.6 mm
# Calibrate from measured force–stroke curves before hardware use.
# ──────────────────────────────────────────────────────────────────────────────

_BIPAM = dict(
    L0          = 0.15,       # m  max extension length per chamber (50 mm dia spec)
    r_m         = 0.025,      # m  effective radius = 50 mm / 2   (positive pressure)
    r_i         = 0.025,      # m  effective radius = 50 mm / 2   (negative pressure)
    c_pos       = (-0.80, 0.30),
    c_neg       = ( 0.50, -0.10),
    eps_max_pos = 0.751,
    eps_max_neg = 4.021,
)


def bipam_force_pos(P_kPa, L_m=None):
    """BiPAM contraction force [N] from positive pressure. P_kPa: abs [kPa]."""
    dP = max(0.0, P_kPa - ATM_KPA) * 1e3
    F0 = dP * np.pi * _BIPAM['r_m'] ** 2
    if L_m is None or F0 == 0.0:
        return F0
    eps = np.clip((_BIPAM['L0'] - L_m) / _BIPAM['L0'], 0.0, _BIPAM['eps_max_pos'])
    c1, c2 = _BIPAM['c_pos']
    return max(0.0, F0 * (1.0 + c1 * eps + c2 * eps ** 2))


def bipam_force_neg(P_kPa, L_m=None):
    """BiPAM extension force [N] from negative pressure. P_kPa: abs [kPa]."""
    dP = max(0.0, ATM_KPA - P_kPa) * 1e3
    F0 = dP * np.pi * _BIPAM['r_i'] ** 2
    if L_m is None or F0 == 0.0:
        return F0
    eps = np.clip((L_m - _BIPAM['L0']) / _BIPAM['L0'], 0.0, _BIPAM['eps_max_neg'])
    c1, c2 = _BIPAM['c_neg']
    return max(0.0, F0 * (1.0 + c1 * eps + c2 * eps ** 2))


def bipam_net_force(P_pos_kPa, P_neg_kPa, L_m=None):
    """Series cooperative net force [N]. Both chambers act in same direction."""
    return bipam_force_pos(P_pos_kPa, L_m) + bipam_force_neg(P_neg_kPa, L_m)


# ── 13-parameter valve model ────────────────────────────────────────────────
# [A_max, k_shape, C_k, C_p, C_z, A_bw, beta_bw, gamma_bw, alpha,
#  wn_up, zeta_up, wn_down, zeta_down]
#
# A_max is calibrated for the 1 mL chamber (VOLUME_M3 = 1e-6 m³).
# The required flow for a typical step (50 kPa, TC=0.2s) is ~0.014 LPM.
# A_max = 0.002 gives Q_max ≈ 0.45 LPM (30x headroom) and negligible leakage at u=0%.
#
# ⚠ Replace all 13 values with MATLAB optimization results when hardware data is available.
#
# MATLAB optimization results (real hardware data)
# wn_up=2.35 rad/s → τ_open=0.435s, wn_down=3.61 → τ_close=0.178s (very slow valve)
# Valve effective dead zone: u < ~25% gives essentially zero flow (alpha=83.57 steep sigmoid)
# Q_max ≈ 46 LPM at P_in=400 kPa — requires VOLUME_M3 ≥ 10 mL to avoid saturation
VP_POS = [0.177485,  24.9354, 0.0918,   0.000251, 0.0,       # A_max, k_shape, C_k, C_p, C_z
          363318.0739, 1.6334, 0.1516, 83.5718,               # A_bw, beta_bw, gamma_bw, alpha
          2.3474, 0.9792, 3.6058, 1.5719]                     # wn_up, zeta_up, wn_down, zeta_down

VP_NEG = [0.177485,  24.9354, 0.0918,   0.000251, 0.0,
          363318.0739, 1.6334, 0.1516, 83.5718,
          2.3474, 0.9792, 3.6058, 1.5719]

# ─── Compressible flow function Phi (MATLAB: get_phi) ───────────────────────
_PCR   = (2.0 / (KAPPA + 1.0)) ** (KAPPA / (KAPPA - 1.0))
_PHICK = np.sqrt(KAPPA * (2.0 / (KAPPA + 1.0)) ** ((KAPPA + 1.0) / (KAPPA - 1.0)))
_PHISC = np.sqrt((2.0 * KAPPA) / (KAPPA - 1.0))

def get_phi(P_in, P_out):
    if P_in < 1e-6 or P_out >= P_in:
        return 0.0
    Pr = min(1.0, P_out / P_in)
    if Pr <= _PCR:
        return _PHICK
    return _PHISC * np.sqrt(max(0.0, Pr ** (2.0 / KAPPA) - Pr ** ((KAPPA + 1.0) / KAPPA)))

# ─── Valve: static flow model (MATLAB: simulate_physics_model steady-state) ─
def q_static(u_pct, z, P_in, P_out, vp):
    """Q_static = Area_eff * P_in * Phi  [LPM]"""
    if P_in <= P_out or u_pct < 0.5:
        return 0.0
    I = u_pct / 100.0 * I_MAX
    Force = max(-500.0, min(500.0, I + vp[4] * z + vp[3] * P_in - vp[2]))
    denom = max(1e-30, (1.0 + np.exp(-vp[1] * Force)) ** vp[8])
    return vp[0] / denom * P_in * get_phi(P_in, P_out)

def inv_q(Q_des, P_in, P_out, vp):
    """Bisect u_pct ∈ [0, 100] to satisfy q_static ≈ Q_des (z≈0 approximation)."""
    if Q_des <= 0.0:
        return 0.0
    if Q_des >= q_static(100.0, 0.0, P_in, P_out, vp):
        return 100.0
    lo, hi = 0.0, 100.0
    for _ in range(20):
        mid = 0.5 * (lo + hi)
        if q_static(mid, 0.0, P_in, P_out, vp) < Q_des:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)

# ─── Valve dynamic state: 2nd-order ODE + Bouc-Wen hysteresis ───────────────
class ValveState:
    """
    State: Q [LPM], dQ/dt, z (Bouc-Wen hysteresis).
    Port of MATLAB simulate_physics_model dynamics section.
    """
    __slots__ = ('Q', 'dQ', 'z', 'I_prev')

    def __init__(self):
        self.Q      = 0.0
        self.dQ     = 0.0
        self.z      = 0.0
        self.I_prev = 0.0

    def step(self, u_pct, P_in, P_out, vp, dt):
        """Advance one tick; return Q_actual [LPM]."""
        u_pct = max(0.0, min(100.0, u_pct))
        I = u_pct / 100.0 * I_MAX

        # Bouc-Wen hysteresis (MATLAB lines within k-loop)
        dI      = I - self.I_prev
        abs_dI  = abs(dI)
        dz = vp[5] * dI - vp[6] * abs_dI * self.z - vp[7] * dI * abs(self.z)
        self.z  = max(-1e6, min(1e6, self.z + dz))

        # Static target flow
        Q_tgt = max(0.0, q_static(u_pct, self.z, P_in, P_out, vp))

        # 2nd-order dynamics: opening vs closing bandwidth
        wn   = vp[9]  if dI >= 0.0 else vp[11]
        zeta = vp[10] if dI >= 0.0 else vp[12]

        dt_s = dt / N_SUB
        for _ in range(N_SUB):
            new_Q  = self.Q  + dt_s * self.dQ
            new_dQ = self.dQ + dt_s * (wn * wn * (Q_tgt - self.Q) - 2.0 * zeta * wn * self.dQ)
            self.Q  = max(0.0, new_Q)
            self.dQ = new_dQ

        self.I_prev = I
        return self.Q

# ─── Controller (feedforward + integral, same as v1) ─────────────────────────
def compute_uref(P_now, P_ref, vp, V, is_pos,
                 TC, ki_mi, ki_ma, ki_at, macro_thr, integral, dt):
    """Returns (u_mi, u_ma, u_at, new_integral)."""
    err = P_ref - P_now

    # Anti-windup reset (mirrors Controller.cpp)
    if is_pos:
        if (err > 0 and integral < 0) or (err < 0 and integral > 0):
            integral = 0.0
        intg = max(-1000.0, min(1000.0, integral + err * dt))
    else:
        if (err > 0 and integral > 0) or (err < 0 and integral < 0):
            integral = 0.0
        intg = max(-1000.0, min(1000.0, integral - err * dt))

    # Feedforward: desired flow [LPM] from plant inversion.
    # coeff [kPa/LPM] converts Q to dP/dt; m_dot = desired_dPdt / coeff
    # NOTE: the v1 formula P_dot*V/(R*T)/LPM2KG gives 1000x too small for V=1e-6 m³;
    #       with the new model's smooth sigmoid there is no dead-zone offset to hide this.
    coeff = (RGAS * TEMP_K / V) * LPM2KG / 1000.0  # kPa / (LPM * s)
    m_dot = err / (max(1e-3, TC) * coeff)            # LPM (positive = fill, negative = exhaust)

    u_mi = u_ma = u_at = 0.0
    clip = lambda x: max(0.0, min(100.0, x))

    if is_pos:
        if m_dot > 0:   # pressurize
            u_mi = clip(inv_q(m_dot, P_LINE_POS,  P_now, vp) + ki_mi * intg)
            if abs(err) >= macro_thr:
                u_ma = clip(inv_q(m_dot, P_LINE_MACRO, P_now, vp) + ki_ma * intg)
        else:            # exhaust
            u_at = clip(inv_q(-m_dot, P_now, ATM_KPA, vp) + ki_at * abs(intg))
    else:
        if m_dot < 0:   # exhaust (draw down toward suction)
            u_mi = clip(inv_q(-m_dot, P_now, P_LINE_NEG,  vp) + ki_mi * intg)
            if abs(err) >= macro_thr:
                u_ma = clip(inv_q(-m_dot, P_now, P_MACRO_NEG, vp) + ki_ma * intg)
        else:            # fill (vent to atm)
            u_at = clip(inv_q(m_dot, ATM_KPA, P_now, vp) + ki_at * abs(intg))

    return u_mi, u_ma, u_at, intg

# ─── Reference trajectory (same as v1) ──────────────────────────────────────
def reference(t):
    if t < 2.0:
        return ATM_KPA, ATM_KPA
    elif t < 6.0:
        return 150.0, 85.0
    elif t < 10.0:
        return 200.0, 72.0
    elif t < 14.0:
        return 130.0, 90.0
    else:
        ph = 2.0 * np.pi * (t - 14.0) / 20.0   # 0.05 Hz  (was 0.25 Hz)
        return 130.0 + 35.0 * np.sin(ph), 90.0 - 8.0 * np.sin(ph)

# ─── Simulation runner ───────────────────────────────────────────────────────
def run(pp, pn, vp_pos=None, vp_neg=None):
    if vp_pos is None: vp_pos = VP_POS
    if vp_neg is None: vp_neg = VP_NEG
    N = int(T_TOTAL / DT)
    V = pp['volume_m3']
    coeff = (RGAS * TEMP_K / V) * LPM2KG / 1000.0

    P_pos = ATM_KPA; P_neg = ATM_KPA
    int_pos = int_neg = 0.0

    vmi_p = ValveState(); vma_p = ValveState(); vat_p = ValveState()
    vmi_n = ValveState(); vma_n = ValveState(); vat_n = ValveState()

    t_arr  = np.empty(N)
    rp_arr = np.empty(N); rn_arr = np.empty(N)
    Pp_arr = np.empty(N); Pn_arr = np.empty(N)
    umi_p  = np.empty(N); uma_p  = np.empty(N); uat_p = np.empty(N)
    umi_n  = np.empty(N); uma_n  = np.empty(N); uat_n = np.empty(N)
    F_pos_arr = np.empty(N); F_neg_arr = np.empty(N); F_net_arr = np.empty(N)

    for k in range(N):
        t = k * DT
        rp, rn = reference(t)

        # Controller
        u_mi_p, u_ma_p, u_at_p, int_pos = compute_uref(
            P_pos, rp, vp_pos, V, True,
            pp['TC'], pp['ki_mi'], pp['ki_ma'], pp['ki_at'], pp['macro_thr'],
            int_pos, DT)
        u_mi_n, u_ma_n, u_at_n, int_neg = compute_uref(
            P_neg, rn, vp_neg, V, False,
            pn['TC'], pn['ki_mi'], pn['ki_ma'], pn['ki_at'], pn['macro_thr'],
            int_neg, DT)

        # Valve dynamics (each valve has independent hysteresis + 2nd-order state)
        Q_mi_p = vmi_p.step(u_mi_p, P_LINE_POS,   P_pos,       vp_pos, DT)
        Q_ma_p = vma_p.step(u_ma_p, P_LINE_MACRO,  P_pos,       vp_pos, DT)
        Q_at_p = vat_p.step(u_at_p, P_pos,         ATM_KPA,     vp_pos, DT)
        Q_mi_n = vmi_n.step(u_mi_n, P_neg,         P_LINE_NEG,  vp_neg, DT)
        Q_ma_n = vma_n.step(u_ma_n, P_neg,         P_MACRO_NEG, vp_neg, DT)
        Q_at_n = vat_n.step(u_at_n, ATM_KPA,       P_neg,       vp_neg, DT)

        Q_net_p =  Q_mi_p + Q_ma_p - Q_at_p
        Q_net_n =  Q_at_n - Q_mi_n - Q_ma_n

        P_pos = max(50.0,  min(800.0, P_pos + coeff * Q_net_p * DT))
        P_neg = max(10.0,  min(110.0, P_neg + coeff * Q_net_n * DT))

        t_arr[k]  = t
        rp_arr[k] = rp;    rn_arr[k] = rn
        Pp_arr[k] = P_pos; Pn_arr[k] = P_neg
        umi_p[k]  = u_mi_p; uma_p[k] = u_ma_p; uat_p[k] = u_at_p
        umi_n[k]  = u_mi_n; uma_n[k] = u_ma_n; uat_n[k] = u_at_n
        F_pos_arr[k] = bipam_force_pos(P_pos)
        F_neg_arr[k] = bipam_force_neg(P_neg)
        F_net_arr[k] = bipam_net_force(P_pos, P_neg)

    return dict(t=t_arr, rp=rp_arr, rn=rn_arr,
                P_pos=Pp_arr, P_neg=Pn_arr,
                umi_p=umi_p, uma_p=uma_p, uat_p=uat_p,
                umi_n=umi_n, uma_n=uma_n, uat_n=uat_n,
                F_pos=F_pos_arr, F_neg=F_neg_arr, F_net=F_net_arr)

# ─── Output helpers (identical to v1) ───────────────────────────────────────
def rmse(a, b, t, t_start=2.0):
    mask = t >= t_start
    return float(np.sqrt(np.mean((a[mask] - b[mask]) ** 2)))

def save_csv(d, path):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t_s', 'ref_pos_kPa', 'P_pos_kPa', 'ref_neg_kPa', 'P_neg_kPa',
                    'u_mi_pos', 'u_ma_pos', 'u_at_pos',
                    'u_mi_neg', 'u_ma_neg', 'u_at_neg',
                    'F_pos_N', 'F_neg_N', 'F_net_N'])
        for i in range(len(d['t'])):
            w.writerow([f"{d['t'][i]:.4f}",
                        f"{d['rp'][i]:.4f}", f"{d['P_pos'][i]:.4f}",
                        f"{d['rn'][i]:.4f}", f"{d['P_neg'][i]:.4f}",
                        f"{d['umi_p'][i]:.2f}", f"{d['uma_p'][i]:.2f}", f"{d['uat_p'][i]:.2f}",
                        f"{d['umi_n'][i]:.2f}", f"{d['uma_n'][i]:.2f}", f"{d['uat_n'][i]:.2f}",
                        f"{d['F_pos'][i]:.3f}", f"{d['F_neg'][i]:.3f}", f"{d['F_net'][i]:.3f}"])

def plot(d, title, path):
    fig, axes = plt.subplots(3, 2, figsize=(14, 13))
    fig.suptitle(title, fontsize=13, fontweight='bold')
    t = d['t']

    for ax, key, ref, col, ch in [
        (axes[0, 0], 'P_pos', 'rp', 'b', 'ch0  Positive Channel'),
        (axes[0, 1], 'P_neg', 'rn', 'r', 'ch6  Negative Channel'),
    ]:
        ax.plot(t, d[ref], 'k--', lw=1.5, label='ref')
        ax.plot(t, d[key], color=col, lw=1.5, label='actual')
        for xv in [2, 6, 10, 14]:
            ax.axvline(xv, color='gray', lw=0.7, ls=':')
        ax.set_ylabel('Pressure [kPa]'); ax.set_title(ch)
        ax.legend(loc='upper left'); ax.grid(True, alpha=0.3); ax.set_xlim(0, T_TOTAL)
        e = rmse(d[key], d[ref], t)
        ax.text(0.98, 0.05, f'RMSE={e:.2f} kPa',
                transform=ax.transAxes, ha='right', fontsize=9,
                bbox=dict(boxstyle='round', fc='white', alpha=0.7))

    for ax, (mi, ma, at), ch in [
        (axes[1, 0], ('umi_p', 'uma_p', 'uat_p'), 'ch0  Control Inputs'),
        (axes[1, 1], ('umi_n', 'uma_n', 'uat_n'), 'ch6  Control Inputs'),
    ]:
        ax.plot(t, d[mi], 'b-', lw=1.0, label='u_micro')
        ax.plot(t, d[at], 'g-', lw=1.0, label='u_atm')
        ax.plot(t, d[ma], 'm-', lw=1.0, label='u_macro')
        ax.set_ylabel('Valve opening [%]'); ax.set_title(ch)
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3); ax.set_xlim(0, T_TOTAL); ax.set_ylim(-5, 105)

    # ── Force: individual channels ───────────────────────────────
    ax = axes[2, 0]
    ax.plot(t, d['F_pos'], 'b-', lw=1.5, label='F_pos (contraction)')
    ax.plot(t, d['F_neg'], 'r-', lw=1.5, label='F_neg (extension)')
    for xv in [2, 6, 10, 14]:
        ax.axvline(xv, color='gray', lw=0.7, ls=':')
    ax.set_ylabel('Force [N]'); ax.set_title('BiPAM Channel Forces')
    ax.set_xlabel('Time [s]'); ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3); ax.set_xlim(0, T_TOTAL)

    # ── Force: net cooperative-antagonistic ──────────────────────
    ax = axes[2, 1]
    ax.plot(t, d['F_net'], 'k-', lw=1.5, label='F_net')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    for xv in [2, 6, 10, 14]:
        ax.axvline(xv, color='gray', lw=0.7, ls=':')
    ax.set_ylabel('Net Force [N]'); ax.set_title('BiPAM Net Force (series cooperative: pos + neg)')
    ax.set_xlabel('Time [s]'); ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3); ax.set_xlim(0, T_TOTAL)

    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()

def plot_comparison(results, path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    t = results[0][1]['t']
    for ax, key, ref_key, title in [
        (axes[0], 'P_pos', 'rp', 'ch0 Positive – gain comparison'),
        (axes[1], 'P_neg', 'rn', 'ch6 Negative – gain comparison'),
    ]:
        ax.plot(t, results[0][1][ref_key], 'k--', lw=1.5, label='ref', zorder=5)
        for i, (label, d) in enumerate(results):
            e = rmse(d[key], d[ref_key], t)
            ax.plot(t, d[key], color=f'C{i}', lw=1.2, label=f'{label}  RMSE={e:.2f}')
        for xv in [2, 6, 10, 14]:
            ax.axvline(xv, color='gray', lw=0.6, ls=':')
        ax.set_xlabel('Time [s]'); ax.set_ylabel('Pressure [kPa]'); ax.set_title(title)
        ax.legend(fontsize=8); ax.grid(True, alpha=0.3); ax.set_xlim(0, T_TOTAL)
    plt.tight_layout()
    plt.savefig(path, dpi=150, bbox_inches='tight')
    plt.close()

# ─── Gain sets (same grid as v1) ─────────────────────────────────────────────
# Gain sets tuned for MATLAB valve params:
#   τ_open=0.435s, τ_close=0.176s  (wn=2.35/3.61 rad/s)
#   Integral MUST be zero or tiny — steep sigmoid (alpha=83.57) causes explosive flow if
#   integral pushes u_pct above ~40%. Pure feedforward (ki=0) is the stable choice.
#   Theoretical min RMSE: step≈21 kPa (TC=0.3s), sine≈28 kPa (bandwidth limit).
GAINS = [
    # label,    TC_p, ki_mi_p, ki_ma_p, ki_at_p, mt_p,
    #            TC_n, ki_mi_n, ki_ma_n, ki_at_n, mt_n  (mt in kPa)
    ("ff_2s",   2.00, 0.00, 0.00, 0.00, 30.0,   2.00, 0.00, 0.00, 0.00, 20.0),
    ("ff_1s",   1.00, 0.00, 0.00, 0.00, 25.0,   1.00, 0.00, 0.00, 0.00, 18.0),
    ("ff_0p7",  0.70, 0.00, 0.00, 0.00, 20.0,   0.70, 0.00, 0.00, 0.00, 15.0),
    ("ff_0p5",  0.50, 0.00, 0.00, 0.00, 18.0,   0.50, 0.00, 0.00, 0.00, 12.0),
    ("ff_0p3",  0.30, 0.00, 0.00, 0.00, 15.0,   0.30, 0.00, 0.00, 0.00, 10.0),
    ("fi_0p5",  0.50, 0.02, 0.01, 0.02, 18.0,   0.50, 0.05, 0.02, 0.05, 12.0),
    ("fi_0p3",  0.30, 0.02, 0.01, 0.02, 15.0,   0.30, 0.05, 0.02, 0.05, 10.0),
    ("fi_0p2",  0.20, 0.02, 0.01, 0.02, 12.0,   0.20, 0.05, 0.02, 0.05,  8.0),
]

def make_params(row):
    lbl, TC_p, kmi_p, kma_p, kat_p, mt_p, TC_n, kmi_n, kma_n, kat_n, mt_n = row
    pp = dict(volume_m3=VOLUME_M3, TC=TC_p, ki_mi=kmi_p, ki_ma=kma_p, ki_at=kat_p, macro_thr=mt_p)
    pn = dict(volume_m3=VOLUME_M3, TC=TC_n, ki_mi=kmi_n, ki_ma=kma_n, ki_at=kat_n, macro_thr=mt_n)
    return lbl, pp, pn

# ─── Main ────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    out = 'results_v2'
    os.makedirs(out, exist_ok=True)

    results = []
    best_label, best_data, best_rmse_total = None, None, 1e9

    print(f"{'Label':<10} {'RMSE_pos':>10} {'RMSE_neg':>10} {'total':>10}")
    print('-' * 46)

    for row in GAINS:
        lbl, pp, pn = make_params(row)
        d = run(pp, pn)
        results.append((lbl, d))

        ep = rmse(d['P_pos'], d['rp'], d['t'])
        en = rmse(d['P_neg'], d['rn'], d['t'])
        total = ep + en
        print(f"{lbl:<10} {ep:>10.3f} {en:>10.3f} {total:>10.3f}")

        save_csv(d, f'{out}/sim_{lbl}.csv')

        if total < best_rmse_total:
            best_rmse_total, best_label, best_data = total, lbl, d

    print(f"\nBest: {best_label}")

    save_csv(best_data, f'{out}/sim_result_best.csv')
    plot(best_data,
         f'Pneumatic Sim v2  [{best_label}]  ch0(+) + ch6(-)  –  20 s\n'
         f'(New valve model: Bouc-Wen + sigmoid + 2nd-order + compressible flow)',
         f'{out}/sim_result_best.png')
    plot_comparison(results, f'{out}/sim_comparison.png')

    print(f"\nOutput files:")
    for f in sorted(os.listdir(out)):
        size = os.path.getsize(f'{out}/{f}')
        print(f"  {out}/{f}  ({size/1024:.1f} KB)")

    # Diagnostics
    d = best_data
    t  = d['t']; Pp = d['P_pos']; rp = d['rp']; Pn = d['P_neg']; rn = d['rn']

    def settling(t, P, t_step, r_target, tol=2.0):
        for i in np.where(t > t_step)[0]:
            if all(abs(P[i:i+50] - r_target) < tol):
                return t[i] - t_step
        return float('nan')

    print(f"\nSettling time (±2 kPa):")
    print(f"  Pos 101→150 kPa (t=2s):  {settling(t, Pp, 2.0, 150.0):.3f} s")
    print(f"  Neg 101→85  kPa (t=2s):  {settling(t, Pn, 2.0,  85.0):.3f} s")
    print(f"  Pos 150→200 kPa (t=6s):  {settling(t, Pp, 6.0, 200.0):.3f} s")
    print(f"  Neg  85→72  kPa (t=6s):  {settling(t, Pn, 6.0,  72.0):.3f} s")
    print(f"  Pos 200→130 kPa (t=10s): {settling(t, Pp,10.0, 130.0):.3f} s")
    print(f"  Neg  72→90  kPa (t=10s): {settling(t, Pn,10.0,  90.0):.3f} s")

    mask_sine = t >= 14.0
    ep_s = np.abs(Pp[mask_sine] - rp[mask_sine])
    en_s = np.abs(Pn[mask_sine] - rn[mask_sine])
    print(f"\nSine tracking (t=14–20 s):")
    print(f"  Positive: RMSE={np.sqrt(np.mean(ep_s**2)):.3f} kPa  peak={ep_s.max():.3f} kPa")
    print(f"  Negative: RMSE={np.sqrt(np.mean(en_s**2)):.3f} kPa  peak={en_s.max():.3f} kPa")
