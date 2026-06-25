#!/usr/bin/env python3
"""Comparison plot: v1 (algebraic valve) vs v2 (MATLAB 13-param valve)."""
import csv, numpy as np, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def load_csv(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            rows.append({k: float(v) for k, v in row.items()})
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}

def rmse(a, b, t, t_start=2.0):
    m = t >= t_start
    return float(np.sqrt(np.mean((a[m] - b[m]) ** 2)))

v1 = load_csv('results/sim_result_best.csv')
v2 = load_csv('results_v2/sim_result_best.csv')

ep1 = rmse(v1['P_pos_kPa'], v1['ref_pos_kPa'], v1['t_s'])
en1 = rmse(v1['P_neg_kPa'], v1['ref_neg_kPa'], v1['t_s'])
ep2 = rmse(v2['P_pos_kPa'], v2['ref_pos_kPa'], v2['t_s'])
en2 = rmse(v2['P_neg_kPa'], v2['ref_neg_kPa'], v2['t_s'])

fig, axes = plt.subplots(2, 2, figsize=(15, 9))
fig.suptitle(
    'Pneumatic Sim — v1 (Algebraic) vs v2 (MATLAB 13-param valve)\n'
    'Same chamber: V=1 mL, same reference trajectory',
    fontsize=12, fontweight='bold')

steps = [2, 6, 10, 14]

# ── Pressure plots ────────────────────────────────────────────────────────
for ax, (pkey, rkey), ch, (ep_a, en_a), ylabel in [
    (axes[0, 0], ('P_pos_kPa', 'ref_pos_kPa'), 'ch0  Positive Channel',
     (ep1, ep2), 'Pressure [kPa]'),
    (axes[0, 1], ('P_neg_kPa', 'ref_neg_kPa'), 'ch6  Negative Channel',
     (en1, en2), 'Pressure [kPa]'),
]:
    t = v1['t_s']
    ax.plot(t, v1[rkey], 'k--', lw=1.5, label='reference', zorder=5)
    ax.plot(t, v1[pkey], 'b-',  lw=1.5, label=f'v1  RMSE={ep_a:.2f} kPa', alpha=0.85)
    ax.plot(t, v2[pkey], 'r-',  lw=1.5, label=f'v2 (MATLAB)  RMSE={ep2 if pkey=="P_pos_kPa" else en2:.2f} kPa', alpha=0.85)
    for xv in steps:
        ax.axvline(xv, color='gray', lw=0.7, ls=':')
    ax.set_ylabel(ylabel); ax.set_title(ch)
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3); ax.set_xlim(0, 20)

# ── Control input: u_mi (fill valve) ─────────────────────────────────────
for ax, (mi1, mi2), ch in [
    (axes[1, 0], ('u_mi_pos', 'u_mi_pos'), 'ch0  Fill Valve (u_mi)'),
    (axes[1, 1], ('u_mi_neg', 'u_mi_neg'), 'ch6  Fill Valve (u_mi)'),
]:
    t = v1['t_s']
    ax.plot(t, v1[mi1], 'b-', lw=0.8, label='v1', alpha=0.75)
    ax.plot(t, v2[mi2], 'r-', lw=0.8, label='v2 (MATLAB)', alpha=0.75)
    for xv in steps:
        ax.axvline(xv, color='gray', lw=0.7, ls=':')
    ax.set_ylabel('Valve opening [%]'); ax.set_title(ch)
    ax.set_xlabel('Time [s]'); ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3); ax.set_xlim(0, 20); ax.set_ylim(-5, 105)

# ── Annotation box ────────────────────────────────────────────────────────
info = (
    f"v1  (algebraic model):       pos RMSE={ep1:.2f} kPa,  neg RMSE={en1:.2f} kPa\n"
    f"v2  (MATLAB 13-param valve): pos RMSE={ep2:.2f} kPa,  neg RMSE={en2:.2f} kPa\n"
    f"\nv2 bandwidth limit: wn=2.35 rad/s → τ=0.44s  (theoretical min RMSE ≈ 28 kPa for 0.25 Hz sine)"
)
fig.text(0.02, 0.01, info, fontsize=8, va='bottom',
         bbox=dict(boxstyle='round', fc='#fffbe6', alpha=0.8))

plt.tight_layout(rect=[0, 0.08, 1, 1])
out = 'results_v2/compare_v1_v2.png'
plt.savefig(out, dpi=150, bbox_inches='tight')
plt.close()
print(f'Saved: {out}')
print(f'\nv1  pos={ep1:.2f} kPa  neg={en1:.2f} kPa')
print(f'v2  pos={ep2:.2f} kPa  neg={en2:.2f} kPa  (MATLAB valve, bandwidth limited)')
