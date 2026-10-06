"""Read-only loading, validity masks and zero-phase differentiation of run.csv."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.signal import savgol_filter


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def gauge_pressures(pos_abs_kpa, neg_abs_kpa, atmosphere_kpa=101.325):
    return np.column_stack((np.asarray(neg_abs_kpa)-atmosphere_kpa,
                            np.asarray(pos_abs_kpa)-atmosphere_kpa))*1000


def differentiate(q, dt, window_s=.31):
    n = max(5, int(round(window_s/dt)) | 1)
    if len(q) < n:
        raise ValueError('Trajectory too short for centered derivative filter')
    return tuple(savgol_filter(q, n, 3, deriv=k, delta=dt, mode='interp') for k in range(3))


def load_run(path, cutoff=None, window_s=.31, dt=.01):
    path = Path(path)
    meta = json.loads((path/'meta.json').read_text())
    raw = np.genfromtxt(path/'run.csv', delimiter=',', names=True, dtype=None, encoding='utf-8')
    t = raw['t_mono_s']
    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0):
        raise ValueError(f'Non-monotonic timestamps: {path}')
    valid = (raw['sensor_valid'] == 1) & np.isfinite(raw['angle_deg']) & \
        np.isfinite(raw['p_pos_kpa']) & np.isfinite(raw['p_neg_kpa']) & \
        (raw['angle_deg'] >= -5) & (raw['angle_deg'] <= 88) & \
        (raw['angle_age_s'] < .1) & (raw['pressure_age_s'] < .1) & \
        (raw['angle_age_s'] >= 0) & (raw['pressure_age_s'] >= 0)
    if cutoff is not None:
        valid &= t < cutoff
    segs = {s['segment']: s for s in meta['profile']['segments']}
    measurement = np.array([segs.get(s, {}).get('kind') == 'measurement' for s in raw['segment']])
    domain = (raw['p_pos_kpa'] >= 101.325) & (raw['p_neg_kpa'] <= 101.325)
    # Only interpolate within uninterrupted, sensor-valid runs; do not bridge faults.
    breaks = (~valid[1:]) | (~valid[:-1]) | (np.diff(t) > .05)
    starts = np.r_[0, np.flatnonzero(breaks)+1]
    ends = np.r_[starts[1:], len(t)]
    chunks = []
    for a, b in zip(starts, ends):
        if b-a < 50 or not np.all(valid[a:b]) or t[b-1]-t[a] < 1:
            continue
        grid = np.arange(t[a], t[b-1], dt)
        idx = np.clip(np.searchsorted(t, grid, side='right')-1, a, b-1)
        q = np.interp(grid, t[a:b], np.deg2rad(raw['angle_deg'][a:b]))
        qs, v, acc = differentiate(q, dt, window_s)
        pressure = gauge_pressures(np.interp(grid, t[a:b], raw['p_pos_kpa'][a:b]),
                                   np.interp(grid, t[a:b], raw['p_neg_kpa'][a:b]))
        # Strict original and interpolated domain masks, never clip pressures.
        good = measurement[idx] & domain[idx] & (pressure[:, 0] <= 0) & (pressure[:, 1] >= 0)
        edge = int(np.ceil(window_s/dt))
        good[:edge] = False
        good[-edge:] = False
        chunks.append(dict(t=grid, q=q, qs=qs, v=v, acc=acc, pressure=pressure,
                           good=good, segment=raw['segment'][idx]))
    return dict(path=str(path), meta=meta, raw=raw, chunks=chunks,
                audit=dict(rows=len(raw), valid_rows=int(valid.sum()),
                           measurement_valid_rows=int((valid & measurement).sum()),
                           pressure_domain_excluded=int((valid & measurement & ~domain).sum()),
                           cutoff=cutoff, hashes={f:sha256(path/f) for f in ('run.csv','meta.json')}))


def stationary_points(run):
    """Recompute means from last 3 s of each settled segment, not polling n_sample."""
    path = Path(run['path'])/'points.csv'
    if not path.exists():
        return []
    out = []
    with path.open() as f:
        for point in csv.DictReader(f):
            if point['settled'] != '1':
                continue
            raw = run['raw']
            mask = raw['segment'] == point['segment']
            if not np.any(mask):
                continue
            mask &= raw['t_mono_s'] >= np.max(raw['t_mono_s'][mask])-3
            mask &= (raw['sensor_valid']==1) & (raw['p_pos_kpa']>=101.325) & (raw['p_neg_kpa']<=101.325) & \
                np.isfinite(raw['angle_deg']) & (raw['angle_deg']>=-5) & (raw['angle_deg']<=88) & \
                (raw['angle_age_s']>=0) & (raw['angle_age_s']<.1) & \
                (raw['pressure_age_s']>=0) & (raw['pressure_age_s']<.1)
            if mask.sum() < 200:
                continue
            out.append(dict(q=np.deg2rad(np.mean(raw['angle_deg'][mask])),
                            pressure=gauge_pressures([np.mean(raw['p_pos_kpa'][mask])],
                                                    [np.mean(raw['p_neg_kpa'][mask])])[0],
                            direction=1 if point['sweep']=='up' else -1,
                            center=float(point['center_kpa']), segment=point['segment'],
                            n_logged=int(mask.sum())))
    return out
