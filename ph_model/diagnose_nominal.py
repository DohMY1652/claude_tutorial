"""Audit nominal assumptions against metadata and static observations; no refit."""
import json
from pathlib import Path
import numpy as np
from scipy.optimize import brentq

from .data import load_run, stationary_points, gauge_pressures, sha256
from .nominal import Parameters, torque, gradient, area_minus
from .fit_nominal import dump


def main():
    root=Path('reports/residual_actuator4_20261007/long_rollout_v2')
    models={name:Parameters.load(Path('reports/nominal_actuator4_20261006')/file)
            for name,file in [('zero_elastic','nominal_fitted.yaml'),('quadratic_elastic','elastic_rollout.yaml')]}
    names=['20261006_134235_833624_S2','20261006_134834_301155_S1a',
           '20261006_135218_489972_S1b','20261006_140127_158356_S4']
    result=dict(paper_sha256=sha256('main.tex'),runs={},geometry={})
    for name in names:
        r=load_run(Path('/home/risebrl/result/ph/4')/name)
        raw=r['raw'];points=stationary_points(r)
        info=dict(point_mechanics=r['meta']['point_mechanics'],encoder=r['meta']['encoder'],
                  source_hashes=r['audit']['hashes'],stationary={})
        for label,p in models.items():
            rows=[]
            for pt in points:
                eq=brentq(lambda q:float(gradient(q,p)-torque(q,pt['pressure'],p)),p.q_min_rad-.03,p.q_max_rad+.03)
                rows.append(dict(segment=pt['segment'],direction=pt['direction'],center_kpa=pt['center'],
                    measured_deg=float(np.rad2deg(pt['q'])),equilibrium_deg=float(np.rad2deg(eq)),
                    pressure_abs_kpa=(pt['pressure']/1000+101.325).tolist(),
                    zero_speed_unbalanced_torque_nm=float(torque(pt['q'],pt['pressure'],p)-gradient(pt['q'],p))))
            errors=[a['measured_deg']-a['equilibrium_deg'] for a in rows]
            info['stationary'][label]=dict(points=rows,rmse_deg=float(np.sqrt(np.mean(np.square(errors))))) if rows else dict(points=[])
        if name.endswith('S2'):
            centers=sorted(set(round(pt['center'],6) for pt in points));paired=[]
            for center in centers:
                sides=[sorted([pt for pt in points if abs(pt['center']-center)<1e-6 and pt['direction']==direction],
                              key=lambda pt:pt['q']) for direction in (1,-1)]
                if min(map(len,sides))<2:continue
                low=max(side[0]['q'] for side in sides);high=min(side[-1]['q'] for side in sides)
                if high<=low:continue
                q=np.linspace(low,high,15)
                pressures=[np.column_stack([np.interp(q,[pt['q'] for pt in side],
                    [pt['pressure'][j] for pt in side]) for j in (0,1)]) for side in sides]
                item=dict(center_kpa=center,angle_range_deg=np.rad2deg([low,high]).tolist(),
                          method='Exploratory same-angle ascending/descending linear interpolation; not independent samples or fit')
                for label,p in models.items():
                    up,down=[torque(q,P,p) for P in pressures]
                    mid=(up+down)/2;half=(up-down)/2
                    item[label]=dict(central_torque_rmse_nm=float(np.sqrt(np.mean((mid-gradient(q,p))**2))),
                                     halfwidth_nm_range=[float(half.min()),float(half.max())])
                paired.append(item)
            info['same_angle_branch_diagnostic']=paired
        if name.endswith('S4'):
            holds=[]
            for seg in r['meta']['profile']['segments']:
                if seg.get('kind')!='measurement':continue
                mask=(raw['segment']==seg['segment'])&(raw['sensor_valid']==1)
                ix=np.flatnonzero(mask)
                if not len(ix) or raw['t_mono_s'][ix[-1]]-raw['t_mono_s'][ix[0]]<100:continue
                mask &= raw['t_mono_s']>=raw['t_mono_s'][ix[-1]]-30
                pos=float(raw['p_pos_kpa'][mask].mean());neg=float(raw['p_neg_kpa'][mask].mean())
                angle=float(raw['angle_deg'][mask].mean());P=gauge_pressures([pos],[neg])[0]
                holds.append(dict(segment=seg['segment'],start_s=float(raw['t_mono_s'][ix[0]]),
                    end_s=float(raw['t_mono_s'][ix[-1]]),pos_abs_kpa=pos,neg_abs_kpa=neg,
                    angle_mean_deg=angle,angle_std_deg=float(raw['angle_deg'][mask].std()),
                    unbalanced_torque_nm={label:float(torque(np.deg2rad(angle),P,p)-gradient(np.deg2rad(angle),p)) for label,p in models.items()}))
            info['holds_last30s']=holds
        result['runs'][name]=info
    for label,p in models.items():
        q=np.deg2rad(np.array([0,10,30,50,73,88]));x=p.x1_zero_m-p.reel_radius_m*q
        h=1e-6;der=(area_minus(x+h,p)-area_minus(x-h,p))/(2*h)
        result['geometry'][label]=dict(q_deg=np.rad2deg(q).tolist(),x1_mm=(x*1000).tolist(),
            Aminus_mm2=(area_minus(x,p)*1e6).tolist(),Aplus_mm2=p.area_plus*1e6,
            vacuum_stiffness_at_minus30kpa_nm_rad=(p.reel_radius_m**2*der*30000).tolist(),
            restoring_stiffness_nm_rad=(p.gravity_nm*np.cos(q)+p.elastic_k_nm_rad).tolist(),
            zero_pressure_equilibrium_deg=float(np.rad2deg(brentq(lambda q:float(gradient(q,p)),-.08,1.535))))
    dump(root/'nominal_diagnosis.json',result)
    print('NOMINAL DIAGNOSIS COMPLETE')
    for name,r in result['runs'].items():
        print(name,{k:v.get('rmse_deg') for k,v in r['stationary'].items()})
        if 'holds_last30s' in r:print(r['holds_last30s'])


if __name__=='__main__':main()
