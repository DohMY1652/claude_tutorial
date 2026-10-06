"""Reproducible, read-only-data nominal fitting for actuator 4 / mount 2.

Run from workspace root with PYTHONNOUSERSITE=1 python3 -m ph_model.fit_nominal.
No ROS dependencies, controller imports, sockets, or hardware commands.
"""
import argparse
import csv
import dataclasses
import json
import hashlib
import math
import platform
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import scipy
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
import yaml

from .data import load_run, stationary_points, sha256
from .nominal import Parameters, torque, friction, gradient, energy


RUNS = [
    ('20261006_132907_695083_S2', 'supplementary'),
    ('20261006_134154_792847_R0', 'train'),
    ('20261006_134235_833624_S2', 'train'),
    ('20261006_134834_301155_S1a', 'train'),
    ('20261006_135218_489972_S1b', 'train'),
    ('20261006_135507_219340_S3', 'train'),
    ('20261006_135932_623541_S5', 'train'),
    ('20261006_140127_158356_S4', 'train'),
    ('20261006_141418_972450_S6', 'train'),
    ('20261006_142450_704718_S6', 'train'),
    ('20261006_143524_612945_S6', 'train'),
    ('20261006_150506_052414_S6', 'development'),
    ('20261006_152201_227262_S6', 'validation_prefix'),
]


def dump(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, ensure_ascii=False, allow_nan=False)+'\n')


def pack(runs):
    parts = []
    for run in runs:
        arrays = []
        for ch in run['chunks']:
            idx = np.flatnonzero(ch['good'] & (np.arange(len(ch['t'])) % 10 == 0))
            arrays.append(np.column_stack([ch['qs'][idx], ch['v'][idx], ch['acc'][idx], ch['pressure'][idx]]))
        arr = np.concatenate(arrays)
        if len(arr):
            parts.append((arr, np.full(len(arr), 1/math.sqrt(len(arr)))))
    data = np.concatenate([p[0] for p in parts])
    weight = np.concatenate([p[1] for p in parts])/math.sqrt(len(parts))
    return data, weight


def equation_error(data, p):
    q, v, acc = data[:, :3].T
    tau = torque(q, data[:, 3:], p)
    return p.inertia_kg_m2*acc + gradient(q, p) + friction(v, tau, p) - tau


def fit_equation(data, weight, base, elastic=False, starts=12, fixed=None, x1_min=.05):
    # V_e=0 main model; optional quadratic V_e fixes M_g to remove gravity/elastic confounding.
    names = ['x1_zero_m', 'alpha', 'damping_nm_s_rad'] + \
        (['elastic_k_nm_rad', 'elastic_bias_nm'] if elastic else ['gravity_nm'])
    lo = np.array([x1_min, 0, 0] + ([0, -2] if elastic else [2.943]))
    hi = np.array([.15, 2, 100] + ([10, 2] if elastic else [6]))
    if fixed:
        base = dataclasses.replace(base, **fixed)
        selected = np.array([n not in fixed for n in names])
        names = [n for n in names if n not in fixed]
        lo, hi = lo[selected], hi[selected]
    scale = hi-lo
    def decode(z):
        return dataclasses.replace(base, **dict(zip(names, map(float, lo+scale*z))))
    def fun(z):
        return weight*equation_error(data, decode(z))
    rng = np.random.default_rng(20261006)
    initial = np.clip((np.array([getattr(base, n) for n in names])-lo)/scale, 1e-5, 1-1e-5)
    attempts = []
    best = None
    for i in range(starts):
        result = least_squares(fun, initial if i==0 else rng.uniform(.05, .95, len(names)),
                               bounds=(np.zeros(len(names)), np.ones(len(names))),
                               ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=400)
        attempts.append(dict(cost=float(result.cost), success=bool(result.success), nfev=result.nfev))
        if best is None or result.cost < best.cost:
            best = result
    s = np.linalg.svd(best.jac, compute_uv=False)
    return decode(best.x), dict(objective='profile-balanced exact equation torque least squares',
        weighted_rmse_nm=float(np.linalg.norm(best.fun)), fitted_names=names,
        bounds={n:[float(a),float(b)] for n,a,b in zip(names,lo,hi)},
        boundary_parameters=[n for n,z in zip(names,best.x) if min(z,1-z)<1e-4],
        scaled_jacobian_singular_values=s.tolist(), scaled_jacobian_condition=float(s[0]/s[-1]),
        normalized_parameter_coordinates=best.x.tolist(), attempts=attempts)


def fit_static(points, base):
    """Directional Coulomb *approximation* for stationary branch initialization only.

    sign(branch) is NOT substituted for tanh in the dynamic model.
    """
    q = np.array([p['q'] for p in points])
    pressures = np.array([p['pressure'] for p in points])
    direction = np.array([p['direction'] for p in points])
    def decode(z):
        return dataclasses.replace(base, x1_zero_m=float(z[0]), alpha=float(z[1]), gravity_nm=float(z[2]))
    def fun(z):
        p = decode(z)
        tau = torque(q, pressures, p)
        return gradient(q,p)+p.alpha*np.abs(tau)*direction-tau
    result = least_squares(fun, [.084,.134,3.], bounds=([.05,0,2.943],[.15,2,6]), x_scale='jac')
    return decode(result.x), dict(rmse_nm=float(np.sqrt(np.mean(result.fun**2))),
                                  n_points=len(points), objective='directional saturated-friction approximation, not exact tanh equilibrium')


def spans(mask, minimum=100):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return [(a,b) for a,b in zip(np.flatnonzero(edges==1), np.flatnonzero(edges==-1)) if b-a>=minimum]


def simulate(t, pressure, z0, p, rtol=2e-5, max_step=.05):
    """Offline continuous nominal ODE, sampled measured pressures, no state feedback."""
    # Scalar implementation keeps the many long LSODA rollouts reasonably fast.
    area2 = p.area_plus
    L, D, r, span = p.fold_length_m, p.diameter_m, p.reel_radius_m, 2*p.folds*p.fold_length_m
    def fun(time, z):
        q, v = z
        s = (p.x1_zero_m-r*q)/span
        if not 0 <= s < 1:
            raise ValueError('Free rollout left bellows geometry domain')
        c = math.sqrt(1-s*s)
        ds = D-L*c/3
        S = math.sqrt((ds/2)**2+(L*s/math.pi)**2)
        area1 = area2-math.pi*L*((1-2*s*s)/c*S+L*s*s/S*(ds/12+L*c/math.pi**2))
        p1, p2 = np.interp(time,t,pressure[:,0]), np.interp(time,t,pressure[:,1])
        tau = r*(-area1*p1+area2*p2)
        grad = p.gravity_nm*math.sin(q)+p.elastic_k_nm_rad*q+p.elastic_bias_nm + \
            p.limit_k_nm_rad*(max(q-p.q_max_rad,0)-max(p.q_min_rad-q,0))
        fric = p.alpha*abs(tau)*math.tanh(v/p.epsilon_rad_s)+p.damping_nm_s_rad*v
        return [v, (tau-grad-fric)/p.inertia_kg_m2]
    sol = solve_ivp(fun, [t[0], t[-1]], z0, t_eval=t, method='LSODA',
                    rtol=rtol, atol=rtol*.01, max_step=max_step)
    if not sol.success or len(sol.t) != len(t):
        raise RuntimeError(sol.message)
    return sol.y.T


def metrics(error):
    error = np.asarray(error)
    return dict(rmse_deg=float(np.sqrt(np.mean(error**2))), mae_deg=float(np.mean(np.abs(error))),
                p95_abs_deg=float(np.percentile(np.abs(error),95)), max_abs_deg=float(np.max(np.abs(error))),
                bias_deg=float(np.mean(error)), samples=len(error))


def evaluate(run, p, artifact=None):
    errors, blocks, saved = [], [], []
    for ch in run['chunks']:
        # Continuous in-domain measurement intervals only. Restart counts are explicit.
        for a,b in spans(ch['good'], minimum=1000):
            ix = np.arange(a,b,10)
            t, pressure = ch['t'][ix], ch['pressure'][ix]
            pred = simulate(t, pressure, [ch['qs'][a],ch['v'][a]], p)
            err = np.rad2deg(pred[:,0]-ch['q'][ix])
            errors.extend(err)
            blocks.append(dict(start_s=float(t[0]), end_s=float(t[-1]), **metrics(err)))
            saved.append(dict(t=t,q=ch['q'][ix],pred=pred[:,0],velocity=pred[:,1],pressure=pressure))
    if not errors:
        return dict(status='no in-domain uninterrupted measurement block >=10 s'), saved
    result = dict(**metrics(errors), blocks=blocks, evaluated_s=sum(b['end_s']-b['start_s'] for b in blocks),
                  reset_count=len(blocks), equation_rmse_nm=float(np.sqrt(np.mean(equation_error(pack([run])[0],p)**2))))
    if artifact is not None:
        np.savez_compressed(artifact, **{f'block{i}_{k}':v for i,s in enumerate(saved) for k,v in s.items()})
    return result, saved


def plot_rollout(saved, title, dest):
    fig, axes = plt.subplots(2,1,figsize=(11,6),sharex=True)
    for i,b in enumerate(saved):
        axes[0].plot(b['t'],np.rad2deg(b['q']),color='black',lw=1,label='Measured' if i==0 else None)
        axes[0].plot(b['t'],np.rad2deg(b['pred']),color='tab:red',lw=1,label='Nominal free rollout' if i==0 else None)
        axes[1].plot(b['t'],b['pressure'][:,1]/1000,color='tab:blue',label='P2 measured' if i==0 else None)
        axes[1].plot(b['t'],b['pressure'][:,0]/1000,color='tab:orange',label='P1 measured' if i==0 else None)
    axes[0].set(ylabel='Angle [deg]',title=title)
    axes[1].set(ylabel='Gauge pressure [kPa]',xlabel='Run time [s]')
    for ax in axes:
        ax.grid(alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(dest,dpi=160); plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root',type=Path,default=Path('/home/risebrl/result/ph/4'))
    parser.add_argument('--output',type=Path,default=Path('reports/nominal_actuator4_20261006'))
    parser.add_argument('--starts',type=int,default=12)
    args=parser.parse_args()
    out=args.output
    out.mkdir(parents=True,exist_ok=True); (out/'artifacts').mkdir(exist_ok=True)
    base=Parameters.load()
    runs=[]
    for name, role in RUNS:
        run=load_run(args.data_root/name, cutoff=356.45 if role=='validation_prefix' else None)
        run['role']=role
        if str(run['meta']['actuator'])!='4' or run['meta']['mount']!=2:
            raise ValueError('Actuator/mount mismatch')
        enc=run['meta']['encoder']
        if run['meta']['axis']!=1 or enc['raw_0deg']!=29890 or enc['raw_90deg']!=9600:
            raise ValueError('Axis/encoder calibration mismatch in selected dataset')
        runs.append(run)
        print('Loaded',name,role,run['audit']['measurement_valid_rows'],flush=True)
    provenance=dict(paper_sha256=sha256('main.tex'),params_sha256=sha256('ph_model/params.yaml'),
        code_sha256={str(p):sha256(p) for p in Path('ph_model').glob('*.py')},
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
        git_status=subprocess.check_output(['git','status','--short'],text=True),
        python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,
        argv=vars(args)|{'data_root':str(args.data_root),'output':str(args.output)},
        split=[dict(path=r['path'],role=r['role'],profile=r['meta']['profile']['id'],
                    seed=r['meta']['profile']['seed'],encoder=r['meta']['encoder'],
                    profile_sha256=r['meta']['profile_sha256'],acquisition_commit=r['meta']['commit'],
                    controller_parameters_sha256=hashlib.sha256(json.dumps(r['meta']['effective_parameters']['/pack2/pp_controller'],sort_keys=True).encode()).hexdigest(),
                    **r['audit']) for r in runs])
    dump(out/'provenance.json',provenance)
    train=[r for r in runs if r['role']=='train']
    data,weight=pack(train)
    points=[pt for r in train for pt in stationary_points(r)]
    static, static_info=fit_static(points,base)
    print('Static approximation',dataclasses.asdict(static),static_info,flush=True)
    fits={'directional_static_initializer':dict(parameters=dataclasses.asdict(static),fit=static_info)}
    models={}
    for name,elastic,eps in [('primary_Ve0',False,.02),('elastic_sensitivity',True,.02),
                             ('epsilon_0005',False,.005),('epsilon_0002',False,.002)]:
        p,info=fit_equation(data,weight,dataclasses.replace(base,epsilon_rad_s=eps),elastic,args.starts)
        fits[name]=dict(parameters=dataclasses.asdict(p),fit=info)
        models[name]=p
        print('Fit',name,info['weighted_rmse_nm'],dataclasses.asdict(p),flush=True)
    dump(out/'fits.json',fits)
    (out/'nominal_fitted.yaml').write_text(yaml.safe_dump(dataclasses.asdict(models['primary_Ve0']),sort_keys=False))
    # No validation data enters any of the optimizations above.
    evaluated={}
    for name,p in models.items():
        evaluated[name]={}
        targets=runs if name=='primary_Ve0' else [r for r in runs if r['role']=='development']
        for run in targets:
            label=Path(run['path']).name
            print('Rollout',name,label,flush=True)
            score,saved=evaluate(run,p,out/'artifacts'/f'{name}_{label}.npz')
            evaluated[name][label]=score
            if name=='primary_Ve0' and saved:
                plot_rollout(saved,f"{run['meta']['profile']['id']} seed {run['meta']['profile']['seed']} | {run['role']} | nominal V_e=0",out/f'{label}.png')
            dump(out/'metrics.json',evaluated)
    # Inertia is an assumption: compare the same model, no refit, on development only.
    dev=next(r for r in runs if r['role']=='development')
    sensitivity={}
    for J in [.0225,.09]:
        p=dataclasses.replace(models['primary_Ve0'],inertia_kg_m2=J)
        score,_=evaluate(dev,p)
        sensitivity[str(J)]=score
    dump(out/'inertia_sensitivity.json',sensitivity)
    # Numerical convergence on 30 s development prefix, no validation tuning.
    ch=dev['chunks'][0]
    a,b=max(spans(ch['good'],1000),key=lambda ab:ab[1]-ab[0])
    ix=np.arange(a,min(b,a+3000),5)
    t,pressure=ch['t'][ix],ch['pressure'][ix]
    p=models['primary_Ve0']; z0=[ch['qs'][a],ch['v'][a]]
    coarse=simulate(t,pressure,z0,p)
    fine=simulate(t,pressure,z0,p,rtol=2e-7,max_step=.01)
    dump(out/'numerical_convergence.json',metrics(np.rad2deg(coarse[:,0]-fine[:,0])))
    print('DONE',out,flush=True)


if __name__=='__main__':
    main()
