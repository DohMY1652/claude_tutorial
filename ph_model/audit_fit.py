"""Generate final nominal diagnostics/figures without touching original data."""
import argparse
import dataclasses
import json
from pathlib import Path
import numpy as np
from scipy.optimize import brentq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .data import load_run, stationary_points, sha256
from .nominal import Parameters, torque, friction, gradient, energy, area_minus
from .fit_nominal import RUNS, dump, simulate, metrics
from .refine_nominal import windows, simulate_windows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data-root',type=Path,default=Path('/home/risebrl/result/ph/4'))
    ap.add_argument('--output',type=Path,default=Path('reports/nominal_actuator4_20261006'))
    args=ap.parse_args();out=args.output
    fits=json.loads((out/'fits.json').read_text())
    refined=json.loads((out/'refined_fits.json').read_text())
    models={name:Parameters(**v['parameters']) for name,v in fits.items() if name in ('primary_Ve0','elastic_sensitivity')}
    models.update({name:Parameters(**v['parameters']) for name,v in refined.items()})
    runs=[]
    for name,role in RUNS:
        if role=='supplementary':continue
        r=load_run(args.data_root/name,cutoff=356.45 if role=='validation_prefix' else None)
        r['role']=role;runs.append(r)
    train=[r for r in runs if r['role']=='train']
    pts=[dict(pt,profile=r['meta']['profile']['id']) for r in train for pt in stationary_points(r)]
    diagnostic={}
    rng=np.random.default_rng(42)
    q=rng.uniform(-.08,1.535,20000);v=rng.uniform(-.5,.5,20000)
    P=np.column_stack((-rng.uniform(0,80000,20000),rng.uniform(0,85000,20000)))
    for name,p in models.items():
        tau=torque(q,P,p)
        port=np.column_stack((-p.reel_radius_m*area_minus(p.x1_zero_m-p.reel_radius_m*q,p)*v,
                              np.full(len(q),p.reel_radius_m*p.area_plus)*v))
        supplied=np.sum(P*port,axis=1)
        acc=(tau-gradient(q,p)-friction(v,tau,p))/p.inertia_kg_m2
        Hdot=p.inertia_kg_m2*v*acc+gradient(q,p)*v
        equilibrium=[]
        for pt in pts:
            root=brentq(lambda qq:float(gradient(qq,p)-torque(qq,pt['pressure'],p)),p.q_min_rad-.03,p.q_max_rad+.03)
            equilibrium.append(dict(profile=pt['profile'],segment=pt['segment'],center=pt['center'],
                                    measured_deg=float(np.rad2deg(pt['q'])),predicted_deg=float(np.rad2deg(root))))
        diagnostic[name]=dict(
            power_identity_max_abs_w=float(np.max(np.abs(supplied-tau*v))),
            dissipation_min_w=float(np.min(v*friction(v,tau,p))),
            continuous_passivity_violations=int(np.sum(Hdot-supplied>1e-10)),
            tested_states=len(q), static_zero_velocity=equilibrium,
            static_zero_velocity_rmse_deg=float(np.sqrt(np.mean([(pt['predicted_deg']-pt['measured_deg'])**2 for pt in equilibrium]))),
            zero_pressure_equilibrium_deg=float(np.rad2deg(brentq(lambda qq:float(gradient(qq,p)),-.08,1.535))))
    dump(out/'physics_diagnostics.json',diagnostic)
    # Verify midpoint optimization against independently integrated real training windows.
    wins=windows(train)
    crosscheck={}
    for name in refined:
        p=models[name]
        batched=simulate_windows(wins,p)
        batched_fine=simulate_windows(wins,p,dt=.01)
        errors=[]
        adaptive_errors=[]
        for i,w in enumerate(wins):
            adaptive=simulate(w['t'],w['pressure'],w['z0'],p,rtol=2e-7,max_step=.01)
            errors.extend(np.rad2deg(batched[i]-adaptive[:,0]))
            adaptive_errors.extend(np.rad2deg(adaptive[:,0]-w['q']))
        crosscheck[name]=dict(midpoint_vs_adaptive=metrics(errors),
                              midpoint_dt002_vs_dt001=metrics(np.rad2deg((batched-batched_fine).ravel())),
                              independent_training_window_prediction=metrics(adaptive_errors))
    dump(out/'integrator_crosscheck.json',crosscheck)
    # Input sample-rate sensitivity on development, never model selection on validation.
    dev=next(r for r in runs if r['role']=='development')
    ch=dev['chunks'][0]
    a=np.flatnonzero(ch['good'])[0];b=a+3001
    ratecheck={}
    for name in ('primary_Ve0','elastic_rollout'):
        p=models[name];predictions={}
        for stride in (10,2,1):
            ix=np.arange(a,b,stride)
            predictions[stride]=simulate(ch['t'][ix],ch['pressure'][ix],[ch['qs'][a],ch['v'][a]],p,rtol=2e-7,max_step=.01)
        ratecheck[name]={str(100//s)+'_vs_100Hz':metrics(np.rad2deg(predictions[s][:,0]-predictions[1][::s,0])) for s in (10,2)}
    dump(out/'input_rate_sensitivity.json',ratecheck)
    limit_audit={}
    for path in sorted((out/'artifacts').glob('*.npz')):
        with np.load(path) as arr:
            predictions=[arr[k] for k in arr.files if k.endswith('_pred')]
            if predictions:
                qhat=np.concatenate(predictions)
                limit_audit[path.stem]=dict(min_deg=float(np.rad2deg(np.min(qhat))),
                    max_deg=float(np.rad2deg(np.max(qhat))),outside_safety_range_samples=int(np.sum((qhat<np.deg2rad(-5))|(qhat>np.deg2rad(88)))))
    dump(out/'prediction_limit_audit.json',limit_audit)
    # Model comparison figure (all candidates fixed before final validation comparison).
    ordinary=json.loads((out/'metrics.json').read_text())
    extra=json.loads((out/'refined_metrics.json').read_text())
    tags=['primary_Ve0','Ve0_rollout','elastic_equation','elastic_rollout']
    devname=Path(dev['path']).name
    valname=Path(next(r for r in runs if r['role']=='validation_prefix')['path']).name
    combined={**ordinary,**extra}
    fig,ax=plt.subplots(figsize=(9,4))
    x=np.arange(len(tags))
    ax.bar(x-.18,[combined[m][devname]['rmse_deg'] for m in tags],.36,label='Development seed 3 (600 s)')
    ax.bar(x+.18,[combined[m][valname]['rmse_deg'] for m in tags],.36,label='Held-out seed 4 prefix (337 s)')
    ax.set(xticks=x,xticklabels=['V_e=0\nequation fit','V_e=0\nwindow fit','Quadratic V_e\nequation fit','Quadratic V_e\nwindow fit'],ylabel='Free-rollout RMSE [deg]')
    ax.legend();ax.grid(axis='y',alpha=.25);fig.tight_layout();fig.savefig(out/'comparison.png',dpi=180);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4.5))
    for name,color in [('primary_Ve0','tab:red'),('elastic_rollout','tab:blue')]:
        pts2=diagnostic[name]['static_zero_velocity']
        axes[0].scatter([pt['measured_deg'] for pt in pts2],[pt['predicted_deg'] for pt in pts2],s=14,alpha=.65,c=color,label=name)
    axes[0].plot([0,90],[0,90],'k--');axes[0].set(xlabel='Measured settled angle [deg]',ylabel='True qdot=0 equilibrium [deg]');axes[0].legend()
    profile=json.loads((out/'x1_profile_loss.json').read_text())
    axes[1].plot([p['x1_zero_m']*1000 for p in profile],[p['loss_rmse_nm'] for p in profile],'o-')
    axes[1].set(xlabel='Fixed x1 zero [mm]',ylabel='Reoptimized equation RMSE [Nm]',title='Conditional objective; not a confidence interval')
    for ax in axes:ax.grid(alpha=.25)
    fig.tight_layout();fig.savefig(out/'static_and_identifiability.png',dpi=180);plt.close(fig)
    dump(out/'audit_provenance.json',dict(paper_sha256=sha256('main.tex'),
         code_sha256={str(p):sha256(p) for p in Path('ph_model').glob('*.py')}))
    print('AUDIT DONE',flush=True)


if __name__=='__main__':main()
