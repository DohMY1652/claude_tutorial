"""Additional nominal-only rollout fitting and identifiability diagnostics.

All optimizations use the fixed train split. Development seed 3 is used only
for comparison; seed 4 is not used for parameter or hyperparameter selection.
"""
import argparse
import dataclasses
from pathlib import Path
import json
import time
import numpy as np
from scipy.optimize import least_squares
import yaml

from .data import load_run, stationary_points, sha256
from .nominal import Parameters, torque, gradient
from .fit_nominal import RUNS, pack, fit_equation, spans, simulate, evaluate, plot_rollout, dump


def windows(runs):
    result=[]
    for run in runs:
        for ch in run['chunks']:
            intervals=spans(ch['good'],minimum=1500)
            if not intervals:
                continue
            a,b=max(intervals,key=lambda ab:ab[1]-ab[0])
            # Three disjoint 10 s windows per profile, with equal profile weighting.
            for offset in [.1,.45,.8]:
                start=a+int((b-a-1000)*offset)
                ix=np.arange(start,start+1001,10)
                result.append(dict(t=ch['t'][ix],pressure=ch['pressure'][ix],
                                   q=ch['q'][ix],z0=[ch['qs'][start],ch['v'][start]],
                                   run=Path(run['path']).name))
    return result


def fit_rollout(wins, base, elastic, starts=2):
    names=['x1_zero_m','alpha','damping_nm_s_rad'] + \
        (['elastic_k_nm_rad','elastic_bias_nm'] if elastic else ['gravity_nm'])
    lo=np.array([.05,0,0]+([0,-2] if elastic else [2.943]))
    hi=np.array([.15,2,100]+([10,2] if elastic else [6]))
    def decode(z):
        return dataclasses.replace(base,**dict(zip(names,map(float,lo+(hi-lo)*z))))
    count=0
    def fun(z):
        nonlocal count
        p=decode(z); errors=[]
        for w in wins:
            pred=simulate(w['t'],w['pressure'],w['z0'],p,rtol=1e-6,max_step=.05)
            errors.extend((pred[:,0]-w['q'])/np.sqrt(len(w['q'])*len(wins)))
        count+=1
        if count%20==0:
            print('objective',count,'RMSE_deg',np.rad2deg(np.linalg.norm(errors)),flush=True)
        return errors
    initial=np.clip((np.array([getattr(base,n) for n in names])-lo)/(hi-lo),1e-5,1-1e-5)
    trials=[]; best=None
    for i in range(starts):
        z=initial.copy()
        if i:
            z[1]=min(.9,z[1]*1.5+.05);z[2]=min(.8,z[2]*2+.02)
        res=least_squares(fun,z,bounds=(np.zeros(len(z)),np.ones(len(z))),diff_step=.002,
                          ftol=2e-4,xtol=2e-4,gtol=1e-5,max_nfev=60)
        trials.append(dict(cost=float(res.cost),success=bool(res.success),nfev=res.nfev,message=res.message))
        if best is None or res.cost<best.cost: best=res
    return decode(best.x),dict(window_rmse_deg=float(np.rad2deg(np.linalg.norm(best.fun))),
                               trials=trials,n_windows=len(wins),window_length_s=10,
                               fitted_names=names,boundary_parameters=[n for n,z in zip(names,best.x) if min(z,1-z)<1e-4])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-root',type=Path,default=Path('/home/risebrl/result/ph/4'))
    parser.add_argument('--output',type=Path,default=Path('reports/nominal_actuator4_20261006'))
    parser.add_argument('--skip-refit',action='store_true')
    args=parser.parse_args();out=args.output
    runs=[]
    for name,role in RUNS:
        if role=='supplementary':continue
        r=load_run(args.data_root/name,cutoff=356.45 if role=='validation_prefix' else None)
        r['role']=role;runs.append(r)
    fits=json.loads((out/'fits.json').read_text())
    train=[r for r in runs if r['role']=='train']
    wins=windows(train)
    dump(out/'rollout_windows.json',[dict(run=w['run'],start_s=float(w['t'][0]),end_s=float(w['t'][-1])) for w in wins])
    refined=json.loads((out/'refined_fits.json').read_text()) if args.skip_refit else {}
    if not args.skip_refit:
        for name,source,elastic in [('Ve0_rollout','primary_Ve0',False),('elastic_rollout','elastic_sensitivity',True)]:
            print('Refine',name,len(wins),'windows',flush=True)
            p,info=fit_rollout(wins,Parameters(**fits[source]['parameters']),elastic)
            refined[name]=dict(parameters=dataclasses.asdict(p),fit=info)
            dump(out/'refined_fits.json',refined)
            print(name,refined[name],flush=True)
    scores={}
    # Evaluate alternative nominal forms independently, without validation-driven selection.
    models={name:Parameters(**v['parameters']) for name,v in refined.items()}
    models['elastic_equation']=Parameters(**fits['elastic_sensitivity']['parameters'])
    for name,p in models.items():
        scores[name]={}
        (out/f'{name}.yaml').write_text(yaml.safe_dump(dataclasses.asdict(p),sort_keys=False))
        for run in runs:
            label=Path(run['path']).name
            print('Evaluate',name,label,flush=True)
            score,saved=evaluate(run,p,out/'artifacts'/f'{name}_{label}.npz')
            scores[name][label]=score
            if run['role'] in ('development','validation_prefix') or run['meta']['profile']['id'] in ('S2','S4'):
                plot_rollout(saved,f'{name} | {run["meta"]["profile"]["id"]} seed {run["meta"]["profile"]["seed"]} | {run["role"]}',out/f'{name}_{label}.png')
            dump(out/'refined_metrics.json',scores)
    # Conditional loss curve is not a statistical confidence interval.
    data,weight=pack(train)
    profile=[]
    for x in np.linspace(.04,.15,12):
        p,info=fit_equation(data,weight,Parameters.load(),starts=2,fixed={'x1_zero_m':float(x)})
        profile.append(dict(x1_zero_m=float(x),loss_rmse_nm=info['weighted_rmse_nm'],parameters=dataclasses.asdict(p)))
    dump(out/'x1_profile_loss.json',profile)
    # Filter-width sensitivity and half/double inertia refit on training only.
    sensitivities={}
    for window in [.15,.61]:
        rr=[load_run(r['path'],window_s=window) for r in train]
        d,w=pack(rr)
        p,info=fit_equation(d,w,Parameters.load(),starts=2)
        sensitivities[f'filter_{window}']=dict(parameters=dataclasses.asdict(p),fit=info)
    for J in [.0225,.09]:
        p,info=fit_equation(data,weight,dataclasses.replace(Parameters.load(),inertia_kg_m2=J),starts=2)
        sensitivities[f'inertia_{J}']=dict(parameters=dataclasses.asdict(p),fit=info)
    dump(out/'refit_sensitivity.json',sensitivities)
    dump(out/'refinement_provenance.json',dict(code_sha256={str(p):sha256(p) for p in Path('ph_model').glob('*.py')},
         paper_sha256=sha256('main.tex'),validation_used_for_fit=False))
    print('REFINEMENT DONE',flush=True)


if __name__=='__main__':main()
