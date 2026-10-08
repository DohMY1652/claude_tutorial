"""Offline refit of slow friction versus move/hold behavior, preserving main.tex."""
import argparse
import copy
import csv
import dataclasses
import json
from pathlib import Path
import time

import numpy as np
import torch
from scipy.optimize import least_squares

from .data import load_run, sha256
from .nominal import Parameters, torque, gradient
from .fit_nominal import RUNS, dump, metrics
from .fit_residual import blocks, evaluate, training_arrays, train_steps
from .residual import PassiveResidual
from .residual_fast import simulate
from .refit_nominal import zero_model
from .improve_residual import parameterization

BASE=Path('reports/residual_actuator4_20261007')
OLD=BASE/'nominal_refit_v3'
OUT=BASE/'motion_refit_v4'


def motion_features(q):
    """Fixed measurement-derived windows; never mask based on a prediction."""
    n=len(q)
    step=np.arange(0,max(0,n-10),5)
    step=step[np.abs(q[step+10]-q[step])>np.deg2rad(.3)]
    hold=np.arange(0,max(0,n-100),5)
    hold=np.array([i for i in hold if np.ptp(q[i:i+101])<np.deg2rad(.3)],dtype=int)
    return dict(step=step,hold=hold)


def prepared(run):
    bs=blocks(run)
    for b in bs:b['features']=motion_features(b['q'])
    return bs


def fast(b,obj,substeps=5):
    return simulate(b,obj,substeps=substeps,method='implicit_euler')


def score_predictions(bb,predictions):
    errors=[];step=[];hold=[];held_motion=[]
    for b,z in zip(bb,predictions):
        e=z[:,0]-b['q'];errors.extend(e)
        for kind,lag,target in [('step',10,step),('hold',100,hold)]:
            i=b['features'][kind]
            target.extend(e[i+lag]-e[i])
        i=b['features']['hold'];held_motion.extend(z[i+100,0]-z[i,0])
    ans=metrics(np.rad2deg(errors))
    for name,arr in [('step_1s_error',step),('hold_10s_error',hold),('hold_10s_predicted_motion',held_motion)]:
        ans[name+'_rmse_deg']=float(np.sqrt(np.mean(np.rad2deg(arr)**2))) if len(arr) else None
        ans[name+'_windows']=len(arr)
    ans['selection_score_deg']=float(np.sqrt(ans['rmse_deg']**2+
        4*(ans['step_1s_error_rmse_deg'] or 0)**2+4*(ans['hold_10s_error_rmse_deg'] or 0)**2))
    return ans


def fastscore(bb,obj):return score_predictions(bb,[fast(b,obj) for b in bb])


def objective_factory(bb,decode,label):
    counts={}
    for b in bb:
        c=counts.setdefault(b['run'],dict(angle=0,step=0,hold=0))
        c['angle']+=len(b['q'][::5])
        for kind in ('step','hold'):c[kind]+=len(b['features'][kind])
    length=sum(sum(c.values()) for c in counts.values());calls=0;tic=time.time()
    def objective(z):
        nonlocal calls
        out=[];obj=decode(z)
        try:
            for b in bb:
                pred=fast(b,obj)[:,0];e=pred-b['q'];c=counts[b['run']]
                out.extend(e[::5]/np.sqrt(c['angle']*len(counts)))
                for kind,lag in [('step',10),('hold',100)]:
                    i=b['features'][kind]
                    if len(i):out.extend(2*(e[i+lag]-e[i])/np.sqrt(c[kind]*len(counts)))
            out=np.asarray(out)
        except ValueError:out=np.full(length,100/np.sqrt(length))
        calls+=1
        if calls%20==0:
            print(label,calls,'combined_loss_deg',np.rad2deg(np.linalg.norm(out)),
                  'seconds',round(time.time()-tic),flush=True)
        return out
    return objective


def optimize(fun,initial,lo,hi,nfev):
    def jac(z):
        f=fun(z);cols=[]
        for i in range(len(z)):
            h=1e-4 if z[i]+1e-4<hi[i] else -1e-4
            v=z.copy();v[i]+=h;cols.append((fun(v)-f)/h)
        return np.column_stack(cols)
    return least_squares(fun,np.clip(initial,lo+1e-8,hi-1e-8),jac=jac,bounds=(lo,hi),
                         max_nfev=nfev,ftol=1e-4,xtol=1e-4,gtol=1e-6,x_scale='jac')


def nominal_stage(runs,train,dev):
    old=json.loads((OLD/'residual/central_nominal.json').read_text())
    oldfull=json.loads((OLD/'residual/selected_model.json').read_text())
    # Diagnose every training/development run without consulting final seed4.
    diagnostic={}
    for r in runs:
        name=Path(r['path']).name;bb=prepared(r)
        entries={}
        for label,obj,filelabel in [('nominal',old,'nominal'),('full',oldfull,'residual')]:
            with np.load(OLD/'residual/artifacts'/f'{filelabel}_{name}.npz') as a:
                entries[label]=score_predictions(bb,[a[f'block{i}_prediction'] for i in range(len(bb))])
        diagnostic[name]=entries
    dump(OUT/'before_motion_metrics.json',diagnostic)
    dump(OUT/'protocol.json',dict(paper_sha256=sha256('main.tex'),
        old_full_sha256=sha256(OLD/'residual/selected_model.json'),
        objective='Equal run angle error + 2x moving 1s increment error + 2x held 10s increment error',
        movement='Observed abs 1s increment > 0.3 deg; start samples every 0.5s',
        hold='Observed 10s peak-to-peak < 0.3 deg; start samples every 0.5s',
        epsilon_bounds_rad_s=[1e-6,.05],epsilon_form_unchanged='tanh(v/epsilon), epsilon strictly positive',
        fixed=dict(gravity_nm=2.943,reel_radius_m=.025,inertia_kg_m2=.045,x1_zero_m=.05),
        selection='development seed3 composite score; historical seed4 after selection, not blind',
        integration='implicit Euler 20ms for fitting; independent adaptive validation',
        files=[dict(path=r['path'],role=r['role'],**r['audit']) for r in runs]))
    # Geometry/Mg fixed, avoiding their previously observed compensation.
    template=zero_model(dataclasses.replace(Parameters(),x1_zero_m=.05))
    def decode(z):
        obj=copy.deepcopy(template)
        obj['nominal'].update(alpha=float(z[0]),damping_nm_s_rad=float(z[1]),
            elastic_k_nm_rad=float(z[2]),elastic_bias_nm=float(z[3]),epsilon_rad_s=float(np.exp(z[4])))
        return obj
    lo=np.array([0,.0001,0,-3,np.log(1e-6)])
    hi=np.array([2,20,8,3,np.log(.05)])
    starts=[[.25,.5,1.13,-.83,np.log(.002)], [.2,.5,1.13,-.83,np.log(.0001)],
            [.60,1.,1.74,-1.49,np.log(.02)]]
    candidates=[]
    for j,start in enumerate(starts):
        name=f'nominal_start{j}';fn=objective_factory(train,decode,name)
        fit=optimize(fn,np.array(start),lo,hi,50);obj=decode(fit.x)
        dump(OUT/f'{name}.json',obj)
        candidates.append(dict(name=name,development=fastscore(dev,obj),parameters=obj['nominal'],
                               success=bool(fit.success),nfev=fit.nfev,training_loss_deg=float(np.rad2deg(np.linalg.norm(fit.fun)))))
        dump(OUT/'nominal_candidates.json',candidates);print('NOMINAL CANDIDATE',candidates[-1],flush=True)
    best=min(candidates,key=lambda c:c['development']['selection_score_deg'])
    dump(OUT/'nominal_selected.json',json.loads((OUT/(best['name']+'.json')).read_text()))
    dump(OUT/'nominal_selection.json',best)


def residual_stage(runs,train,dev):
    nominal=json.loads((OUT/'nominal_selected.json').read_text())
    p=Parameters(**nominal['nominal']);data=training_arrays(train,p)
    torch.manual_seed(1801)
    m=PassiveResidual(p,(float(data['q'].min()),float(data['q'].max())),np.max(np.abs(data['P']),axis=0))
    dump(OUT/'residual_zero_initialized.json',m.export())
    trace=train_steps(m,data,np.zeros(len(data['q'])),600,1801)
    dump(OUT/'equation_trace.json',trace)
    original=m.export();original['state']['history_k']=[.05,10.];original['state']['history_rho']=[.5,3.]
    initial,lo,hi,decode=parameterization(original)
    lo[21:23]=np.log([1e-6,1e-6])
    candidates=[dict(name='nominal_selected',development=fastscore(dev,nominal))]
    for j in range(2):
        z=initial.copy()
        if j:z[21:23]=np.log([1.,30.]);z[23:25]=np.log([1.,3.])
        name=f'residual_start{j}';fn=objective_factory(train,decode,name)
        fit=optimize(fn,z,lo,hi,35);obj=decode(fit.x)
        assert obj['nominal']==nominal['nominal']
        dump(OUT/f'{name}.json',obj)
        candidates.append(dict(name=name,development=fastscore(dev,obj),success=bool(fit.success),
                               nfev=fit.nfev,training_loss_deg=float(np.rad2deg(np.linalg.norm(fit.fun)))))
        dump(OUT/'residual_candidates.json',candidates);print('RESIDUAL CANDIDATE',candidates[-1],flush=True)
    best=min(candidates,key=lambda c:c['development']['selection_score_deg'])
    dump(OUT/'full_selected.json',json.loads((OUT/(best['name']+'.json')).read_text()))
    dump(OUT/'full_selection.json',best)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--stage',choices=['nominal','residual'],default='nominal')
    args=ap.parse_args();OUT.mkdir(parents=True,exist_ok=True);torch.set_num_threads(1)
    runs=[];train=[];dev=[]
    for name,role in RUNS:
        if role not in ('train','development'):continue
        r=load_run(Path('/home/risebrl/result/ph/4')/name);r['role']=role;runs.append(r)
        (train if role=='train' else dev).extend(prepared(r))
    (nominal_stage if args.stage=='nominal' else residual_stage)(runs,train,dev)
    print('MOTION REFIT STAGE COMPLETE',args.stage,flush=True)


if __name__=='__main__':main()
