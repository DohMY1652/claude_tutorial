"""Residual retraining on central-branch nominal, never on a gravity-boundary fit."""
import copy
import dataclasses
import json
from pathlib import Path
import time

import numpy as np
import torch
from scipy.optimize import least_squares

from .data import load_run,sha256
from .fit_nominal import RUNS,dump,metrics
from .fit_residual import blocks,training_arrays,train_steps,evaluate,plot_comparison
from .residual import PassiveResidual
from .residual_fast import simulate
from .nominal import Parameters
from .improve_residual import parameterization
from .refit_nominal import OUT,BASE,zero_model


def main():
    torch.set_num_threads(1);torch.manual_seed(1708)
    out=OUT/'residual';out.mkdir(parents=True,exist_ok=True)
    choices=json.loads((OUT/'nominal_candidates.json').read_text())
    # Physical screening: the 0.2-Nm gravity boundary solution is diagnostic,
    # not a measured mass nor a defensible replacement for the central fit.
    p=Parameters(**choices['fixed25_central_then_friction']['parameters'])
    runs=[];blks=[]
    for name,role in RUNS:
        if role not in ('train','development'):continue
        r=load_run(Path('/home/risebrl/result/ph/4')/name);r['role']=role;runs.append(r)
        if role=='train':blks.extend(blocks(r))
    dev=next(r for r in runs if r['role']=='development')
    data=training_arrays(blks,p)
    bounds=(float(data['q'].min()),float(data['q'].max()));scale=np.max(np.abs(data['P']),axis=0)
    m=PassiveResidual(p,bounds,scale)
    dump(out/'zero_initialized.json',m.export())
    dump(out/'protocol.json',dict(nominal_source='fixed25_central_then_friction',nominal=dataclasses.asdict(p),
        nominal_is_effective_not_uniquely_identified=True,gravity_boundary_candidate_rejected=True,
        random_seed=1708,zero_residual_initialization=True,equation_initialization_steps=600,
        full_rollout_no_teacher_forcing=True,nominal_frozen=True,area_correction=False,
        reference_model_sha256=sha256(BASE/'long_rollout_v2/selected_model.json'),
        paper_sha256=sha256('main.tex'),files=[dict(path=r['path'],role=r['role'],**r['audit']) for r in runs]))
    trace=train_steps(m,data,np.zeros(len(data['q'])),600,1708)
    dump(out/'equation_trace.json',trace);dump(out/'equation_initialized.json',m.export())
    # Warm start positive history optimization after checking the zero-history model.
    template=m.export();template['state']['history_k']=[.05,10.];template['state']['history_rho']=[.5,3.]
    initial,lo,hi,decode=parameterization(template)
    sizes={b['run']:sum(len(bb['q'][::5]) for bb in blks if bb['run']==b['run']) for b in blks}
    weights=[1/np.sqrt(sizes[b['run']]*len(sizes)) for b in blks];N=sum(len(b['q'][::5]) for b in blks)
    calls=0;start=time.time();progress=[]
    def fun(z):
        nonlocal calls
        obj=decode(z)
        try:e=np.concatenate([(simulate(b,obj)[::5,0]-b['q'][::5])*w for b,w in zip(blks,weights)])
        except ValueError:e=np.full(N,10/np.sqrt(N))
        calls+=1
        if calls%25==0:
            record=dict(call=calls,training_equal_run_rmse_deg=float(np.rad2deg(np.linalg.norm(e))),elapsed_s=time.time()-start)
            print('RETRAIN',record,flush=True);progress.append(record);dump(out/'progress.json',progress)
        return np.r_[e,.001*(z[:17]-initial[:17])]
    def jac(z):
        f=fun(z);cols=[]
        for i in range(len(z)):
            h=1e-4 if z[i]+1e-4<=hi[i] else -1e-4
            zz=z.copy();zz[i]+=h;cols.append((fun(zz)-f)/h)
        return np.column_stack(cols)
    objects={'central_nominal':zero_model(p),'equation_initialized':m.export(),
             'previous_v2':json.loads((BASE/'long_rollout_v2/selected_model.json').read_text())}
    candidates=[]
    starts=[initial.copy(),initial.copy()];starts[1][21:23]=np.log([5.,30.]);starts[1][23:25]=np.log([1.,3.])
    for i,z in enumerate(starts):
        r=least_squares(fun,np.clip(z,lo+1e-8,hi-1e-8),jac=jac,bounds=(lo,hi),x_scale='jac',
                        max_nfev=25,ftol=2e-4,xtol=2e-4,gtol=1e-6)
        name=f'central_residual_start{i}';objects[name]=decode(r.x);dump(out/f'{name}.json',objects[name])
        score=metrics(np.concatenate([np.rad2deg(simulate(b,objects[name])[:,0]-b['q']) for b in blocks(dev)]))
        candidates.append(dict(name=name,training_rmse_deg=float(np.rad2deg(np.linalg.norm(r.fun[:N]))),
             success=bool(r.success),nfev=r.nfev,development_fast=score))
        dump(out/'candidates.json',candidates);print('CANDIDATE',candidates[-1],flush=True)
    for name,obj in objects.items():
        dump(out/f'{name}.json',obj)
        try:score,_=evaluate(dev,PassiveResidual.restore(obj))
        except (ValueError,RuntimeError) as exc:
            candidates.append(dict(name=name,failed=str(exc)));continue
        c=next((c for c in candidates if c['name']==name),None)
        if c is None:c=dict(name=name);candidates.append(c)
        c['development_adaptive']=score
    dump(out/'candidates.json',candidates)
    best=min([c for c in candidates if 'development_adaptive' in c],key=lambda c:c['development_adaptive']['rmse_deg'])
    selected=objects[best['name']]
    dump(out/'selected_model.json',selected);dump(out/'selection.json',best)
    # All model choices fixed above. Seed4 is a historical diagnostic only.
    name=next(n for n,r in RUNS if r=='validation_prefix')
    val=load_run(Path('/home/risebrl/result/ph/4')/name,cutoff=356.45);val['role']='historical_diagnostic_prefix';runs.append(val)
    scores={}
    for r in runs:
        name=Path(r['path']).name
        ns,na=evaluate(r,PassiveResidual.restore(zero_model(p)),out/'artifacts',f'nominal_{name}')
        rs,ra=evaluate(r,PassiveResidual.restore(selected),out/'artifacts',f'residual_{name}')
        scores[name]=dict(role=r['role'],central_nominal=ns,selected_residual=rs)
        dump(out/'metrics.json',scores)
        plot_comparison(na,ra,out/f'{name}.png',r['role'],nominal_label='Central-branch nominal',residual_label=best['name'])
        print('FINAL V3',name,ns['rmse_deg'],rs['rmse_deg'],flush=True)
    print('PHYSICAL RESIDUAL COMPLETE',best['name'],flush=True)


if __name__=='__main__':main()
