"""Frozen-nominal passive residual refinement on complete training trajectories.

No measured angle/velocity enters the dynamics after each valid block starts.
Historical seed 4 is diagnosed data, not a blind test; it is loaded only after
development-based selection. Never change the baseline artifacts in place.
"""
import argparse
import copy
import json
from pathlib import Path
import time

import numpy as np
import torch
from scipy.optimize import least_squares

from .data import load_run, sha256
from .fit_nominal import RUNS, dump, metrics
from .fit_residual import blocks, evaluate, plot_comparison
from .residual import PassiveResidual
from .residual_fast import simulate


BASE=Path('reports/residual_actuator4_20261007')


def parameterization(original):
    s=original['state']
    head=np.r_[np.array(s['potential_net.4.weight']).ravel(),s['potential_net.4.bias']]
    initial=np.r_[head,s['mu_mix'],np.log(s['history_k']),np.log(s['history_rho'])]
    lo=np.r_[head-1.5,np.zeros(4),np.log([.002,.002]),np.log([.02,.02])]
    hi=np.r_[head+1.5,np.full(4,3.),np.log([150.,150.]),np.log([50.,50.])]
    def decode(z):
        o=copy.deepcopy(original);st=o['state']
        st['potential_net.4.weight']=[z[:16].tolist()];st['potential_net.4.bias']=[float(z[16])]
        st['mu_mix']=z[17:21].tolist();st['history_k']=np.exp(z[21:23]).tolist()
        st['history_rho']=np.exp(z[23:25]).tolist()
        return o
    return initial,lo,hi,decode


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,default=BASE/'long_rollout_v2')
    ap.add_argument('--nfev',type=int,default=22)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
    torch.set_num_threads(1)
    original=json.loads((BASE/'best_model.json').read_text())
    initial,lo,hi,decode=parameterization(original)
    runs=[];blks=[];train_sizes={}
    for name,role in RUNS:
        if role not in ('train','development'):continue
        r=load_run(Path('/home/risebrl/result/ph/4')/name);r['role']=role;runs.append(r)
        if role=='train':
            bb=blocks(r);blks.extend(bb);train_sizes[name]=sum(len(b['q'][::5]) for b in bb)
    dev=next(r for r in runs if r['role']=='development')
    dev_blocks=blocks(dev)
    def devscore(obj):
        return metrics(np.concatenate([np.rad2deg(simulate(b,obj)[:,0]-b['q']) for b in dev_blocks]))
    candidates=[dict(name='unchanged_baseline',development=devscore(original))]
    dump(out/'unchanged_baseline.json',original)
    dump(out/'protocol.json',dict(baseline_sha256=sha256(BASE/'best_model.json'),paper_sha256=sha256('main.tex'),
         train=[r['path'] for r in runs if r['role']=='train'],development=dev['path'],
         objective='Equal weight per training run; complete valid blocks, no teacher forcing; angle error at 2Hz',
         pressure_rate_hz=10,midpoint_step_s=.05,nominal_fixed=True,epsilon_fixed=True,
         potential_head_learned=True,hidden_layers_fixed=True,nonnegative_mu_learned=True,
         history_k_rho_learned=True,history_rv_fixed=True,d_mix_fixed=True,
         area_residual=False,validation='Previously diagnosed seed4 prefix; not blind; no selection',
         train_files=[dict(path=r['path'],**r['audit']) for r in runs if r['role']=='train']))
    weights=[1/np.sqrt(train_sizes[b['run']]*len(train_sizes)) for b in blks]
    total_rows=sum(len(b['q'][::5]) for b in blks)
    calls=0;trace=[];start=time.time()
    def objective(z):
        nonlocal calls
        obj=decode(z)
        try:
            errors=np.concatenate([(simulate(b,obj)[::5,0]-b['q'][::5])*w for b,w in zip(blks,weights)])
        except ValueError:
            errors=np.full(total_rows,10/np.sqrt(total_rows))
        # Potential weight anchor only: prohibit neither positive history nor friction changes.
        residual=np.r_[errors,.001*(z[:17]-initial[:17])]
        calls+=1
        if calls%25==0:
            value=float(np.rad2deg(np.linalg.norm(errors)))
            print('FULL TRAIN',calls,'equal-run RMSE',value,'elapsed',round(time.time()-start,1),flush=True)
            trace.append(dict(call=calls,training_rmse_deg=value,elapsed_s=time.time()-start))
            dump(out/'progress.json',trace)
        return residual
    def jac(z):
        f0=objective(z);cols=[]
        for i in range(len(z)):
            h=1e-4 if z[i]+1e-4<=hi[i] else -1e-4
            zz=z.copy();zz[i]+=h;cols.append((objective(zz)-f0)/h)
        return np.column_stack(cols)
    # Fixed starts declared before checking final diagnostic data.
    starts=[initial.copy(),initial.copy()]
    starts[1][21:23]=np.log([5.,30.]);starts[1][23:25]=np.log([1.,3.])
    for j,z0 in enumerate(starts):
        print('START',j,'baseline development',candidates[0]['development']['rmse_deg'],flush=True)
        result=least_squares(objective,np.clip(z0,lo+1e-8,hi-1e-8),jac=jac,bounds=(lo,hi),
                            max_nfev=args.nfev,ftol=2e-4,xtol=2e-4,gtol=1e-6,x_scale='jac')
        name=f'full_rollout_start{j}';obj=decode(result.x)
        dump(out/f'{name}.json',obj)
        try:score=devscore(obj)
        except ValueError as exc:
            candidates.append(dict(name=name,failed=str(exc)));continue
        entry=dict(name=name,development=score,training_rmse_deg=float(np.rad2deg(np.linalg.norm(result.fun[:total_rows]))),
                   success=bool(result.success),message=result.message,nfev=result.nfev,
                   nominal_unchanged=obj['nominal']==original['nominal'])
        candidates.append(entry);dump(out/'candidates.json',candidates)
        print('CANDIDATE',entry,flush=True)
    # Validate shortlisted candidates with a separate adaptive solver before selection.
    for c in candidates:
        if 'development' not in c:continue
        obj=json.loads((out/f"{c['name']}.json").read_text())
        score,_=evaluate(dev,PassiveResidual.restore(obj))
        c['adaptive_development']=score
    dump(out/'candidates.json',candidates)
    best=min([c for c in candidates if 'adaptive_development' in c],key=lambda c:c['adaptive_development']['rmse_deg'])
    selected=json.loads((out/f"{best['name']}.json").read_text())
    dump(out/'selected_model.json',selected);dump(out/'selection.json',best)
    # Comparison only; no optimizer sees seed4, and no promotion based on seed4.
    name=next(n for n,r in RUNS if r=='validation_prefix')
    val=load_run(Path('/home/risebrl/result/ph/4')/name,cutoff=356.45)
    val['role']='historical_diagnostic_prefix';runs.append(val)
    allscores={}
    for r in runs:
        name=Path(r['path']).name
        score,arr=evaluate(r,PassiveResidual.restore(selected),out/'artifacts',f'residual_{name}')
        with np.load(BASE/'ve0_baseline/artifacts'/f'residual_{name}.npz') as old:
            for key in old.files:
                if key.endswith(('_t','_P','_q')):np.testing.assert_array_equal(old[key],arr[key])
            plot_comparison(old,arr,out/f'{name}.png',r['role'],nominal_label='Previous passive residual',residual_label='Long-rollout passive residual')
        allscores[name]=dict(role=r['role'],residual=score)
        dump(out/'metrics.json',allscores)
        print('FINAL',name,score['rmse_deg'],flush=True)
    dump(out/'source_hashes.json',{str(f):sha256(f) for f in Path('ph_model').glob('residual_fast.*')})
    print('LONG ROLLOUT COMPLETE',best['name'],flush=True)


if __name__=='__main__':main()
