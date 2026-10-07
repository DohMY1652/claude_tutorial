"""Train passive residuals on actuator 4. Fixed nominal, profile-level split.

No control code is imported. Development selects candidates; historical seed 4
prefix is evaluated once after selection, never passed to optimization.
"""
import argparse
import copy
import dataclasses
import json
import math
from pathlib import Path
import time
import numpy as np
import torch
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .data import load_run,sha256
from .nominal import Parameters,torque,gradient,friction
from .fit_nominal import RUNS,spans,metrics,dump
from .residual import PassiveResidual,NumpyResidual


def blocks(run,minimum=1000):
    ans=[]
    for ch in run['chunks']:
        for a,b in spans(ch['good'],minimum):
            ix=np.arange(a,b,10)
            ans.append(dict(t=ch['t'][ix],q=ch['q'][ix],qs=ch['qs'][ix],v=ch['v'][ix],
                            acc=ch['acc'][ix],P=ch['pressure'][ix],run=Path(run['path']).name))
    return ans


def observed_history(blks,k,rv,rho,components=False):
    """Exact interval update for linear observed q and constant R per interval.

    Observer uses training q only; free evaluation integrates independent xi.
    """
    result=[]
    for b in blks:
        t,q,v=b['t'],b['qs'],b['v'];force=np.zeros((len(t),2))
        for j in range(2):
            kj,rvj,rhoj=float(k[j]),float(rv[j]),float(rho[j]);f=0.
            for i in range(1,len(t)):
                dt=t[i]-t[i-1]
                rate=kj*(rvj+rhoj*abs((v[i]+v[i-1])/2))
                z=rate*dt
                decay=math.exp(-z)
                factor=-math.expm1(-z)/z if z>1e-10 else 1-z/2
                f=decay*f+kj*(q[i]-q[i-1])*factor
                force[i,j]=f
        result.append(force if components else force.sum(axis=1))
    return np.concatenate(result)


def training_arrays(blks,p):
    # Equal total weight for each original training run, not each block/sample.
    counts={}
    for b in blks:counts[b['run']]=counts.get(b['run'],0)+len(b['q'])
    q=np.concatenate([b['qs'] for b in blks]);v=np.concatenate([b['v'] for b in blks])
    a=np.concatenate([b['acc'] for b in blks]);P=np.concatenate([b['P'] for b in blks])
    weight=np.concatenate([np.full(len(b['q']),1/counts[b['run']]/len(counts)) for b in blks])
    tau=torque(q,P,p)
    target=tau-gradient(q,p)-friction(v,tau,p)-p.inertia_kg_m2*a
    return dict(q=q,v=v,P=P,target=target,weight=weight)


def nn_terms(model,data):
    out=[]
    for a in range(0,len(data['q']),4096):
        q=torch.tensor(data['q'][a:a+4096],dtype=torch.float64,requires_grad=True)
        P=torch.tensor(data['P'][a:a+4096],dtype=torch.float64)
        V,g,d,mu=model.slots(q,P,create_graph=False)
        v=torch.tensor(data['v'][a:a+4096],dtype=torch.float64)
        f=g+d*v+mu*torch.tanh(v/model.nominal.epsilon_rad_s)
        out.extend(f.detach().numpy())
    return np.array(out)


def train_steps(model,data,history,steps,seed,callback=None):
    rng=np.random.default_rng(seed)
    optimizer=torch.optim.Adam([p for p in model.parameters() if p.requires_grad],lr=.003,weight_decay=1e-6)
    trace=[]
    for step in range(1,steps+1):
        ix=rng.choice(len(data['q']),size=2048,replace=True,p=data['weight'])
        q=torch.tensor(data['q'][ix],dtype=torch.float64,requires_grad=True)
        P=torch.tensor(data['P'][ix],dtype=torch.float64)
        v=torch.tensor(data['v'][ix],dtype=torch.float64)
        target=torch.tensor(data['target'][ix]-history[ix],dtype=torch.float64)
        V,g,d,mu=model.slots(q,P)
        error=g+d*v+mu*torch.tanh(v/model.nominal.epsilon_rad_s)-target
        loss=(error*error).mean()+1e-4*((g*g).mean()+(d*v).square().mean()+mu.square().mean())
        optimizer.zero_grad();loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(),5.)
        optimizer.step();model.project()
        for group in optimizer.param_groups:group['lr']=.0005+.0025*(1-step/steps)
        if step%200==0:
            full=nn_terms(model,data)+history-data['target']
            rmse=float(np.sqrt(np.sum(data['weight']*full**2)))
            trace.append(dict(step=step,equation_rmse_nm=rmse))
            print('NN',seed,step,'equation RMSE',rmse,flush=True)
        if callback and step%600==0:callback(model,step)
    return trace


def fit_history(model,blks,data):
    remaining=data['target']-nn_terms(model,data)
    weight=np.sqrt(data['weight'])
    k=model.history_k.detach().numpy();rv=model.history_rv.numpy();rho=model.history_rho.numpy()
    def decode(z):return z[:2]*10,np.exp(z[2:4]),np.exp(z[4:])
    def fun(z):
        kk,rr,hh=decode(z)
        return weight*(observed_history(blks,kk,rr,hh)-remaining)
    x=np.r_[k/10,np.log(rv),np.log(rho)]
    lo=np.r_[[0.,0.],np.log([1e-6,1e-6]),np.log([.05,.05])]
    hi=np.r_[[10.,10.],np.log([.1,.1]),np.log([30.,30.])]
    # Start K=0 on the first history stage: exact nominal+NN consistency.
    res=least_squares(fun,np.clip(x,lo+1e-8,hi-1e-8),bounds=(lo,hi),
                      diff_step=1e-3,ftol=1e-6,xtol=1e-6,gtol=1e-7,max_nfev=100)
    k,rv,rho=decode(res.x)
    with torch.no_grad():
        model.history_k.copy_(torch.from_numpy(k));model.history_rv.copy_(torch.from_numpy(rv));model.history_rho.copy_(torch.from_numpy(rho))
    return dict(k=k.tolist(),rv=rv.tolist(),rho=rho.tolist(),success=bool(res.success),
                nfev=res.nfev,equation_rmse_nm=float(np.linalg.norm(res.fun))),observed_history(blks,k,rv,rho)


def simulate_block(b,model,rtol=2e-5,max_step=.05):
    n=NumpyResidual(model.export());p=n.nominal;t=b['t'];P=b['P']
    pos=P[:,1].copy();neg=P[:,0].copy()
    def rhs(time,z):
        q,v=z[:2];xi=z[2:]
        pressure=np.array([np.interp(time,t,neg),np.interp(time,t,pos)])
        tau=float(torque(q,pressure,p));_,g,d,mu=n.slots(q,pressure)
        fh=n.k*(q-xi);R=n.rv+n.rho*abs(v)
        acc=(tau-float(gradient(q,p))-g-float(friction(v,tau,p))-d*v-mu*np.tanh(v/p.epsilon_rad_s)-sum(fh))/p.inertia_kg_m2
        return np.r_[v,acc,R*fh]
    z0=[b['qs'][0],b['v'][0],b['qs'][0],b['qs'][0]]
    sol=solve_ivp(rhs,[t[0],t[-1]],z0,t_eval=t,method='LSODA',rtol=rtol,atol=rtol*.01,max_step=max_step)
    if not sol.success:raise RuntimeError(sol.message)
    return sol.y.T


def evaluate(run,model,output=None,label=None):
    errors=[];saved={};blocks_info=[]
    for i,b in enumerate(blocks(run)):
        pred=simulate_block(b,model)
        err=np.rad2deg(pred[:,0]-b['q']);errors.extend(err)
        blocks_info.append(dict(start_s=float(b['t'][0]),end_s=float(b['t'][-1]),**metrics(err)))
        for key,value in dict(t=b['t'],q=b['q'],prediction=pred,P=b['P']).items():saved[f'block{i}_{key}']=value
    score=dict(**metrics(errors),blocks=blocks_info,initializations=len(blocks_info),
               initial_history='xi=q at each valid block start; no fitted validation state')
    if output:
        path=Path(output);path.mkdir(exist_ok=True,parents=True)
        np.savez_compressed(path/f'{label}.npz',**saved)
    return score,saved


def plot_comparison(nominal,residual,dest,title):
    fig,axes=plt.subplots(2,1,figsize=(11,6),sharex=True)
    i=0
    while f'block{i}_t' in nominal:
        t=nominal[f'block{i}_t'];q=nominal[f'block{i}_q']
        axes[0].plot(t,np.rad2deg(q),'k',lw=1,label='Measured' if i==0 else None)
        axes[0].plot(t,np.rad2deg(nominal[f'block{i}_prediction'][:,0]),color='tab:orange',lw=1,label='Fixed nominal' if i==0 else None)
        axes[0].plot(t,np.rad2deg(residual[f'block{i}_prediction'][:,0]),color='tab:blue',lw=1,label='Nominal + passive residual' if i==0 else None)
        axes[1].plot(t,np.rad2deg(nominal[f'block{i}_prediction'][:,0]-q),color='tab:orange',lw=1)
        axes[1].plot(t,np.rad2deg(residual[f'block{i}_prediction'][:,0]-q),color='tab:blue',lw=1)
        i+=1
    axes[0].set(ylabel='Angle [deg]',title=title);axes[0].legend()
    axes[1].set(ylabel='Prediction - measured [deg]',xlabel='Run time [s]')
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(dest,dpi=170);plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data-root',type=Path,default=Path('/home/risebrl/result/ph/4'))
    ap.add_argument('--nominal',type=Path,default=Path('reports/nominal_actuator4_20261006/elastic_rollout.yaml'))
    ap.add_argument('--output',type=Path,default=Path('reports/residual_actuator4_20261007'))
    ap.add_argument('--steps',type=int,default=1800)
    ap.add_argument('--resume',action='store_true',help='Reuse saved candidates and completed equation checkpoints')
    args=ap.parse_args();out=args.output;out.mkdir(exist_ok=True,parents=True)
    torch.set_num_threads(1);torch.set_default_dtype(torch.float64)
    p=Parameters.load(args.nominal)
    runs=[]
    # Deliberately do not load historical validation until candidate selection ends.
    for name,role in RUNS:
        if role not in ('train','development'):continue
        r=load_run(args.data_root/name);r['role']=role;runs.append(r)
    train=[r for r in runs if r['role']=='train'];dev=next(r for r in runs if r['role']=='development')
    blks=[b for r in train for b in blocks(r)];data=training_arrays(blks,p)
    bounds=(float(data['q'].min()),float(data['q'].max()));scale=tuple(np.max(np.abs(data['P']),axis=0))
    nominal=PassiveResidual(p,bounds,scale)
    nom_dev,_=evaluate(dev,nominal)
    print('Fixed nominal development',nom_dev['rmse_deg'],flush=True)
    candidates=json.loads((out/'candidates.json').read_text()) if args.resume and (out/'candidates.json').exists() else []
    objects={c['name']:json.loads((out/f"{c['name']}.json").read_text()) for c in candidates if 'development' in c};trace={}
    def candidate(model,name):
        obj=copy.deepcopy(model.export());dump(out/f'{name}.json',obj)
        try:score,_=evaluate(dev,model)
        except (ValueError,RuntimeError) as e:
            candidates.append(dict(name=name,status='integration_failed',reason=str(e)));return
        candidates.append(dict(name=name,development=score))
        objects[name]=obj;dump(out/'candidates.json',candidates)
        print('Candidate',name,'development RMSE',score['rmse_deg'],flush=True)
    for seed in (0,1):
        if args.resume and (out/f'pd_seed{seed}_step{args.steps}.json').exists():
            for step in range(600,args.steps+1,600):
                name=f'pd_seed{seed}_step{step}'
                if name not in objects:
                    candidate(PassiveResidual.restore(json.loads((out/f'{name}.json').read_text())),name)
            continue
        torch.manual_seed(seed);m=PassiveResidual(p,bounds,scale)
        trace[f'pd_seed{seed}']=train_steps(m,data,np.zeros(len(data['q'])),args.steps,seed,
                                          lambda model,step:candidate(model,f'pd_seed{seed}_step{step}'))
    valid=[c for c in candidates if 'development' in c]
    best_pd=min(valid,key=lambda c:c['development']['rmse_deg'])['name']
    m=PassiveResidual.restore(objects[best_pd])
    history_fits=[]
    for round_id in (1,2):
        info,hist=fit_history(m,blks,data);history_fits.append(info)
        print('History',round_id,info,flush=True)
        candidate(m,f'history_round{round_id}_before_nn')
        trace[f'history_round{round_id}']=train_steps(m,data,hist,600,100+round_id)
        candidate(m,f'history_round{round_id}_after_nn')
    # Freeze hidden features, then train only NN output layers on short rollouts.
    # This is necessary because smaller equation error is not necessarily smaller angle RMSE.
    from .refine_nominal import windows
    from .residual_rollout import refine
    refinement=[]
    history_names=[c for c in candidates if 'development' in c and c['name'].startswith('history_')]
    best_history=min(history_names,key=lambda c:c['development']['rmse_deg'])['name']
    for source in (best_pd,best_history):
        model=PassiveResidual.restore(objects[source]);wins=windows(train)
        nn=NumpyResidual(model.export())
        for w in wins:
            match=next((b for b in blks if b['run']==w['run'] and b['t'][0]<=w['t'][0]<=b['t'][-1]),None)
            if match is not None and np.any(nn.k>0):
                f=observed_history([match],nn.k,nn.rv,nn.rho,components=True)
                f0=np.array([np.interp(w['t'][0],match['t'],f[:,j]) for j in range(2)])
                q0=np.interp(w['t'][0],match['t'],match['qs'])
                w['xi0']=q0-np.divide(f0,nn.k,out=np.zeros(2),where=nn.k>0)
        model,info=refine(model,wins)
        refinement.append(dict(source=source,**info));candidate(model,'rollout_'+source)
        dump(out/'rollout_refinement.json',refinement)
    valid=[c for c in candidates if 'development' in c]
    best=min(valid,key=lambda c:c['development']['rmse_deg'])
    selected=PassiveResidual.restore(objects[best['name']])
    dump(out/'selected_model.json',selected.export())
    dump(out/'training_trace.json',trace);dump(out/'history_fits.json',history_fits)
    dump(out/'selection.json',dict(selected=best['name'],nominal_development=nom_dev,
                                  residual_development=best['development'],selection_rule='minimum full development seed 3 free-rollout RMSE',
                                  historical_validation_used_for_selection=False))
    valname=next(n for n,r in RUNS if r=='validation_prefix')
    val=load_run(args.data_root/valname,cutoff=356.45);val['role']='historical_validation_prefix';runs.append(val)
    scores={}
    for r in runs:
        name=Path(r['path']).name;print('Final paired evaluation',name,flush=True)
        ns,na=evaluate(r,nominal,out/'artifacts','nominal_'+name)
        rs,ra=evaluate(r,selected,out/'artifacts','residual_'+name)
        scores[name]=dict(role=r['role'],nominal=ns,residual=rs,
                         rmse_reduction_percent=100*(1-rs['rmse_deg']/ns['rmse_deg']))
        dump(out/'metrics.json',scores)
        plot_comparison(na,ra,out/f'{name}.png',f"{r['meta']['profile']['id']} seed {r['meta']['profile']['seed']} | {r['role']}")
    dump(out/'provenance.json',dict(nominal_file=str(args.nominal),nominal_sha256=sha256(args.nominal),
        paper_sha256=sha256('main.tex'),torch=torch.__version__,dtype='float64',
        q_bounds=bounds,pressure_scale=scale,training_rows=len(data['q']),
        files=[dict(path=r['path'],role=r['role'],**r['audit']) for r in runs],
        code_sha256={str(f):sha256(f) for f in Path('ph_model').glob('*.py')},
        learned_area_residual=False,multi_actuator_embedding=False))
    print('RESIDUAL FIT COMPLETE',best['name'],flush=True)


if __name__=='__main__':main()
