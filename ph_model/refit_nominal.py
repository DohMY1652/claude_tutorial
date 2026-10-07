"""Nominal central-branch fit, free gravity, conditional reel-radius analysis.

All fits use training profiles only. Radius profiles are not physical dimension
measurements. Force-normalized central losses avoid favoring a smaller radius
merely because it scales all torques down. No hardware imports or operations.
"""
import copy
import dataclasses
import json
from pathlib import Path
import time

import numpy as np
import torch
from scipy.optimize import least_squares, lsq_linear

from .data import load_run, stationary_points, sha256
from .nominal import Parameters, torque, gradient
from .fit_nominal import RUNS, dump, metrics
from .fit_residual import blocks, evaluate
from .residual import PassiveResidual
from .residual_fast import simulate

BASE=Path('reports/residual_actuator4_20261007')
OUT=BASE/'nominal_refit_v3'


def pair_branches(points,profile,count=15):
    centers=sorted(set(round(pt['center'],6) for pt in points)) if profile=='S2' else [None]
    groups=[]
    for center in centers:
        sides=[]
        for direction in (1,-1):
            pts=[pt for pt in points if pt['direction']==direction and
                 (center is None or abs(pt['center']-center)<1e-6)]
            if len(pts)<2:break
            # Average duplicate angle coordinates; never extrapolate either branch.
            q=np.array(sorted(set(round(pt['q'],10) for pt in pts)))
            P=np.array([np.mean([pt['pressure'] for pt in pts if round(pt['q'],10)==qi],axis=0) for qi in q])
            if len(q)<2:break
            sides.append((q,P))
        if len(sides)!=2:continue
        lo=max(s[0][0] for s in sides);hi=min(s[0][-1] for s in sides)
        if hi<=lo:continue
        q=np.linspace(lo,hi,count)
        up,down=[np.column_stack([np.interp(q,x,P[:,j]) for j in (0,1)]) for x,P in sides]
        groups.append(dict(profile=profile,center=center,q=q,up=up,down=down))
    return groups


def central_residual(p,groups):
    return np.concatenate([(gradient(g['q'],p)-torque(g['q'],(g['up']+g['down'])/2,p))/p.reel_radius_m/
                           np.sqrt(len(g['q'])*len(groups)) for g in groups])


def central_fit(base,groups):
    names=['gravity_nm','elastic_k_nm_rad','elastic_bias_nm','x1_zero_m']
    lo=np.array([.2,0.,-3.,.05]);hi=np.array([8.,10.,3.,.15])
    def decode(z):return dataclasses.replace(base,**dict(zip(names,z.tolist())))
    def fun(z):return central_residual(decode(z),groups)
    fits=[];best=None
    for start in ([2.943,1.7,-1.49,.06],[4.,0.,-.5,.10],[1.,3.,-1.,.14]):
        fit=least_squares(fun,np.clip(start,lo+1e-7,hi-1e-7),bounds=(lo,hi),
                          x_scale='jac',max_nfev=200,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        fits.append(dict(cost=float(fit.cost),nfev=fit.nfev,success=bool(fit.success)))
        if best is None or fit.cost<best.cost:best=fit
    p=decode(best.x)
    # Approximate branch halfwidth only initializes alpha; never replace tanh by direction.
    mid=np.concatenate([torque(g['q'],(g['up']+g['down'])/2,p) for g in groups])
    half=np.concatenate([(torque(g['q'],g['up'],p)-torque(g['q'],g['down'],p))/2 for g in groups])
    alpha=float(np.clip(np.dot(mid,half)/np.dot(mid,mid),0,2))
    p=dataclasses.replace(p,alpha=alpha)
    singular=np.linalg.svd(best.jac*(hi-lo)[None,:],compute_uv=False)
    return p,dict(force_rmse_n=float(np.linalg.norm(best.fun)),parameters=dataclasses.asdict(p),
        bounds=dict(zip(names,np.column_stack((lo,hi)).tolist())),attempts=fits,
        boundary=[name for name,z,a,b in zip(names,best.x,lo,hi) if min(z-a,b-z)<1e-4*(b-a)],
        scaled_jacobian_singular_values=singular.tolist(),approximate_branch_alpha=alpha)


def zero_model(p):
    return PassiveResidual(p,(.1,1.3),(65000.,75000.)).export()


def gravity_profile(base,groups):
    """Conditional central-curve sensitivity, not a confidence interval."""
    q=np.concatenate([g['q'] for g in groups])
    P=np.concatenate([(g['up']+g['down'])/2 for g in groups])
    w=np.concatenate([np.full(len(g['q']),1/np.sqrt(len(g['q'])*len(groups))) for g in groups])
    A=np.column_stack((q,np.ones(len(q))))*w[:,None]/base.reel_radius_m
    rows=[]
    for Mg in (.5,1.,2.,2.943,4.,6.,8.):
        target=(torque(q,P,base)-Mg*np.sin(q))*w/base.reel_radius_m
        fit=lsq_linear(A,target,bounds=([0,-3],[10,3]))
        rows.append(dict(gravity_nm=Mg,elastic_k_nm_rad=float(fit.x[0]),elastic_bias_nm=float(fit.x[1]),
                         central_force_rmse_n=float(np.linalg.norm(A@fit.x-target))))
    return rows


def main():
    OUT.mkdir(exist_ok=True,parents=True);torch.set_num_threads(1)
    old=Parameters.load('reports/nominal_actuator4_20261006/elastic_rollout.yaml')
    runs=[];groups=[];train=[]
    for name,role in RUNS:
        if role not in ('train','development'):continue
        r=load_run(Path('/home/risebrl/result/ph/4')/name);r['role']=role;runs.append(r)
        if role=='train':
            train.extend(blocks(r))
            profile=r['meta']['profile']['id']
            if profile in ('S2','S1a','S1b'):groups.extend(pair_branches(stationary_points(r),profile))
    dev=next(r for r in runs if r['role']=='development')
    dump(OUT/'protocol.json',dict(paper_sha256=sha256('main.tex'),radii_mm=[20,22.5,25,27.5,30],
        gravity_bounds_nm=[.2,8],mass_and_arm_not_separately_identified=True,inertia_fixed=old.inertia_kg_m2,
        static_objective='equal group force residual (torque/r); paired at same angle, no extrapolation',
        dynamic_objective='equal run full-rollout angle RMSE + 0.25 central-force/100N anchor',
        radius_policy='conditional sensitivity only; fixed measured-setting 25mm model selected for residual stage',
        selection='development seed3 adaptive RMSE among old nominal and fixed25 new candidates',
        historical_seed4='already diagnosed, not blind; not loaded until after residual selection',
        files=[dict(path=r['path'],role=r['role'],**r['audit']) for r in runs]))
    dump(OUT/'paired_branches.json',[{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in g.items()} for g in groups])
    counts={b['run']:sum(len(bb['q'][::5]) for bb in train if bb['run']==b['run']) for b in train}
    sizes=sum(len(b['q'][::5]) for b in train)
    weights=[1/np.sqrt(counts[b['run']]*len(counts)) for b in train]
    def score(p,blks):return metrics(np.concatenate([np.rad2deg(simulate(b,zero_model(p))[:,0]-b['q']) for b in blks]))
    def dynamic_fit(base,joint):
        names=['alpha','damping_nm_s_rad']+(['gravity_nm','elastic_k_nm_rad','elastic_bias_nm'] if joint else [])
        lo=np.array([0.,0.]+([.2,0.,-3.] if joint else []));hi=np.array([2.,50.]+([8.,10.,3.] if joint else []))
        def decode(z):return dataclasses.replace(base,**dict(zip(names,z.tolist())))
        calls=0;tic=time.time()
        def fun(z):
            nonlocal calls
            p=decode(z);o=zero_model(p)
            try:e=np.concatenate([(simulate(b,o)[::5,0]-b['q'][::5])*w for b,w in zip(train,weights)])
            except ValueError:e=np.full(sizes,10/np.sqrt(sizes))
            calls+=1
            if calls%25==0:print('NOMINAL',base.reel_radius_m,joint,calls,'train',np.rad2deg(np.linalg.norm(e)),'seconds',round(time.time()-tic),flush=True)
            return np.r_[e,.25*central_residual(p,groups)/100] if joint else e
        def jac(z):
            f=fun(z);ans=[]
            for i in range(len(z)):
                h=1e-4 if z[i]+1e-4<=hi[i] else -1e-4
                zz=z.copy();zz[i]+=h;ans.append((fun(zz)-f)/h)
            return np.column_stack(ans)
        result=least_squares(fun,np.clip([getattr(base,n) for n in names],lo+1e-7,hi-1e-7),
            jac=jac,bounds=(lo,hi),x_scale='jac',max_nfev=30,ftol=2e-4,xtol=2e-4,gtol=1e-6)
        p=decode(result.x)
        return p,dict(parameters=dataclasses.asdict(p),training_equal_run_rmse_deg=float(np.rad2deg(np.linalg.norm(result.fun[:sizes]))),
            central_force_rmse_n=float(np.linalg.norm(central_residual(p,groups))),success=bool(result.success),nfev=result.nfev,
            fitted_names=names,bounds=dict(zip(names,np.column_stack((lo,hi)).tolist())),
            boundary=[n for n,z,a,b in zip(names,result.x,lo,hi) if min(z-a,b-z)<1e-4*(b-a)])
    candidates={'old_nominal':dict(parameters=dataclasses.asdict(old),development_fast=score(old,blocks(dev)))}
    radius=[]
    # Center-first and full-trajectory fits for each radius; do not compare raw Nm cost across radii.
    for mm in (25.,20.,22.5,27.5,30.):
        p,info=central_fit(dataclasses.replace(old,reel_radius_m=mm/1000),groups)
        dump(OUT/f'central_r{mm:g}.json',info)
        if mm==25:
            dump(OUT/'gravity_profile.json',gravity_profile(p,groups))
            fixed,fixedinfo=dynamic_fit(p,False)
            fixedinfo['development_fast']=score(fixed,blocks(dev));candidates['fixed25_central_then_friction']=fixedinfo
            dump(OUT/'nominal_candidates.json',candidates)
        joint,jinfo=dynamic_fit(p,True)
        jinfo['development_fast']=score(joint,blocks(dev))
        radius.append(dict(radius_mm=mm,central=info,dynamic=jinfo))
        dump(OUT/'radius_profile.json',radius)
        print('RADIUS',mm,'Mg',joint.gravity_nm,'dev',jinfo['development_fast']['rmse_deg'],flush=True)
        if mm==25:candidates['fixed25_joint_gravity']=jinfo
    for name,c in candidates.items():
        p=Parameters(**c['parameters']);c['development_adaptive']=evaluate(dev,PassiveResidual.restore(zero_model(p)))[0]
    best=min(candidates,key=lambda name:candidates[name]['development_adaptive']['rmse_deg'])
    dump(OUT/'nominal_candidates.json',candidates)
    dump(OUT/'selected_nominal.json',candidates[best]['parameters'])
    dump(OUT/'nominal_selection.json',dict(name=best,**candidates[best]))
    print('NOMINAL REFIT COMPLETE',best,candidates[best]['development_adaptive']['rmse_deg'],flush=True)


if __name__=='__main__':main()
