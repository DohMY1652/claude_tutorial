"""V7: learned chamber feature geometry followed by passive pressure-dependent R.

No ROS, no measured-angle injection during rollout, no changes to V6 artifacts.
"""
import argparse
import copy
import json
import time
from pathlib import Path
import numpy as np
from .structured_model import Model,simulate
from .refit_structured import prepare,parameterization,scores
from .diagnose_structure import load_blocks
from .refit_motion import optimize
from .fit_nominal import dump
from .data import sha256

BASE=Path('reports/residual_actuator4_20261007/structured_refit_v6')
OUT=BASE.parent/'adaptive_slots_v7'


def param(template):
    initial,base_decode=parameterization(template)
    p=template['nominal'];f=template['feature_reference'];r=p['reel_radius_m']
    scales=np.asarray(f.get('area_scales_m',[f['area_scale_m']]*2))/r
    extra=np.r_[(p['x1_zero_m']-f['x1_center_m'])/r,(f['x2_center_m']-p['x2_zero_m'])/r,scales,
                np.asarray(template.get('history_rate_coefficients',np.zeros((2,2)))).ravel()]
    lo=np.array([.15,.15,.08,.08,-1.5,-1.5,-1.5,-1.5]);hi=np.array([1.2,1.2,.8,.8,1.5,1.5,1.5,1.5])
    def decode(z):
        obj=base_decode(z[:24]);a=lo+z[24:]*(hi-lo)
        obj['schema']='structured_ph_v7'
        obj['feature_reference'].update(x1_center_m=float(p['x1_zero_m']-r*a[0]),
            x2_center_m=float(p['x2_zero_m']+r*a[1]),area_scales_m=(r*a[2:4]).tolist())
        obj['history_rate_coefficients']=a[4:].reshape(2,2).tolist()
        return obj
    return np.r_[initial,(extra-lo)/(hi-lo)],decode


def add_long_windows(bb):
    runs=set(b['run'] for b in bb)
    for b in bb:
        q=np.rad2deg(b['q']);b['long']={}
        for lag,limit in [(300,.5),(1200,.75)]:
            b['long'][lag]=np.array([i for i in range(0,max(0,len(q)-lag),100) if np.ptp(q[i:i+lag+1])<limit],dtype=int)
    for run in runs:
        group=[b for b in bb if b['run']==run]
        counts={lag:sum(len(b['long'][lag]) for b in group) for lag in (300,1200)}
        for b in group:b['long_weight']={lag:.5/np.sqrt(max(1,n)*len(runs)) for lag,n in counts.items()}
    return bb


def objective(bb,decode,label,baseline_metrics):
    calls=0;tic=time.time();qgrid=np.deg2rad(np.linspace(5,80,61))
    baseline={r['run']:r['rmse_deg'] for r in baseline_metrics}
    def fun(z):
        nonlocal calls
        obj=decode(z);parts=[];squares={};counts={}
        for b in bb:
            try:e=np.rad2deg(simulate(b,obj)[:,0]-b['q'])
            except ValueError:e=np.full(len(b['q']),1e4)
            parts.extend([e[b['angle_idx']]*b['angle_weight'],(e[b['win']+100]-e[b['win']])*b['win_weight']])
            for lag,idx in b['long'].items():parts.append((e[idx+lag]-e[idx])*b['long_weight'][lag])
            run=b['run'];squares[run]=squares.get(run,0)+float(e@e);counts[run]=counts.get(run,0)+len(e)
        # Training-only profile regression penalty, fixed before candidate runs.
        parts.append(np.array([2*max(0,np.sqrt(squares[run]/counts[run])-max(1.08*baseline[run],baseline[run]+.15))
                               /np.sqrt(len(squares)) for run in squares]))
        m=Model(obj);g=np.array([m.potential_gradient(q) for q in qgrid]);k=np.gradient(g,qgrid)
        total=m.p.gravity_nm*np.cos(qgrid)+m.p.elastic_k_nm_rad+k
        parts.extend([.1*m.area.ravel(),.01*np.diff(k),np.minimum(total,0)*.3,.03*m.rate_coeff.ravel()])
        result=np.concatenate(parts);calls+=1
        if calls%40==0:print(label,'calls',calls,'loss',round(float(np.linalg.norm(result)),5),'seconds',round(time.time()-tic),flush=True)
        return result
    return fun


def fit(train,dev,base_metrics,template,stage,start,nfev,polish=False):
    initial,decode_all=param(template)
    if stage=='curve' and start==1:
        # Alternate length transitions, never measured trajectory corrections.
        initial[24:26]=[(.5-.15)/1.05,(.8-.15)/1.05]
    active={'curve':np.r_[np.arange(5,12),np.arange(24,28)],
            'history':np.r_[[0,1,4],np.arange(18,24),np.arange(28,32)],
            'joint':np.arange(32)}[stage]
    def decode(z):
        full=initial.copy();full[active]=z;return decode_all(full)
    name=f'{stage}_start{start}';fn=objective(train,decode,name,base_metrics)
    if polish:
        from scipy.optimize import least_squares
        def jac(z):
            f=fn(z);cols=[]
            for j in range(len(z)):
                h=1e-5 if z[j]+1e-5<1 else -1e-5
                shifted=z.copy();shifted[j]+=h;cols.append((fn(shifted)-f)/h)
            return np.column_stack(cols)
        fit=least_squares(fn,np.clip(initial[active],1e-8,1-1e-8),jac=jac,bounds=(0,1),
            x_scale=1.,max_nfev=nfev,ftol=1e-5,xtol=1e-6,gtol=1e-6)
    else:fit=optimize(fn,initial[active],np.zeros(len(active)),np.ones(len(active)),nfev)
    obj=decode(fit.x);dump(OUT/(name+'.json'),obj)
    result=dict(name=name,loss=float(np.linalg.norm(fit.fun)),nfev=fit.nfev,success=bool(fit.success),
        optimizer_message=fit.message,optimality=float(fit.optimality),polish=polish,
        training=scores(train,obj),development=scores(dev,obj))
    dump(OUT/(name+'_metrics.json'),result)
    print('FINISHED',name,'development',result['development'],flush=True)
    return result


def seed(stage,baseline):
    names=['baseline'] if stage=='history' else ['baseline']
    names += ['curve_start0','curve_start1']
    if stage=='joint':names+=['history_start0']
    candidates=[]
    for name in names:
        path=OUT/(name+'_metrics.json')
        if path.exists():candidates.append(json.loads(path.read_text()))
    best=min(candidates,key=lambda c:c['loss'])
    dump(OUT/(stage+'_initialization.json'),dict(source=best['name'],criterion='training loss only; intermediate seed is not accepted final model'))
    return json.loads((OUT/(best['name']+'.json')).read_text())


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=['curve','history','joint'],required=True)
    ap.add_argument('--start',type=int,default=0);ap.add_argument('--nfev',type=int,default=60)
    ap.add_argument('--polish',action='store_true');args=ap.parse_args()
    if args.polish and (args.stage!='joint' or args.start!=1):ap.error('Polish uses joint --start 1')
    OUT.mkdir(parents=True,exist_ok=True)
    bb=load_blocks();train=add_long_windows(prepare([b for b in bb if b['role']=='train']))
    dev=add_long_windows(prepare([b for b in bb if b['role']=='development']))
    baseline=json.loads((BASE/'selected_model.json').read_text())
    if not (OUT/'baseline_metrics.json').exists():
        dump(OUT/'protocol.json',dict(baseline_sha256=sha256(BASE/'selected_model.json'),paper_sha256=sha256('main.tex'),
            sequence='Area head+transition shape only, pressure-dependent passive history, joint refinement.',
            loss='V6 loss + 30/120s measured stationary-window increment error (weight0.5) + training-only per-run regression penalty + pressure-rate coefficient L2.',
            long_window='30s range<0.5deg; 120s range<0.75deg; starts every10s within each valid block.',
            physical='Energy pressure-independent; R multiplied by exp(w*tanh(abs(P)/50kPa)); positive area maintained; unchanged nominal anchors and state reset policy.',
            shape_bounds='Centers equivalent q=.15..1.2rad, widths .08..0.8rad, converted to chamber lengths. These are basis parameters, not physical measured dimensions.',
            rate_coeff_bounds=[-1.5,1.5],
            selection='Train/development each RMSE <=max(1.08*V6,V6+.15deg); mean run RMSE improves; dev RMSE improves by >=.01deg; seed0..2 slow misses each <=V6+.03 and mean does not increase; dev hold-increment <=max(1.25*V6,V6+.05deg). Development score RMSE+0.4*slow increment RMSE+0.2*mean run RMSE.',
            historical='Previously seen seed4 only evaluated after selection; not blind. No hardware, push or data calibration changes.'))
        metrics=scores(train,baseline);fn=objective(train,lambda z:baseline,'baseline',metrics)
        dump(OUT/'baseline.json',baseline);dump(OUT/'baseline_metrics.json',dict(name='baseline',loss=float(np.linalg.norm(fn(np.zeros(1)))),
            training=metrics,development=scores(dev,baseline)))
    base_metrics=json.loads((OUT/'baseline_metrics.json').read_text())['training']
    template=baseline if args.stage=='curve' else seed(args.stage,baseline)
    if args.polish:
        template=json.loads((OUT/'joint_start0.json').read_text())
        dump(OUT/'polish_protocol.json',dict(source='joint_start0',
            reason='Joint initial optimizer stopped after 3 function evaluations; check numerical optimization sensitivity, no model/evaluation changes.',
            finite_difference_normalized_step=1e-5,x_scale=1.,ftol=1e-5,xtol=1e-6,
            selection='Same predeclared V6 gates; no historical data used.'))
    fit(train,dev,base_metrics,template,args.stage,args.start,args.nfev,args.polish)


if __name__=='__main__':main()
