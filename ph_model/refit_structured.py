"""Staged offline pH fitting, measured chamber inputs, untouched V4 reference."""
import argparse
import copy
import json
from pathlib import Path
import time
import numpy as np
from .structured_model import Model,zero_model,simulate
from .diagnose_structure import load_blocks,windows,write_csv
from .fit_nominal import dump
from .refit_motion import optimize
from .data import sha256

OUT=Path('reports/residual_actuator4_20261007/structured_refit_v6')


def prepare(bb):
    runs=list(dict.fromkeys(b['run'] for b in bb))
    for b in bb:
        q=np.rad2deg(b['q']);b['angle_idx']=np.arange(0,len(q),5)
        b['bins']=np.floor(q[b['angle_idx']]/10).astype(int)
        b['win'],b['kind']=windows(q)
    for run in runs:
        group=[b for b in bb if b['run']==run];bins=np.concatenate([b['bins'] for b in group]);keys,counts=np.unique(bins,return_counts=True)
        count=dict(zip(keys,counts));kinds={kind:sum(np.sum(b['kind']==kind) for b in group) for kind in ('hold','slow','large')}
        for b in group:
            b['angle_weight']=np.array([1/np.sqrt(count[k]*len(keys)*len(runs)) for k in b['bins']])
            b['win_weight']=np.array([dict(hold=.5,slow=2.,large=1.)[k]/np.sqrt(kinds[k]*len(runs)) for k in b['kind']])
    return bb


def scores(bb,obj,substeps=5):
    rows=[]
    for run in dict.fromkeys(b['run'] for b in bb):
        group=[b for b in bb if b['run']==run];errors=[];changes={k:[] for k in ('hold','slow','large')};miss=[]
        for b in group:
            pred=np.rad2deg(simulate(b,obj,substeps=substeps)[:,0]);actual=np.rad2deg(b['q']);e=pred-actual;errors.extend(e)
            idx,kind=windows(actual)
            for i,k in zip(idx,kind):
                changes[k].append(e[i+100]-e[i])
                if k=='slow':miss.append(np.ptp(pred[i:i+101])<.2*np.ptp(actual[i:i+101]))
        row=dict(run=run,role=group[0]['role'],rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),max_abs_deg=float(np.max(np.abs(errors))))
        for k,v in changes.items():row[k+'_increment_rmse_deg']=float(np.sqrt(np.mean(np.square(v)))) if v else None
        row['slow_missed_fraction']=float(np.mean(miss)) if miss else None;rows.append(row)
    return rows


def parameterization(template,area_range='base'):
    p=template['nominal'];area=np.asarray(template['area_coefficients']).ravel();area_idx=[0,1,2,3,5,6,7]
    x=np.r_[p['alpha'],np.log(p['damping_nm_s_rad']),p['elastic_k_nm_rad'],p['elastic_bias_nm'],np.log(p['epsilon_rad_s']),
        area[area_idx],template['potential_coefficients'],template['mu_coefficients'],
        np.log(np.maximum(template['history_k'],1e-5)),np.log(template['history_tau_s']),np.log(template['history_rho'])]
    lo=np.r_[0,np.log(.001),.001,-3,np.log(1e-5),[-.20]*4,[-.25]*3,[-.20]*4,[0]*2,[np.log(1e-5)]*2,[np.log(1.)]*2,[np.log(.01)]*2]
    hi=np.r_[1,np.log(20),8,3,np.log(.03),[.20]*4,[.25]*3,[.20]*4,[1.5]*2,[np.log(10.)]*2,[np.log(600.)]*2,[np.log(100.)]*2]
    if area_range=='wider':
        # Separate sensitivity experiment: each chamber's correction L1 bound
        # is <=0.9, so areas remain >=10% of nominal at every feature input.
        hi[5:12]=[.15,.30,.30,.15,.30,.30,.30];lo[5:12]=-hi[5:12]
    def decode(z):
        obj=copy.deepcopy(template);a=lo+z*(hi-lo)
        obj['nominal'].update(alpha=float(a[0]),damping_nm_s_rad=float(np.exp(a[1])),elastic_k_nm_rad=float(a[2]),
            elastic_bias_nm=float(a[3]),epsilon_rad_s=float(np.exp(a[4])))
        arr=np.zeros(8);arr[area_idx]=a[5:12];obj['area_coefficients']=arr.reshape(2,4).tolist()
        obj['potential_coefficients']=a[12:16].tolist();obj['mu_coefficients']=a[16:18].tolist()
        obj['history_k']=np.exp(a[18:20]).tolist();obj['history_tau_s']=np.exp(a[20:22]).tolist();obj['history_rho']=np.exp(a[22:24]).tolist()
        return obj
    return (x-lo)/(hi-lo),decode


def objective(bb,decode,label):
    calls=0;tic=time.time()
    qgrid=np.deg2rad(np.linspace(5,80,61))
    def fun(z):
        nonlocal calls
        obj=decode(z);parts=[]
        for b in bb:
            try:e=np.rad2deg(simulate(b,obj)[:,0]-b['q'])
            except ValueError:e=np.full(len(b['q']),1e4)
            parts.extend([e[b['angle_idx']]*b['angle_weight'],(e[b['win']+100]-e[b['win']])*b['win_weight']])
        m=Model(obj);gg=np.array([m.potential_gradient(q) for q in qgrid]);k=np.gradient(gg,qgrid)
        total=m.p.gravity_nm*np.cos(qgrid)+m.p.elastic_k_nm_rad+k
        # Smoothness and locally nonnegative restoring stiffness are explicit
        # fitting priors, NOT requirements of the passivity theorem.
        parts.extend([.1*m.area.ravel(),.01*np.diff(k),np.minimum(total,0)*.3])
        result=np.concatenate(parts);calls+=1
        if calls%40==0:print(label,'calls',calls,'loss',round(float(np.linalg.norm(result)),5),'seconds',round(time.time()-tic),flush=True)
        return result
    return fun


def fit_stage(train,dev,template,stage,start=0,nfev=65,area_range='base'):
    initial,decode_all=parameterization(template,area_range)
    active={'nominal':np.arange(5),'area':np.arange(12),'full':np.arange(24)}[stage]
    if stage=='full' and area_range=='base':
        warm=copy.deepcopy(template);warm['history_k']=[.3,2.] if start==0 else [1.,5.]
        warm['history_tau_s']=[5.,80.] if start==0 else [20.,200.]
        warm['history_rho']=[2.,10.];initial,decode_all=parameterization(warm)
    def decode(z):
        vector=initial.copy();vector[active]=z;obj=decode_all(vector)
        if stage!='full':obj['history_k']=[0.,0.]
        return obj
    name=f'{stage}_'+('wider_' if area_range=='wider' else '')+f'start{start}';fun=objective(train,decode,name)
    fit=optimize(fun,initial[active],np.zeros(len(active)),np.ones(len(active)),nfev)
    obj=decode(fit.x);dump(OUT/(name+'.json'),obj)
    result=dict(name=name,nfev=fit.nfev,success=bool(fit.success),loss=float(np.linalg.norm(fit.fun)),
        training=scores(train,obj),development=scores(dev,obj))
    dump(OUT/(name+'_metrics.json'),result);print('FINISHED',name,'development',result['development'],flush=True)
    return obj,result


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--stage',choices=['nominal','area','full'],required=True)
    ap.add_argument('--start',type=int,default=0);ap.add_argument('--nfev',type=int,default=65)
    ap.add_argument('--area-range',choices=['base','wider'],default='base');args=ap.parse_args()
    if args.area_range=='wider' and args.stage!='full':ap.error('Wider sensitivity requires full stage')
    OUT.mkdir(parents=True,exist_ok=True);bb=load_blocks()
    train=prepare([b for b in bb if b['role']=='train']);dev=prepare([b for b in bb if b['role']=='development'])
    if args.stage=='nominal':
        template=zero_model();dump(OUT/'zero_initialized.json',template)
        dump(OUT/'protocol.json',dict(paper_sha256=sha256('main.tex'),
            objective='Equal runs and 10-degree bins; measured-only 10s regimes: hold weight0.5, slow2, large1. No online angle feedback.',
            fixed='Mg=2.943Nm, J=.045kgm2, r=.025m, D=.05m, x10=.05m: conditional anchors, not verified measurements.',
            features='Fixed smooth tanh chamber-length neurons, zero output heads. Single actuator only. Pressure-dependent area and nonnegative friction slots.',
            selection='Compare every train run + development to V4; each RMSE <= max(1.10*old,old+.25deg). Require aggregate improvement and smaller seed0..2 slow-motion miss fraction. Historical seed4 only after selection.',
            constraints='Area globally positive; nonnegative dissipation/history; bounded pressure-independent energy; matching power port; positive time constants. No hardware.',
            history_tau_bounds_s=[1,600],history_note='Exploration range, not measured timescales; boundary solutions flagged. xi initially q0, identical reference policy.'))
    else:
        prior='nominal_start0.json' if args.stage=='area' else 'area_start0.json'
        if args.area_range=='wider':
            prior='full_start0.json'
            dump(OUT/'area_range_sensitivity_protocol.json',dict(
                motivation='Two vacuum length-head coefficients reached base bounds. Test smooth area head capacity without removing positive-area constraint.',
                minus_head_bounds=[.15,.30,.30,.15],plus_head_bounds=[0.,.30,.30,.30],
                absolute_sum_bound=.90,initial='full_start0',
                selection='Same V4 regression and slow-motion gates; record as extra non-blind development exploration.'))
        template=json.loads((OUT/prior).read_text())
    fit_stage(train,dev,template,args.stage,args.start,args.nfev,args.area_range)


if __name__=='__main__':main()
