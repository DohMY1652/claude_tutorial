"""Paired offline exclusion sensitivity; never labels difficult data as corrupt.

Full pressure histories/state rollouts are retained. Only selected angle loss
terms are masked. V7 warm starts make this a sensitivity study, NOT blind CV.
"""
import argparse
import csv
import json
import time
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from .refit_adaptive_slots import param, add_long_windows
from .refit_structured import prepare
from .structured_model import Model, simulate
from .diagnose_structure import load_blocks, write_csv
from .fit_nominal import dump
from .data import sha256

BASE=Path('reports/residual_actuator4_20261007/adaptive_slots_v7')
OUT=BASE.parent/'exclusion_study_v8'
# Frozen before fitting, based on the V7 report/graphs, not reselected by results.
INTERVALS={
    '20261006_141418_972450_S6':[(80.,280.)],
    '20261006_142450_704718_S6':[(520.,650.)],
    '20261006_143524_612945_S6':[(60.,160.),(320.,540.)],
}


def exclusion_mask(b):
    mask=np.zeros(len(b['t']),dtype=bool)
    for lo,hi in INTERVALS.get(b['run'],[]):mask|=(b['t']>=lo)&(b['t']<=hi)
    return mask


def valid_windows(keep,idx,lag):
    prefix=np.r_[0,np.cumsum(~keep)]
    return prefix[idx+lag+1]-prefix[idx]==0


def objective(bb,decode,variant):
    tic=time.time();calls=0;failures=0
    template=json.loads((BASE/'selected_model.json').read_text())
    # Keep original normalization for retained terms: only masking differs.
    baseline={};counts={}
    for b in bb:
        keep=~exclusion_mask(b) if variant=='excluded' else np.ones(len(b['q']),dtype=bool)
        b['keep']=keep
        e=np.rad2deg(simulate(b,template,substeps=20,robust=True)[:,0]-b['q'])
        baseline[b['run']]=baseline.get(b['run'],0)+float(e[keep]@e[keep])
        counts[b['run']]=counts.get(b['run'],0)+int(keep.sum())
    baseline={r:np.sqrt(v/counts[r]) for r,v in baseline.items()}
    def fun(z):
        nonlocal calls,failures
        obj=decode(z);parts=[];squares={}
        for b in bb:
            try:e=np.rad2deg(simulate(b,obj,substeps=20,robust=True)[:,0]-b['q'])
            except ValueError:
                failures+=1;e=np.full(len(b['q']),1e4)
            keep=b['keep'];idx=b['angle_idx'];sel=keep[idx]
            parts.append(e[idx[sel]]*b['angle_weight'][sel])
            sel=valid_windows(keep,b['win'],100);idx=b['win'][sel]
            parts.append((e[idx+100]-e[idx])*b['win_weight'][sel])
            for lag,idx in b['long'].items():
                idx=idx[valid_windows(keep,idx,lag)]
                parts.append((e[idx+lag]-e[idx])*b['long_weight'][lag])
            squares[b['run']]=squares.get(b['run'],0)+float(e[keep]@e[keep])
        parts.append(np.array([2*max(0,np.sqrt(squares[r]/counts[r])-max(1.08*baseline[r],baseline[r]+.15))/np.sqrt(len(counts)) for r in counts]))
        m=Model(obj);qgrid=np.deg2rad(np.linspace(5,80,61))
        k=np.gradient([m.potential_gradient(q) for q in qgrid],qgrid)
        total=m.p.gravity_nm*np.cos(qgrid)+m.p.elastic_k_nm_rad+k
        parts.extend([.1*m.area.ravel(),.01*np.diff(k),np.minimum(total,0)*.3,.03*m.rate_coeff.ravel()])
        result=np.concatenate(parts);calls+=1
        if calls%40==0:print(variant,'calls',calls,'loss',float(np.linalg.norm(result)),'seconds',round(time.time()-tic),'failed_rollouts',failures,flush=True)
        return result
    return fun


def fit(variant,nfev):
    OUT.mkdir(parents=True,exist_ok=True)
    bb=add_long_windows(prepare([b for b in load_blocks() if b['role']=='train']))
    template=json.loads((BASE/'selected_model.json').read_text());initial,decode=param(template)
    protocol=dict(intervals=INTERVALS,baseline_sha256=sha256(BASE/'selected_model.json'),
        initialization='Identical V7 warm start; excluded labels already informed V7, not independent held-out validation.',
        rollout='All measured pressures and original block initial states retained, no resets at exclusion boundaries.',
        loss='V7 form at fixed 5ms; same original weights, remove masked angle terms and any intersecting motion window; regression references evaluated on each fitted subset.',
        comparison='Paired full-data control and excluded fit; fixed budget, no development-based selection.',
        integration_ms=5,nfev=nfev,finite_difference_step=1e-5,
        solver='Safeguarded Newton with bisection every fourth iteration, 200 iterations, velocity residual tolerance 1e-9. Original V7 solver remains default.',
        untouched='V7 accepted snapshot, hardware configuration, raw logs, paper')
    dump(OUT/(variant+'_protocol.json'),protocol)
    fn=objective(bb,decode,variant)
    def jac(z):
        f=fn(z);cols=[]
        for j in range(len(z)):
            h=1e-5 if z[j]+1e-5<1 else -1e-5
            shifted=z.copy();shifted[j]+=h;cols.append((fn(shifted)-f)/h)
        return np.column_stack(cols)
    result=least_squares(fn,np.clip(initial,1e-8,1-1e-8),jac=jac,bounds=(0,1),
        x_scale=1.,max_nfev=nfev,ftol=1e-5,xtol=1e-6,gtol=1e-6)
    dump(OUT/(variant+'_model.json'),decode(result.x))
    dump(OUT/(variant+'_optimizer.json'),dict(nfev=result.nfev,success=bool(result.success),message=result.message,
        optimality=float(result.optimality),loss=float(np.linalg.norm(result.fun))))
    print('FINISHED',variant,result.message,flush=True)


def raw_quality():
    rows=[];blocks=load_blocks()
    for run in INTERVALS:
        path=Path('/home/risebrl/result/ph/4')/run/'run.csv'
        with path.open() as f:rr=list(csv.DictReader(f))
        t=np.array([float(r['t_mono_s']) for r in rr]);valid=np.zeros(len(t),dtype=bool)
        for b in blocks:
            if b['run']==run:valid|=(t>=b['t'][0])&(t<=b['t'][-1])
        for lo,hi in INTERVALS[run]:
            idx=np.flatnonzero((t>=lo)&(t<=hi)&valid);group=[rr[i] for i in idx]
            row=dict(run=run,start_s=lo,end_s=hi,raw_samples=len(idx))
            for key in ('angle_deg','p_pos_kpa','p_neg_kpa','angle_age_s','pressure_age_s','sensor_valid'):
                vals=np.array([float(r[key]) if r[key] else np.nan for r in group]);finite=np.isfinite(vals)
                row[key+'_nonfinite']=int((~finite).sum())
                row[key+'_min']=float(np.nanmin(vals));row[key+'_max']=float(np.nanmax(vals))
            row['dt_max_s']=float(np.diff(t[idx]).max())
            rows.append(row)
    write_csv(OUT/'raw_quality.csv',rows)


def evaluate():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    from .audit_structured import physics,independent
    bb=load_blocks();models={'accepted_v7':json.loads((BASE/'selected_model.json').read_text())}
    models.update({k:json.loads((OUT/(k+'_model.json')).read_text()) for k in ('control','excluded')})
    series={k:[] for k in models};summary=[];runrows=[];outrows=[]
    with (BASE/'all_runs.csv').open() as f:source=list(csv.DictReader(f))
    index=0
    for b in bb:
        mask=exclusion_mask(b);actual=np.rad2deg(b['q'])
        pred={k:np.rad2deg(simulate(b,m,substeps=20,robust=True)[:,0]) for k,m in models.items()}
        for j in range(len(actual)):
            row=dict(source[index]);row['excluded_from_refit']=int(mask[j])
            row['angle_control_refit_deg']=float(pred['control'][j]);row['angle_excluded_refit_deg']=float(pred['excluded'][j]);outrows.append(row);index+=1
        for k in models:
            e=pred[k]-actual
            series[k].append((b,mask,e,pred[k]))
    write_csv(OUT/'all_runs.csv',outrows)
    for k,items in series.items():
        for role in ('train','development','historical_diagnostic_prefix'):
            for subset in ('all','retained','excluded') if role=='train' else ('all',):
                parts=[]
                for b,mask,e,_ in items:
                    if b['role']==role:parts.extend(e if subset=='all' else e[~mask if subset=='retained' else mask])
                if parts:summary.append(dict(model=k,role=role,subset=subset,samples=len(parts),rmse_deg=float(np.sqrt(np.mean(np.square(parts))))))
        for run in dict.fromkeys(b['run'] for b in bb):
            for subset in ('all','retained','excluded'):
                parts=[]
                for b,mask,e,_ in items:
                    if b['run']==run:parts.extend(e if subset=='all' else e[~mask if subset=='retained' else mask])
                if parts:runrows.append(dict(model=k,run=run,subset=subset,samples=len(parts),rmse_deg=float(np.sqrt(np.mean(np.square(parts))))))
    write_csv(OUT/'summary.csv',summary);write_csv(OUT/'run_metrics.csv',runrows)
    with PdfPages(OUT/'comparison.pdf') as pdf:
        for number,run in enumerate(dict.fromkeys(b['run'] for b in bb),3):
            fig,axs=plt.subplots(2,1,figsize=(12,7),sharex=True)
            blocks=[b for b in bb if b['run']==run]
            for i,b in enumerate(blocks):
                axs[0].plot(b['t'],np.rad2deg(b['q']),color='black',label='Measured' if i==0 else None)
            for k,color in [('accepted_v7','gray'),('control','#1875bc'),('excluded','#ce6c14')]:
                items=[x for x in series[k] if x[0]['run']==run]
                for i,(b,mask,e,pred) in enumerate(items):
                    axs[0].plot(b['t'],pred,color=color,label=k if i==0 else None,lw=1.1)
                    axs[1].plot(b['t'],e,color=color,lw=1.1)
            for ax in axs:
                for lo,hi in INTERVALS.get(run,[]):ax.axvspan(lo,hi,color='red',alpha=.09)
                ax.grid(alpha=.2)
            axs[0].legend();axs[0].set_ylabel('Angle [deg]');axs[1].set_ylabel('Prediction - measured [deg]');axs[1].set_xlabel('Original logged time [s]')
            fig.suptitle(f'Graph {number}: {run}\nShaded: excluded from loss only; continuous pressure-driven rollout')
            fig.tight_layout();fig.savefig(OUT/f'{number:02d}_{run}.png',dpi=150);pdf.savefig(fig);plt.close(fig)
    for k in ('control','excluded'):
        dump(OUT/(k+'_physics.json'),physics(models[k]));errors=[]
        for b in bb:
            if b['role']=='development':
                ref=independent(b,models[k]);fast=simulate(b,models[k],substeps=20,robust=True)
                errors.extend(np.rad2deg(ref[:,0]-fast[:,0]))
        audit=dict(independent_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),
                   max_abs_difference_deg=float(np.max(np.abs(errors))))
        dump(OUT/(k+'_numerical.json'),audit)
        print('INDEPENDENT AUDIT',k,audit,flush=True)
        if audit['independent_vs_5ms_rmse_deg']>.03:raise ValueError('Independent numerical tolerance exceeded')
    raw_quality()
    print(json.dumps(summary,indent=2),flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=['control','excluded','evaluate']);ap.add_argument('--nfev',type=int,default=25)
    args=ap.parse_args()
    if args.action=='evaluate':evaluate()
    else:fit(args.action,args.nfev)


if __name__=='__main__':main()
