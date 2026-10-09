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


def finite_jac(fn,z,workers=1):
    from concurrent.futures import ThreadPoolExecutor
    f=fn(z)
    def column(j):
        h=1e-5 if z[j]+1e-5<1 else -1e-5
        shifted=z.copy();shifted[j]+=h
        return (fn(shifted)-f)/h
    if workers==1:return np.column_stack([column(j) for j in range(len(z))])
    with ThreadPoolExecutor(max_workers=workers) as pool:
        return np.column_stack(list(pool.map(column,range(len(z)))))


def fit(variant,nfev,workers=1):
    OUT.mkdir(parents=True,exist_ok=True)
    bb=add_long_windows(prepare([b for b in load_blocks() if b['role']=='train']))
    template=json.loads((BASE/'selected_model.json').read_text());initial,decode=param(template)
    protocol=dict(intervals=INTERVALS,baseline_sha256=sha256(BASE/'selected_model.json'),
        initialization='Identical V7 warm start; excluded labels already informed V7, not independent held-out validation.',
        rollout='All measured pressures and original block initial states retained, no resets at exclusion boundaries.',
        loss='V7 form at fixed 5ms; same original weights, remove masked angle terms and any intersecting motion window; regression references evaluated on each fitted subset.',
        comparison='Paired full-data control and excluded fit; fixed budget, no development-based selection.',
        integration_ms=5,nfev=nfev,finite_difference_step=1e-5,workers=workers,
        solver='Safeguarded Newton with bisection every fourth iteration, 200 iterations, velocity residual tolerance 1e-9. Original V7 solver remains default.',
        untouched='V7 accepted snapshot, hardware configuration, raw logs, paper')
    dump(OUT/(variant+'_protocol.json'),protocol)
    fn=objective(bb,decode,variant)
    iteration=0
    def jac(z):
        nonlocal iteration
        iteration+=1
        dump(OUT/(variant+'_progress_model.json'),decode(z))
        dump(OUT/(variant+'_progress.json'),dict(jacobian_iteration=iteration,
             normalized_parameters=z.tolist(),status='Intermediate iterate, not final optimizer result'))
        return finite_jac(fn,z,workers)
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
        meta=json.loads(path.with_name('meta.json').read_text())
        with path.open() as f:rr=list(csv.DictReader(f))
        t=np.array([float(r['t_mono_s']) for r in rr]);valid=np.zeros(len(t),dtype=bool)
        for b in blocks:
            if b['run']==run:valid|=(t>=b['t'][0])&(t<=b['t'][-1])
        for lo,hi in INTERVALS[run]:
            idx=np.flatnonzero((t>=lo)&(t<=hi)&valid);group=[rr[i] for i in idx]
            row=dict(run=run,start_s=lo,end_s=hi,raw_samples=len(idx))
            row.update(status=meta.get('status'),actuator=meta.get('actuator'),mount=meta.get('mount'),
                       encoder_0=meta['encoder']['raw_0deg'],encoder_90=meta['encoder']['raw_90deg'],
                       freshness_scope=meta.get('freshness_scope'))
            for key in ('angle_deg','p_pos_kpa','p_neg_kpa','angle_age_s','pressure_age_s','sensor_valid'):
                vals=np.array([float(r[key]) if r[key] else np.nan for r in group]);finite=np.isfinite(vals)
                row[key+'_nonfinite']=int((~finite).sum())
                row[key+'_min']=float(np.nanmin(vals));row[key+'_max']=float(np.nanmax(vals))
            row['dt_max_s']=float(np.diff(t[idx]).max())
            angle=np.array([float(r['angle_deg']) for r in group])
            cuts=np.r_[0,np.flatnonzero(np.diff(angle)!=0)+1,len(angle)]
            row['angle_distinct_values']=int(len(np.unique(angle)))
            row['longest_identical_angle_s']=float(max(t[idx[end-1]]-t[idx[start]] for start,end in zip(cuts[:-1],cuts[1:])))
            rows.append(row)
    write_csv(OUT/'raw_quality.csv',rows)


def audit_model(k):
    from .audit_structured import physics,independent
    path=OUT/(k+'_model.json');model=json.loads(path.read_text())
    audit_path=OUT/(k+'_numerical.json')
    if audit_path.exists():
        saved=json.loads(audit_path.read_text())
        if saved.get('model_sha256')==sha256(path) and saved['independent_vs_5ms_rmse_deg']<=.03:return saved
    dump(OUT/(k+'_physics.json'),physics(model));errors=[]
    for b in load_blocks():
        if b['role']=='development':
            ref=independent(b,model);fast=simulate(b,model,substeps=20,robust=True)
            errors.extend(np.rad2deg(ref[:,0]-fast[:,0]))
    audit=dict(independent_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),
               max_abs_difference_deg=float(np.max(np.abs(errors))),model_sha256=sha256(path))
    dump(audit_path,audit);print('INDEPENDENT AUDIT',k,audit,flush=True)
    if audit['independent_vs_5ms_rmse_deg']>.03:raise ValueError('Independent numerical tolerance exceeded')
    return audit


def motion_diagnostics():
    bb=add_long_windows(load_blocks());rows=[]
    models={'accepted_v7':json.loads((BASE/'selected_model.json').read_text())}
    models.update({k:json.loads((OUT/(k+'_model.json')).read_text()) for k in ('control','excluded')})
    for name,obj in models.items():
        groups={}
        for b in bb:
            e=np.rad2deg(simulate(b,obj,substeps=20,robust=True)[:,0]-b['q'])
            for lag,idx in b['long'].items():groups.setdefault((b['role'],lag),[]).extend(e[idx+lag]-e[idx])
        for (role,lag),errors in groups.items():
            if errors:rows.append(dict(model=name,role=role,window_s=lag/10,windows=len(errors),
                 increment_rmse_deg=float(np.sqrt(np.mean(np.square(errors))))))
    write_csv(OUT/'long_motion.csv',rows)
    return rows


def evaluate():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    bb=load_blocks();models={'accepted_v7':json.loads((BASE/'selected_model.json').read_text())}
    models.update({k:json.loads((OUT/(k+'_model.json')).read_text()) for k in ('control','excluded')})
    series={k:[] for k in models};summary=[];runrows=[];outrows=[]
    with (BASE/'all_runs.csv').open() as f:source=list(csv.DictReader(f))
    index=0
    for b in bb:
        mask=exclusion_mask(b);actual=np.rad2deg(b['q'])
        pred={k:np.rad2deg(simulate(b,m,substeps=20,robust=True)[:,0]) for k,m in models.items()}
        for j in range(len(actual)):
            row=dict(source[index])
            assert row['run_id']==b['run'] and int(row['block_id'])==b['block']
            assert abs(float(row['t_mono_s'])-float(b['t'][j]))<1e-8
            row['excluded_from_refit']=int(mask[j])
            row['angle_control_refit_deg']=float(pred['control'][j]);row['angle_excluded_refit_deg']=float(pred['excluded'][j]);outrows.append(row);index+=1
        for k in models:
            e=pred[k]-actual
            series[k].append((b,mask,e,pred[k]))
    assert index==len(source)
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
    focus=list(INTERVALS)+['20261006_150506_052414_S6']
    fig,axs=plt.subplots(2,2,figsize=(15,9))
    for number,(ax,run) in enumerate(zip(axs.flat,focus),10):
        group=[b for b in bb if b['run']==run]
        for i,b in enumerate(group):ax.plot(b['t'],np.rad2deg(b['q']),color='black',lw=1.5,label='Measured' if i==0 else None)
        for k,color in [('accepted_v7','gray'),('control','#1875bc'),('excluded','#ce6c14')]:
            for i,(b,mask,e,pred) in enumerate(x for x in series[k] if x[0]['run']==run):
                ax.plot(b['t'],pred,color=color,lw=1.1,label=k if i==0 else None)
        for lo,hi in INTERVALS.get(run,[]):ax.axvspan(lo,hi,color='red',alpha=.09)
        ax.set_title(f'Graph {number}'+(' (development; never fitted)' if number==13 else ''))
        ax.set_xlabel('Logged time [s]');ax.set_ylabel('Angle [deg]');ax.grid(alpha=.2)
    handles,labels=axs.flat[0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',ncol=4)
    fig.suptitle('Exclusion sensitivity: shaded intervals removed from loss, not from state history',y=.95)
    fig.tight_layout(rect=(0,0,1,.92));fig.savefig(OUT/'comparison_focus.png',dpi=160);plt.close(fig)
    for k in ('control','excluded'):audit_model(k)
    raw_quality()
    motion_diagnostics()
    dump(OUT/'provenance.json',dict(baseline_model_sha256=sha256(BASE/'selected_model.json'),
        baseline_csv_sha256=sha256(BASE/'all_runs.csv'),paper_sha256=sha256('main.tex'),
        models={k:sha256(OUT/(k+'_model.json')) for k in ('control','excluded')},
        solver_sha256=sha256('ph_model/structured_model.cpp'),rows=len(outrows),
        raw_quality_sources={run:sha256(Path('/home/risebrl/result/ph/4')/run/'run.csv') for run in INTERVALS},
        note='All baseline columns preserved; only added mask/control/excluded predictions. No state reset at mask boundaries. V7 remains accepted.'))
    print(json.dumps(summary,indent=2),flush=True)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=['control','excluded','evaluate']);ap.add_argument('--nfev',type=int,default=25)
    ap.add_argument('--workers',type=int,default=1)
    args=ap.parse_args()
    if args.action=='evaluate':evaluate()
    else:fit(args.action,args.nfev,args.workers)


if __name__=='__main__':main()
