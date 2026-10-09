"""V7 selection against preserved V6, numerical/energy audit and export."""
import csv
import json
from pathlib import Path
import numpy as np
from .refit_adaptive_slots import BASE,OUT,add_long_windows
from .refit_structured import scores,prepare
from .audit_structured import physics,independent
from .structured_model import simulate,Model
from .diagnose_structure import load_blocks,write_csv
from .fit_nominal import dump
from .data import sha256


def assess(result,reference):
    rows=result['training']+result['development'];old={r['run']:r for r in reference['training']+reference['development']}
    failures=[]
    for r in rows:
        limit=max(1.08*old[r['run']]['rmse_deg'],old[r['run']]['rmse_deg']+.15)
        if r['rmse_deg']>limit:failures.append(r['run']+': angle regression')
    seeds=['20261006_141418_972450_S6','20261006_142450_704718_S6','20261006_143524_612945_S6']
    new={r['run']:r for r in rows};before=[old[s]['slow_missed_fraction'] for s in seeds];after=[new[s]['slow_missed_fraction'] for s in seeds]
    if any(a>b+.03 for a,b in zip(after,before)) or np.mean(after)>np.mean(before):failures.append('slow-motion miss gate')
    average=np.mean([r['rmse_deg'] for r in rows]);oldmean=np.mean([r['rmse_deg'] for r in old.values()])
    if average>=oldmean:failures.append('mean-run RMSE gate')
    d=result['development'][0];d0=reference['development'][0]
    if d['rmse_deg']>d0['rmse_deg']-.01:failures.append('development improvement <0.01deg')
    if d['hold_increment_rmse_deg']>max(1.25*d0['hold_increment_rmse_deg'],d0['hold_increment_rmse_deg']+.05):failures.append('development held-motion gate')
    return dict(name=result['name'],eligible=not failures,failures=failures,mean_run_rmse_deg=float(average),
        score=float(d['rmse_deg']+.4*d['slow_increment_rmse_deg']+.2*average),slow_missed_fractions=after)


def select():
    reference=json.loads((OUT/'baseline_metrics.json').read_text());candidates=[]
    for path in sorted(OUT.glob('*_start*_metrics.json')):candidates.append(assess(json.loads(path.read_text()),reference))
    good=[c for c in candidates if c['eligible']]
    choice=min(good,key=lambda c:c['score']) if good else None
    dump(OUT/'selection.json',dict(candidates=candidates,selected=choice['name'] if choice else None))
    if not choice:raise ValueError('No candidate passes V6 regression gates; V6 preserved')
    return json.loads((OUT/(choice['name']+'.json')).read_text())


def extra_motion(bb,obj):
    rows=[]
    for b in add_long_windows(bb):
        actual=np.rad2deg(b['q']);pred=np.rad2deg(simulate(b,obj,substeps=20)[:,0]);e=pred-actual
        for lag,idx in b['long'].items():
            for i in idx:rows.append(dict(run=b['run'],block=b['block'],kind=f'hold_{lag/10:g}s',t_s=float(b['t'][i]),
                actual_change_deg=float(actual[i+lag]-actual[i]),predicted_change_deg=float(pred[i+lag]-pred[i]),
                increment_error_deg=float(e[i+lag]-e[i])))
        # Evaluation only: clear 1s direction reversals, at least 5s apart.
        idx=np.arange(5,len(actual)-5);velocity=actual[idx+5]-actual[idx-5]
        valid=idx[np.abs(velocity)>.1];sign=np.sign(velocity[np.abs(velocity)>.1]);last=-1000
        for j in range(1,len(valid)):
            i=valid[j]
            if sign[j]==sign[j-1] or i-last<50 or i<30 or i+30>=len(e):continue
            last=i;rows.append(dict(run=b['run'],block=b['block'],kind='reversal_6s',t_s=float(b['t'][i]),
                actual_change_deg=float(actual[i+30]-actual[i-30]),predicted_change_deg=float(pred[i+30]-pred[i-30]),
                increment_error_deg=float(e[i+30]-e[i-30])))
    return rows


def export(bb,obj):
    with (BASE/'all_runs.csv').open(newline='') as f:source=list(csv.DictReader(f))
    rows=[];index=0
    for b in bb:
        pred=simulate(b,obj,substeps=20)
        for j in range(len(b['q'])):
            row=dict(source[index]);assert row['run_id']==b['run'] and int(row['block_id'])==b['block']
            row['angle_residual_previous_deg']=row['angle_residual_selected_deg']
            row['angle_residual_selected_deg']=float(np.rad2deg(pred[j,0]));rows.append(row);index+=1
    assert index==len(source);write_csv(OUT/'all_runs.csv',rows)
    metrics=[]
    for run in dict.fromkeys(r['run_id'] for r in rows):
        group=[r for r in rows if r['run_id']==run];item=dict(run=run,role=group[0]['role'])
        for label,key in [('nominal','angle_nominal_refit_deg'),('full','angle_residual_selected_deg'),('previous_full','angle_residual_previous_deg')]:
            e=np.array([float(r[key])-float(r['angle_actual_deg']) for r in group]);item[label+'_rmse_deg']=float(np.sqrt(np.mean(e*e)))
        metrics.append(item)
    write_csv(OUT/'run_metrics.csv',metrics);dump(OUT/'motion_metrics.json',scores(bb,obj))
    write_csv(OUT/'extra_motion.csv',extra_motion(bb,obj));write_csv(OUT/'baseline_extra_motion.csv',extra_motion(bb,json.loads((BASE/'selected_model.json').read_text())))
    dump(OUT/'provenance.json',dict(baseline_sha256=sha256(BASE/'selected_model.json'),baseline_csv_sha256=sha256(BASE/'all_runs.csv'),
        selected_sha256=sha256(OUT/'selected_model.json'),paper_sha256=sha256('main.tex'),rows=len(rows),
        note='Exact measured/timing/nominal columns preserved from V6; updated full prediction only; 5ms export, unchanged state reset policy. Historical seed4 is not blind.'))


def main():
    bb=load_blocks();obj=select();errors=[];refinement=[]
    for b in bb:
        if b['role']!='development':continue
        ref=independent(b,obj);fine=simulate(b,obj,substeps=20);coarse=simulate(b,obj)
        errors.extend(np.rad2deg(ref[:,0]-fine[:,0]));refinement.extend(np.rad2deg(fine[:,0]-coarse[:,0]))
    numerical=dict(independent_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),
        steps_20ms_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(refinement)))))
    dump(OUT/'numerical_audit.json',numerical)
    if max(numerical.values())>.03:raise ValueError('Numerical validation failed')
    audit=physics(obj);m=Model(obj)
    audit['rate_multiplier_bounds']=[np.exp(-np.sum(abs(m.rate_coeff),axis=1)).tolist(),np.exp(np.sum(abs(m.rate_coeff),axis=1)).tolist()]
    audit['history_rate_coefficients']=m.rate_coeff.tolist();audit['feature_reference']=m.f
    dump(OUT/'physics_audit.json',audit);dump(OUT/'selected_model.json',obj);export(bb,obj)
    print('V7 SELECTED AND AUDITED',flush=True)


if __name__=='__main__':main()
