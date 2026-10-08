"""Selection gates, independent ODE/physics audit, and immutable-reference export."""
import argparse
import copy
import csv
import json
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from .structured_model import Model,zero_model,simulate
from .refit_structured import OUT,scores
from .diagnose_structure import BASE,load_blocks,windows,write_csv
from .fit_nominal import dump
from .data import sha256


def gate(rows,old):
    failures=[]
    for row in rows:
        before=old[row['run']]['full']['rmse_deg'];limit=max(before*1.1,before+.25)
        if row['rmse_deg']>limit:failures.append(dict(run=row['run'],before=before,after=row['rmse_deg'],limit=limit))
    return failures


def independent(b,obj):
    m=Model(obj);t=b['t'];P=b['P'];z0=np.array([b['qs'][0],b['v'][0],*b.get('xi0',[b['qs'][0]]*2)])
    def rhs(time,z):return m.rhs(z,np.array([np.interp(time,t,P[:,j]) for j in range(2)]))
    sol=solve_ivp(rhs,[t[0],t[-1]],z0,t_eval=t,method='LSODA',rtol=1e-6,atol=1e-8,max_step=.05)
    if not sol.success:raise ValueError(sol.message)
    return sol.y.T


def physics(obj):
    m=Model(obj);rng=np.random.default_rng(4928);values=[]
    for _ in range(5000):
        q=rng.uniform(.02,1.53);z=np.r_[q,rng.uniform(-1,1),rng.uniform(-.1,1.6,2)]
        P=np.array([-rng.uniform(0,75000),rng.uniform(0,85000)])
        values.append(m.audit(z,P))
    ans=dict(states=len(values),max_power_error=max(abs(v['power_error']) for v in values),
        max_balance_error=max(abs(v['balance_error']) for v in values),minimum_dissipation=min(v['dissipation'] for v in values),
        violations=sum(v['hdot']>v['supply']+1e-10 for v in values),
        energy_residual_global_lower_bound_j=-float(np.sum(np.abs(m.c))),
        area_multiplier_global_lower_bounds=(1-np.sum(np.abs(m.area),axis=1)).tolist(),
        history_k=m.k.tolist(),history_tau_s=m.tau.tolist(),history_rho=m.rho.tolist(),
        history_rv=[float(1/(k*t)) if k>0 else 0. for k,t in zip(m.k,m.tau)])
    sol=solve_ivp(lambda t,z:m.rhs(z,np.zeros(2)),[0,5],[.7,.2,.3,.8],
        t_eval=np.linspace(0,5,1001),method='LSODA',rtol=1e-9,atol=1e-11)
    if not sol.success:raise ValueError(sol.message)
    E=np.array([m.energy(z) for z in sol.y.T]);ans['unforced_energy_max_increase_j']=float(np.diff(E).max())
    assert ans['violations']==0 and ans['max_balance_error']<1e-9 and ans['unforced_energy_max_increase_j']<1e-7
    return ans


def select(bb):
    old=json.loads((BASE/'metrics.json').read_text());candidates=[]
    for path in sorted(OUT.glob('*_start*_metrics.json')):
        if path.name.startswith('nominal'):continue
        result=json.loads(path.read_text());rows=result['training']+result['development']
        failures=gate(rows,old)
        # Seed0..2 movement gate uses the already frozen V5 metric definition.
        seedruns=['20261006_141418_972450_S6','20261006_142450_704718_S6','20261006_143524_612945_S6']
        oldmiss=[.45161290322580644,.589041095890411,.5481927710843374]
        missed=[next(r['slow_missed_fraction'] for r in rows if r['run']==name) for name in seedruns]
        aggregate=np.mean([r['rmse_deg'] for r in rows]);before=np.mean([old[r['run']]['full']['rmse_deg'] for r in rows])
        eligible=not failures and aggregate<before and all(a<b for a,b in zip(missed,oldmiss))
        dev=result['development'][0]
        score=float(np.sqrt(dev['rmse_deg']**2+dev['slow_increment_rmse_deg']**2)+.25*aggregate)
        candidates.append(dict(name=result['name'],eligible=bool(eligible),gate_failures=failures,
            mean_run_rmse_deg=float(aggregate),development_selection_score=score,slow_missed_fractions=missed))
    good=[c for c in candidates if c['eligible']]
    # Do not promote wider exploratory bounds for a floating-point-sized gain.
    bestscore=min((c['development_selection_score'] for c in good),default=float('inf'))
    equivalent=[c for c in good if c['development_selection_score']<=bestscore+.005]
    chosen=min(equivalent,key=lambda c:('wider' in c['name'],c['development_selection_score'])) if equivalent else None
    dump(OUT/'selection.json',dict(candidates=candidates,selected=chosen['name'] if chosen else None,
        equivalent_score_tolerance=.005,tie_policy='Prefer base bounds over wider sensitivity within 0.005 score units'))
    if not good:raise ValueError('No candidate meets predeclared selection gates; V4 retained')
    name=chosen['name']
    obj=json.loads((OUT/(name+'.json')).read_text())
    # Independent validation before any final export or selected checkpoint.
    errors=[];refinement=[]
    for b in bb:
        if b['role']!='development':continue
        fine=simulate(b,obj,substeps=20);coarse=simulate(b,obj);ref=independent(b,obj)
        errors.extend(np.rad2deg(ref[:,0]-fine[:,0]));refinement.extend(np.rad2deg(fine[:,0]-coarse[:,0]))
    audit=dict(selected=name,independent_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),
        steps_20ms_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(refinement)))))
    dump(OUT/'numerical_audit.json',audit)
    if max(audit['independent_vs_5ms_rmse_deg'],audit['steps_20ms_vs_5ms_rmse_deg'])>.03:
        raise ValueError('Numerical audit exceeds 0.03 degree tolerance')
    dump(OUT/'physics_audit.json',physics(obj));dump(OUT/'selected_model.json',obj)
    return obj


def export(bb,obj):
    rows=[];metrics=[];index=0
    with (BASE/'all_runs.csv').open(newline='') as f:source_rows=list(csv.DictReader(f))
    # Pure nominal refit is not adopted when its S1a error worsens the reference.
    nominal=zero_model();dump(OUT/'nominal_reference.json',nominal)
    dump(OUT/'nominal_selection.json',dict(selected='V4 pure nominal retained',
        reason='V6 nominal-only candidate worsened S1a 7.153 -> 8.240deg; full candidate jointly reidentifies nominal coefficients, not a frozen-nominal ablation.'))
    for b in bb:
        pred=simulate(b,obj,substeps=20);q=np.rad2deg(b['q']);full=np.rad2deg(pred[:,0])
        for j in range(len(q)):
            source=source_rows[index]
            assert source['run_id']==b['run'] and int(source['block_id'])==b['block']
            rows.append(dict(time_s=index/10,run_id=b['run'],role=b['role'],block_id=b['block'],t_mono_s=float(b['t'][j]),
                p_pos_kpa_abs=float(source['p_pos_kpa_abs']),p_neg_kpa_abs=float(source['p_neg_kpa_abs']),
                angle_actual_deg=float(source['angle_actual_deg']),angle_nominal_refit_deg=float(b['nominal'][j]),
                angle_nominal_previous_deg=float(b['nominal'][j]),angle_residual_selected_deg=float(full[j]),
                angle_residual_previous_deg=float(b['full'][j]),block_start=int(j==0)))
            index+=1
    assert index==len(source_rows)
    write_csv(OUT/'all_runs.csv',rows)
    for run in dict.fromkeys(r['run_id'] for r in rows):
        group=[r for r in rows if r['run_id']==run]
        row=dict(run=run,role=group[0]['role'])
        for name,key in [('nominal','angle_nominal_refit_deg'),('full','angle_residual_selected_deg'),('previous_full','angle_residual_previous_deg')]:
            e=np.array([r[key]-r['angle_actual_deg'] for r in group]);row[name+'_rmse_deg']=float(np.sqrt(np.mean(e*e)))
        metrics.append(row)
    write_csv(OUT/'run_metrics.csv',metrics)
    dump(OUT/'motion_metrics.json',scores(bb,obj))
    dump(OUT/'provenance.json',dict(paper_sha256=sha256('main.tex'),baseline_csv_sha256=sha256(BASE/'all_runs.csv'),
        baseline_model_sha256=sha256(BASE/'full_selected.json'),selected_model_sha256=sha256(OUT/'selected_model.json'),
        rows=len(rows),notes='5ms implicit Euler export; pressure/angle rows and block initial conditions unchanged. Historical prefix only after selection, not blind.'))


def main():
    bb=load_blocks();obj=select(bb);export(bb,obj)
    print('SELECTED, AUDITED, EXPORTED',OUT,flush=True)


if __name__=='__main__':main()
