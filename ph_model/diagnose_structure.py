"""Read-only V4 diagnostics and conditional nominal parameter screening (no ROS).

Candidates are NOT replacements. Physical dimensions are effective conditional
estimates with fixed area and inertia anchors, not independently measured values.
"""
import argparse
import copy
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .nominal import gradient, torque
from .residual import NumpyResidual
from .fit_nominal import dump
from .data import sha256
from .refit_motion import fast, optimize

BASE = Path('reports/residual_actuator4_20261007/motion_refit_v4')
OUT = BASE.parent / 'structure_diagnosis_v5'


def windows(q, lag=100, stride=10):
    """Measured-only windows; q degrees, 10 Hz, never cross a valid block."""
    i = np.arange(0, max(0, len(q)-lag), stride, dtype=int)
    amplitude = np.array([np.ptp(q[j:j+lag+1]) for j in i])
    return i, np.where(amplitude < .3, 'hold', np.where(amplitude < 3, 'slow', 'large'))


def write_csv(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def load_blocks():
    data = np.genfromtxt(BASE/'all_runs.csv', delimiter=',', names=True, dtype=None, encoding='utf8')
    result = []
    for run in dict.fromkeys(data['run_id']):
        rr = data[data['run_id'] == run]
        with np.load(BASE/'artifacts'/f'nominal_{run}.npz') as a:
            for block in np.unique(rr['block_id']):
                rows = rr[rr['block_id'] == block]
                z = a[f'block{block}_prediction']
                result.append(dict(run=run, role=rows['role'][0], t=rows['t_mono_s'],
                    q=np.deg2rad(rows['angle_actual_deg']), qs=np.array([z[0,0]]), v=np.array([z[0,1]]),
                    P=np.column_stack((rows['p_neg_kpa_abs']-101.325,rows['p_pos_kpa_abs']-101.325))*1000,
                    nominal=rows['angle_nominal_refit_deg'], full=rows['angle_residual_selected_deg'], block=int(block)))
    return result


def diagnose(bb):
    obj = json.loads((BASE/'full_selected.json').read_text()); model = NumpyResidual(obj)
    q = np.deg2rad(np.linspace(5,85,801)); P = np.tile([-20000.,30000.], (len(q),1))
    _, g, d, mu = model.slots_batch(q,P)
    gn = gradient(q,model.nominal); stiffness = np.gradient(gn+g,q)
    write_csv(OUT/'restoring_curve.csv', [dict(angle_deg=float(np.rad2deg(x)),nominal_nm=float(a),
        residual_nm=float(b),full_stiffness_nm_rad=float(c)) for x,a,b,c in zip(q,gn,g,stiffness)])
    fig,axs = plt.subplots(2,1,figsize=(9,7),sharex=True)
    axs[0].plot(np.rad2deg(q),gn,label='Nominal');axs[0].plot(np.rad2deg(q),gn+g,label='Full')
    axs[0].set_ylabel('Restoring torque [Nm]');axs[0].legend()
    axs[1].plot(np.rad2deg(q),np.gradient(gn,q),label='Nominal')
    axs[1].plot(np.rad2deg(q),stiffness,label='Full');axs[1].set_ylabel('dG/dq [Nm/rad]')
    axs[1].set_xlabel('Angle [deg]');axs[1].legend()
    for ax in axs: ax.axvspan(45,65,color='orange',alpha=.15);ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(OUT/'01_restoring_stiffness.png',dpi=160);plt.close(fig)
    summary=[];bins=[];detail=[]
    for b in bb:
        actual=np.rad2deg(b['q']);idx,regime=windows(actual)
        for name in ('nominal','full'):
            pred=b[name]
            for i,kind in zip(idx,regime):
                sl=slice(i,i+101); ar=float(np.ptp(actual[sl])); pr=float(np.ptp(pred[sl]))
                detail.append(dict(run=b['run'],role=b['role'],block=b['block'],model=name,
                    time_s=float(b['t'][i]),regime=kind,actual_range_deg=ar,predicted_range_deg=pr,
                    increment_error_deg=float((pred[i+100]-pred[i])-(actual[i+100]-actual[i])),
                    missed_motion=int(kind!='hold' and pr < .2*ar)))
            dq=np.gradient(actual,.1)
            for low in range(0,80,10):
                for direction,sgn in [('up',1),('down',-1)]:
                    mask=(actual>=low)&(actual<low+10)&(sgn*dq>.05)
                    if np.any(mask):
                        e=pred[mask]-actual[mask]
                        bins.append(dict(run=b['run'],block=b['block'],model=name,angle_low_deg=low,
                            direction=direction,count=int(mask.sum()),bias_deg=float(np.mean(e)),rmse_deg=float(np.sqrt(np.mean(e*e)))))
    for run in dict.fromkeys(b['run'] for b in bb):
        for name in ('nominal','full'):
            for regime in ('hold','slow','large'):
                rows=[r for r in detail if r['run']==run and r['model']==name and r['regime']==regime]
                if rows:
                    summary.append(dict(run=run,model=name,regime=regime,windows=len(rows),
                        missed_fraction=float(np.mean([r['missed_motion'] for r in rows])),
                        increment_rmse_deg=float(np.sqrt(np.mean([r['increment_error_deg']**2 for r in rows]))),
                        median_range_ratio=float(np.median([r['predicted_range_deg']/max(r['actual_range_deg'],.01) for r in rows]))))
    write_csv(OUT/'motion_windows.csv',detail);write_csv(OUT/'motion_summary.csv',summary);write_csv(OUT/'angle_direction_bias.csv',bins)
    fig,axs=plt.subplots(3,1,figsize=(12,9))
    for ax,regime in zip(axs,('hold','slow','large')):
        rows=[r for r in summary if r['model']=='full' and r['regime']==regime]
        ax.bar(np.arange(len(rows)),[r['median_range_ratio'] for r in rows]);ax.axhline(1,color='black',lw=1)
        ax.set_xticks(np.arange(len(rows)));ax.set_xticklabels([r['run'][16:] for r in rows],rotation=35,ha='right',fontsize=8)
        ax.set_ylabel(regime+'\nrange ratio');ax.grid(axis='y',alpha=.2)
    fig.suptitle('Full / measured 10-second angle range (median; overlapping windows)')
    fig.tight_layout();fig.savefig(OUT/'02_motion_regimes.png',dpi=160);plt.close(fig)
    dump(OUT/'structure.json',dict(epsilon_rad_s=model.nominal.epsilon_rad_s,
        history_k=model.k.tolist(),history_relaxation_days=(1/(model.k*model.rv)/86400).tolist(),
        dissipation_viscous_range=[float(d.min()),float(d.max())],
        extra_coulomb_range_nm=[float(mu.min()),float(mu.max())],
        minimum_stiffness_nm_rad=float(stiffness.min()),minimum_stiffness_angle_deg=float(np.rad2deg(q[np.argmin(stiffness)])),
        interpretation='Restoring derivative only, NOT full branch tangent; positive here, no bistability demonstrated.',
        windows='10 seconds every 1 second within valid blocks; hold range <0.3deg, slow 0.3..3deg, large >=3deg; overlapping descriptive windows, not independent samples'))
    # Evaluate the force budget on the actual free-rollout state (not by
    # substituting measured angle into a predicted trajectory).
    budgets=[]
    for graph,run in [(7,'20261006_135507_219340_S3'),(10,'20261006_141418_972450_S6'),
                      (11,'20261006_142450_704718_S6'),(12,'20261006_143524_612945_S6')]:
        fig,axs=plt.subplots(2,1,figsize=(12,6),sharex=True)
        with np.load(BASE/'artifacts'/f'full_{run}.npz') as a:
            for b in [b for b in bb if b['run']==run]:
                state=a[f"block{b['block']}_prediction"];q=state[:,0]
                _,g,_,mu=model.slots_batch(q,b['P'])
                tau=torque(q,b['P'],model.nominal)
                hist=np.sum((q[:,None]-state[:,2:])*model.k,axis=1)
                drive=tau-gradient(q,model.nominal)-g-hist
                threshold=model.nominal.alpha*np.abs(tau)+mu
                ratio=drive/np.maximum(threshold,1e-10)
                axs[0].plot(b['t'],np.rad2deg(b['q']),color='black',label='Measured')
                axs[0].plot(b['t'],np.rad2deg(q),color='tab:orange',label='Full')
                axs[1].plot(b['t'],ratio,color='tab:blue')
                for j in range(0,len(q),10):
                    budgets.append(dict(run=run,block=b['block'],t_s=float(b['t'][j]),
                        actual_deg=float(np.rad2deg(b['q'][j])),predicted_deg=float(np.rad2deg(q[j])),
                        predicted_speed_deg_s=float(np.rad2deg(state[j,1])),drive_nm=float(drive[j]),
                        coulomb_scale_nm=float(threshold[j]),history_torque_nm=float(hist[j]),
                        drive_over_coulomb_scale=float(ratio[j])))
        axs[0].set_ylabel('Angle [deg]')
        handles,labels=axs[0].get_legend_handles_labels();unique=dict(zip(labels,handles))
        axs[0].legend(unique.values(),unique.keys());axs[0].set_title(f'Original graph {graph}: sticking diagnosis')
        axs[1].axhspan(-1,1,color='grey',alpha=.15);axs[1].axhline(1,color='grey',ls='--');axs[1].axhline(-1,color='grey',ls='--')
        axs[1].set_ylim(-1.5,1.5);axs[1].set_ylabel('Drive / Coulomb scale');axs[1].set_xlabel('Logged run time [s]')
        for ax in axs:ax.grid(alpha=.2)
        fig.tight_layout();fig.savefig(OUT/f'graph_{graph:02d}_sticking.png',dpi=150);plt.close(fig)
    write_csv(OUT/'rollout_torque_budget.csv',budgets)


def evaluate_candidates(bb):
    """Report plain angle errors separately from the optimization score."""
    rows=[]
    for path in sorted(OUT.glob('conditional_candidate_*.json')):
        obj=json.loads(path.read_text())
        for run in dict.fromkeys(b['run'] for b in bb if b['role'] in ('train','development')):
            group=[b for b in bb if b['run']==run];errors=[];old=[];refinement=[]
            for b in group:
                pred=fast(b,obj)[:,0]
                errors.extend(np.rad2deg(pred-b['q']));old.extend(b['nominal']-np.rad2deg(b['q']))
                if b['role']=='development':refinement.extend(np.rad2deg(fast(b,obj,substeps=20)[:,0]-pred))
            rows.append(dict(candidate=path.stem,run=run,role=group[0]['role'],
                rmse_deg=float(np.sqrt(np.mean(np.square(errors)))),
                baseline_nominal_rmse_deg=float(np.sqrt(np.mean(np.square(old)))),
                refinement_20ms_vs_5ms_rmse_deg=float(np.sqrt(np.mean(np.square(refinement)))) if refinement else ''))
    write_csv(OUT/'conditional_run_metrics.csv',rows)


def chamber_comparison():
    """Conditional same-angle force-balance proxy, NOT area measurement.

    Equal conservative force and identical direction-dependent friction are
    assumptions; their failure also changes this apparent ratio.
    """
    root=Path('/home/risebrl/result/ph/4')
    paths=[root/name/'points.csv' for name in ('20261006_134834_301155_S1a','20261006_135218_489972_S1b')]
    tables=[]
    for path in paths:
        with path.open() as f:tables.append(list(csv.DictReader(f)))
    model=NumpyResidual(json.loads((BASE/'full_selected.json').read_text()));p=model.nominal
    from .nominal import area_minus
    rows=[]
    for direction in ('up','down'):
        aa,bb=[sorted([x for x in table if x['sweep']==direction and x['settled']=='1'],
                      key=lambda x:float(x['angle_deg'])) for table in tables]
        for angle in (15,20,25,30,35,40):
            if not max(float(aa[0]['angle_deg']),float(bb[0]['angle_deg']))<=angle<=min(float(aa[-1]['angle_deg']),float(bb[-1]['angle_deg'])):continue
            def interp(table,key):
                return np.interp(angle,[float(x['angle_deg']) for x in table],[float(x[key]) for x in table])
            ratio=(interp(aa,'p_pos_meas')-interp(bb,'p_pos_meas'))/(interp(aa,'p_neg_meas')-interp(bb,'p_neg_meas'))
            rows.append(dict(direction=direction,angle_deg=angle,apparent_area_ratio=float(ratio),
                nominal_area_ratio=float(area_minus(p.x1_zero_m-p.reel_radius_m*np.deg2rad(angle),p)/p.area_plus)))
    write_csv(OUT/'conditional_chamber_ratio.csv',rows)
    fig,ax=plt.subplots(figsize=(8,5))
    for direction in ('up','down'):
        rr=[r for r in rows if r['direction']==direction]
        ax.plot([r['angle_deg'] for r in rr],[r['apparent_area_ratio'] for r in rr],'o-',label='Apparent, '+direction)
    ax.plot([r['angle_deg'] for r in rr],[r['nominal_area_ratio'] for r in rr],'--',label='Nominal geometry')
    ax.set(xlabel='Matched measured angle [deg]',ylabel='A_minus / A_plus (conditional proxy)',
           title='Single-chamber comparison: friction/history assumptions apply')
    ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(OUT/'03_chamber_ratio.png',dpi=160);plt.close(fig)
    dump(OUT/'provenance.json',dict(paper_sha256=sha256('main.tex'),
        sources={str(path):sha256(path) for path in paths+[BASE/'all_runs.csv',BASE/'full_selected.json',BASE/'nominal_selected.json']},
        notes='Original source data and selected V4 checkpoints unchanged. Chamber proxy interpolates logged settled means, not recalibrated raw sensor measurements.'))


def fit_screen(bb):
    train=[b for b in bb if b['role']=='train'];dev=[b for b in bb if b['role']=='development']
    template=json.loads((BASE/'nominal_selected.json').read_text())
    names=['alpha','damping_nm_s_rad','elastic_k_nm_rad','elastic_bias_nm','epsilon_rad_s',
           'gravity_nm','reel_radius_m','x1_zero_m']
    lo=np.array([.001,.001,.001,-3,np.log(1e-6),.2,.02,.05])
    hi=np.array([1.5,20,8,3,np.log(.05),6,.03,.13])
    start=np.array([template['nominal'][n] for n in names]);start[4]=np.log(start[4])
    def decode(z):
        obj=copy.deepcopy(template);x=lo+z*(hi-lo);x[4]=np.exp(x[4])
        obj['nominal'].update(dict(zip(names,map(float,x))));return obj
    def error(blocks,obj):
        parts=[]
        counts={run:sum(len(b['q'][::10]) for b in blocks if b['run']==run) for run in set(b['run'] for b in blocks)}
        for b in blocks:
            try:
                pred=fast(b,obj)[:,0];e=np.rad2deg(pred-b['q'])
            except ValueError:
                # Fixed-size finite rejection of an invalid candidate, never
                # substitute a clipped or successful-looking trajectory.
                e=np.full(len(b['q']),1e5)
            parts.append(e[::10]/np.sqrt(counts[b['run']]*len(counts)))
            # Include the previously unrepresented intermediate slow motions.
            i,kind=windows(np.rad2deg(b['q']))
            for regime in ('hold','slow','large'):
                j=i[kind==regime]
                if len(j):parts.append((e[j+100]-e[j])/np.sqrt(len(j)*len(blocks)))
        return np.concatenate(parts)
    results=[]
    baseline={role:float(np.linalg.norm(error(bs,template))) for role,bs in [('train',train),('development',dev)]}
    for index,mg in enumerate([2.943,1.0]):
        initial=start.copy();initial[5]=mg;calls=0
        def fn(z):
            nonlocal calls
            calls+=1;val=error(train,decode(z))
            if calls%40==0:print('screen',index,'calls',calls,'loss',float(np.linalg.norm(val)),flush=True)
            return val
        fit=optimize(fn,(initial-lo)/(hi-lo),np.zeros(8),np.ones(8),35)
        obj=decode(fit.x);dump(OUT/f'conditional_candidate_{index}.json',obj)
        singular=np.linalg.svd(fit.jac,compute_uv=False)
        results.append(dict(start_gravity_nm=mg,parameters=obj['nominal'],nfev=fit.nfev,optimizer_success=bool(fit.success),
            training_score=float(np.linalg.norm(fit.fun)),development_score=float(np.linalg.norm(error(dev,obj))),
            scaled_jacobian_singular_values=singular.tolist(),scaled_jacobian_condition=float(singular[0]/singular[-1])))
        dump(OUT/'conditional_screen.json',dict(baseline=baseline,candidates=results,
            caveats='Training only; development for diagnosis, no historical seed4 selection. Fixed D=0.05m,J=0.045kgm2. Bounds are exploration bounds, not metrology confidence intervals. No selected model overwritten. Score mixes angle and regime increment errors, not angle RMSE.'))
        print('CANDIDATE',index,results[-1],flush=True)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fit-screen',action='store_true');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);bb=load_blocks();diagnose(bb);chamber_comparison()
    if args.fit_screen:fit_screen(bb)
    evaluate_candidates(bb)


if __name__=='__main__':main()
