"""Independent adaptive checks and exports for the move/hold offline refit."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import torch

from .data import load_run,sha256
from .fit_nominal import RUNS,dump,metrics
from .fit_residual import evaluate,simulate_block
from .residual import PassiveResidual,NumpyResidual
from .nominal import energy
from .audit_nominal_refit import physics
from .refit_motion import OUT,OLD,prepared,score_predictions,fast


def select(kind):
    devname=next(n for n,r in RUNS if r=='development')
    run=load_run(Path('/home/risebrl/result/ph/4')/devname);bb=prepared(run)
    candidates=json.loads((OUT/f'{kind}_candidates.json').read_text())
    for c in candidates:
        obj=json.loads((OUT/(c['name']+'.json')).read_text())
        m=PassiveResidual.restore(obj)
        predictions=[simulate_block(b,m) for b in bb]
        c['adaptive_development']=score_predictions(bb,predictions)
        print('ADAPTIVE',c['name'],c['adaptive_development'],flush=True)
        dump(OUT/f'{kind}_candidates.json',candidates)
    best=min(candidates,key=lambda c:c['adaptive_development']['selection_score_deg'])
    label='nominal' if kind=='nominal' else 'full'
    dump(OUT/f'{label}_selection.json',best)
    dump(OUT/f'{label}_selected.json',json.loads((OUT/(best['name']+'.json')).read_text()))
    print('ADAPTIVE SELECTION',label,best['name'],flush=True)


def final():
    protocol=json.loads((OUT/'protocol.json').read_text())
    assert sha256('main.tex')==protocol['paper_sha256']
    assert sha256(OLD/'residual/selected_model.json')==protocol['old_full_sha256']
    for item in protocol['files']:
        for name,h in item['hashes'].items():assert sha256(Path(item['path'])/name)==h
    objects={label:json.loads((OUT/f'{label}_selected.json').read_text()) for label in ('nominal','full')}
    assert objects['nominal']['nominal']==objects['full']['nominal']
    audit={label:physics(obj) for label,obj in objects.items()}
    m=PassiveResidual.restore(objects['full']);n=NumpyResidual(objects['full']);p=m.nominal
    t=np.arange(0,3.0001,.01)
    z=simulate_block(dict(t=t,P=np.zeros((len(t),2)),qs=np.full(len(t),.5),v=np.zeros(len(t))),m,rtol=1e-9,max_step=.002)
    H=energy(z[:,0],z[:,1],p)+n.slots_batch(z[:,0],np.zeros((len(t),2)))[0]+.5*np.sum(n.k*(z[:,0,None]-z[:,2:])**2,axis=1)
    audit['unforced_max_energy_increment_j']=float(np.max(np.diff(H)))
    assert audit['unforced_max_energy_increment_j']<1e-8
    dump(OUT/'physics_audit.json',audit)
    results={};rowindex=0;numerical={}
    with (OUT/'all_runs.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(['time_s','run_id','role','block_id','t_mono_s','p_pos_kpa_abs','p_neg_kpa_abs',
                         'angle_actual_deg','angle_nominal_refit_deg','angle_nominal_previous_deg',
                         'angle_residual_selected_deg','angle_residual_previous_deg','block_start'])
        for name,role in RUNS:
            if role not in ('train','development','validation_prefix'):continue
            r=load_run(Path('/home/risebrl/result/ph/4')/name,cutoff=356.45 if role=='validation_prefix' else None)
            if role=='validation_prefix':role='historical_diagnostic_prefix'
            bb=prepared(r);scores={};arrays={}
            for label,obj in objects.items():
                _,a=evaluate(r,PassiveResidual.restore(obj),OUT/'artifacts',f'{label}_{name}')
                arrays[label]=a
                scores[label]=score_predictions(bb,[a[f'block{i}_prediction'] for i in range(len(bb))])
                if role=='development':
                    coarse=[fast(b,obj) for b in bb];fine=[fast(b,obj,20) for b in bb]
                    numerical[label]=dict(euler_20_vs_5ms=metrics(np.concatenate([np.rad2deg(c[:,0]-d[:,0]) for c,d in zip(coarse,fine)])),
                        euler_5ms_vs_adaptive=metrics(np.concatenate([np.rad2deg(c[:,0]-a[f'block{i}_prediction'][:,0]) for i,c in enumerate(fine)])))
            with np.load(OLD/'residual/artifacts'/f'nominal_{name}.npz') as on, np.load(OLD/'residual/artifacts'/f'residual_{name}.npz') as of:
                for label,a in [('previous_nominal',on),('previous_full',of)]:
                    scores[label]=score_predictions(bb,[a[f'block{i}_prediction'] for i in range(len(bb))])
                for i,b in enumerate(bb):
                    k=f'block{i}_';new=arrays['full'];nom=arrays['nominal']
                    for a in (on,of,nom):
                        for key in ('t','q','P'):np.testing.assert_array_equal(a[k+key],new[k+key])
                    for j in range(len(b['t'])):
                        P=b['P'][j]
                        writer.writerow([f'{rowindex/10:.1f}',name,role,i,b['t'][j],P[1]/1000+101.325,P[0]/1000+101.325,
                            np.rad2deg(b['q'][j]),np.rad2deg(nom[k+'prediction'][j,0]),np.rad2deg(on[k+'prediction'][j,0]),
                            np.rad2deg(new[k+'prediction'][j,0]),np.rad2deg(of[k+'prediction'][j,0]),int(j==0)])
                        rowindex+=1
            results[name]=dict(role=role,**scores);dump(OUT/'metrics.json',results)
            print('FINAL',name,'nominal',scores['nominal']['rmse_deg'],'full',scores['full']['rmse_deg'],flush=True)
    dump(OUT/'numerical_audit.json',numerical)
    dump(OUT/'provenance.json',dict(rows=rowindex,paper_sha256=sha256('main.tex'),
         models={k:sha256(OUT/f'{k}_selected.json') for k in objects},source_files_unchanged=True,
         code={str(f):sha256(f) for f in Path('ph_model').glob('*') if f.suffix in ('.py','.cpp')}))
    print('FINAL MOTION AUDIT COMPLETE',rowindex,audit,flush=True)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--phase',choices=['nominal','residual','final'],required=True)
    args=ap.parse_args();torch.set_num_threads(1)
    if args.phase=='final':final()
    else:select(args.phase)


if __name__=='__main__':main()
