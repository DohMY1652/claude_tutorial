"""Audit/plot completed nominal sensitivity and central-nominal residual trials."""
import csv
import json
from pathlib import Path
import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from .data import sha256
from .nominal import Parameters,torque,area_minus,gradient,friction,energy
from .residual import PassiveResidual,NumpyResidual
from .fit_residual import simulate_block
from .residual_fast import simulate
from .fit_nominal import dump,metrics
from .refit_nominal import OUT,BASE


def physics(obj):
    m=PassiveResidual.restore(obj);n=NumpyResidual(obj);p=m.nominal
    rng=np.random.default_rng(1710);N=4000
    q=rng.uniform(-.08,1.53,N);v=rng.uniform(-.5,.5,N);xi=rng.uniform(-.08,1.53,(N,2))
    P=np.column_stack((-rng.uniform(0,80000,N),rng.uniform(0,85000,N)))
    tq=torch.tensor(q,requires_grad=True);tv=torch.tensor(v,requires_grad=True);tx=torch.tensor(xi,requires_grad=True)
    V,g,d,mu=m.slots(tq,torch.tensor(P))
    H=.5*p.inertia_kg_m2*tv**2-p.gravity_nm*torch.cos(tq)+.5*p.elastic_k_nm_rad*tq**2+p.elastic_bias_nm*tq+V+\
      .5*p.limit_k_nm_rad*(torch.clamp(tq-p.q_max_rad,min=0)**2+torch.clamp(p.q_min_rad-tq,min=0)**2)+\
      .5*torch.sum(m.history_k*(tq[:,None]-tx)**2,dim=1)
    Hq,Hv,Hx=[a.detach().numpy() for a in torch.autograd.grad(H.sum(),(tq,tv,tx))]
    _,ng,nd,nmu=n.slots_batch(q,P);tau=torque(q,P,p)
    f=n.k*(q[:,None]-xi);R=n.rv+n.rho*np.abs(v[:,None])
    a=(tau-gradient(q,p)-ng-friction(v,tau,p)-nd*v-nmu*np.tanh(v/p.epsilon_rad_s)-f.sum(axis=1))/p.inertia_kg_m2
    hd=Hq*v+Hv*a+np.sum(Hx*R*f,axis=1)
    power=np.sum(P*np.column_stack((-p.reel_radius_m*area_minus(p.x1_zero_m-p.reel_radius_m*q,p)*v,p.reel_radius_m*p.area_plus*v)),axis=1)
    loss=v*(friction(v,tau,p)+nd*v+nmu*np.tanh(v/p.epsilon_rad_s))+np.sum(R*f*f,axis=1)
    result=dict(states=N,energy_balance_max_w=float(np.max(np.abs(hd-power+loss))),
                power_identity_max_w=float(np.max(np.abs(power-tau*v))),violations=int(np.sum(hd-power>1e-9)))
    assert result['violations']==0 and result['energy_balance_max_w']<1e-8
    return result


def main():
    torch.set_num_threads(1);root=OUT/'residual'
    nominal=json.loads((OUT/'nominal_candidates.json').read_text())
    radii=sorted(json.loads((OUT/'radius_profile.json').read_text()),key=lambda x:x['radius_mm'])
    grav=json.loads((OUT/'gravity_profile.json').read_text())
    fig,ax=plt.subplots(1,3,figsize=(13,4))
    x=[r['radius_mm'] for r in radii]
    ax[0].plot(x,[r['dynamic']['development_fast']['rmse_deg'] for r in radii],'o-')
    ax[0].set(xlabel='Conditional reel radius [mm]',ylabel='Development RMSE [deg]',title='Not a measured radius')
    ax[1].plot(x,[r['central']['parameters']['gravity_nm'] for r in radii],'o-',label='Central fit')
    ax[1].plot(x,[r['dynamic']['parameters']['gravity_nm'] for r in radii],'s-',label='Joint rollout fit')
    ax[1].set(xlabel='Reel radius [mm]',ylabel='Gravity coefficient [Nm]');ax[1].legend()
    ax[2].plot([g['gravity_nm'] for g in grav],[g['central_force_rmse_n'] for g in grav],'o-')
    ax[2].set(xlabel='Fixed gravity coefficient [Nm]',ylabel='Reoptimized central force RMSE [N]',title='Elasticity refitted, not a CI')
    for a in ax:a.grid(alpha=.25)
    fig.tight_layout();fig.savefig(OUT/'identifiability.png',dpi=160);plt.close(fig)
    protocol=json.loads((root/'protocol.json').read_text())
    assert sha256('main.tex')==protocol['paper_sha256']
    assert sha256(BASE/'long_rollout_v2/selected_model.json')==protocol['reference_model_sha256']
    for item in json.loads((BASE/'ve0_baseline/provenance.json').read_text())['files']:
        for name,h in item['hashes'].items():assert sha256(Path(item['path'])/name)==h
    audit={}
    for name in ('central_residual_start0','central_residual_start1','selected_model'):
        obj=json.loads((root/f'{name}.json').read_text())
        if name!='selected_model':assert obj['nominal']==protocol['nominal']
        audit[name]=physics(obj)
    obj=json.loads((root/'selected_model.json').read_text());m=PassiveResidual.restore(obj);n=NumpyResidual(obj);p=m.nominal
    t=np.arange(0,3.0001,.01)
    z=simulate_block(dict(t=t,P=np.zeros((len(t),2)),qs=np.full(len(t),.5),v=np.zeros(len(t))),m,rtol=1e-9,max_step=.002)
    E=energy(z[:,0],z[:,1],p)+n.slots_batch(z[:,0],np.zeros((len(t),2)))[0]+.5*np.sum(n.k*(z[:,0,None]-z[:,2:])**2,axis=1)
    audit['unforced_max_energy_increase_j']=float(np.max(np.diff(E)));assert audit['unforced_max_energy_increase_j']<1e-8
    dump(OUT/'physics_audit.json',audit)
    name='20261006_150506_052414_S6'
    with np.load(root/'artifacts'/f'residual_{name}.npz') as arr:
        z=arr['block0_prediction'];b=dict(t=arr['block0_t'],P=arr['block0_P'],qs=z[:,0],v=z[:,1])
        fast=simulate(b,obj,substeps=2);fine=simulate(b,obj,substeps=10)
        dump(OUT/'numerical_audit.json',dict(midpoint_50_vs_10ms=metrics(np.rad2deg(fast[:,0]-fine[:,0])),
             midpoint_10ms_vs_adaptive=metrics(np.rad2deg(fine[:,0]-z[:,0]))))
    scores=json.loads((root/'metrics.json').read_text())
    old=json.loads((BASE/'long_rollout_v2/metrics.json').read_text())
    oldnom=json.loads((BASE/'comparison_metrics.json').read_text())
    comparison={};index=0
    with (OUT/'all_runs.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','run_id','role','block_id','t_mono_s',
          'p_pos_kpa_abs','p_neg_kpa_abs','angle_actual_deg','angle_nominal_refit_deg',
          'angle_nominal_previous_deg','angle_residual_selected_deg','angle_residual_previous_deg','block_start'])
        for name,s in scores.items():
            comparison[name]=dict(role=s['role'],previous_nominal=oldnom[name]['strong_nominal'],
                central_nominal=s['central_nominal'],previous_residual=old[name]['residual'],selected_residual=s['selected_residual'])
            with np.load(root/'artifacts'/f'nominal_{name}.npz') as nom, np.load(root/'artifacts'/f'residual_{name}.npz') as new, \
              np.load(BASE/'long_rollout_v2/artifacts'/f'residual_{name}.npz') as prior, np.load(BASE/'artifacts'/f'nominal_{name}.npz') as oldn:
                i=0
                while f'block{i}_t' in new:
                    k=f'block{i}_'
                    for a in (nom,prior,oldn):
                        for key in ('t','q','P'):np.testing.assert_array_equal(new[k+key],a[k+key])
                    t=new[k+'t'];P=new[k+'P'];q=np.rad2deg(new[k+'q'])
                    preds=[np.rad2deg(a[k+'prediction'][:,0]) for a in (nom,oldn,new,prior)]
                    for j in range(len(t)):
                        writer.writerow([f'{index/10:.1f}',name,s['role'],i,t[j],P[j,1]/1000+101.325,P[j,0]/1000+101.325,
                                         q[j],*[v[j] for v in preds],int(j==0)]);index+=1
                    i+=1
    dump(OUT/'comparison.json',comparison)
    dump(OUT/'audit_provenance.json',dict(selected_model_sha256=sha256(root/'selected_model.json'),
        paper_unchanged=True,source_data_unchanged=True,baseline_unchanged=True,export_rows=index,
        code={str(f):sha256(f) for f in Path('ph_model').glob('*') if f.suffix in ('.py','.cpp')}))
    print('NOMINAL REFIT AUDIT COMPLETE',audit,flush=True)


if __name__=='__main__':main()
