"""Independent physical and numerical checks of the long-rollout candidate."""
import json
import csv
from pathlib import Path
import numpy as np
import torch

from .data import sha256
from .fit_nominal import dump, metrics
from .fit_residual import simulate_block
from .nominal import torque, gradient, friction, area_minus, energy
from .residual import PassiveResidual, NumpyResidual
from .residual_fast import simulate


def main():
    root=Path('reports/residual_actuator4_20261007');out=root/'long_rollout_v2'
    obj=json.loads((out/'selected_model.json').read_text())
    original=json.loads((root/'best_model.json').read_text())
    protocol=json.loads((out/'protocol.json').read_text())
    assert obj['nominal']==original['nominal']
    assert sha256(root/'best_model.json')==protocol['baseline_sha256']
    assert sha256('main.tex')==protocol['paper_sha256']
    for source in protocol['train_files']:
        for f,h in source['hashes'].items():assert sha256(Path(source['path'])/f)==h
    prior_inputs=json.loads((root/'ve0_baseline/provenance.json').read_text())['files']
    for source in prior_inputs:
        for f,h in source['hashes'].items():assert sha256(Path(source['path'])/f)==h
    torch.set_num_threads(1)
    m=PassiveResidual.restore(obj);n=NumpyResidual(obj);p=m.nominal
    rng=np.random.default_rng(1707);N=4000
    q=rng.uniform(-.08,1.53,N);v=rng.uniform(-.5,.5,N);xi=rng.uniform(-.08,1.53,(N,2))
    P=np.column_stack((-rng.uniform(0,80000,N),rng.uniform(0,85000,N)))
    tq=torch.tensor(q,requires_grad=True);tv=torch.tensor(v,requires_grad=True);tx=torch.tensor(xi,requires_grad=True)
    V,g,d,mu=m.slots(tq,torch.tensor(P))
    H=.5*p.inertia_kg_m2*tv**2-p.gravity_nm*torch.cos(tq)+.5*p.elastic_k_nm_rad*tq**2+p.elastic_bias_nm*tq+V + \
        .5*p.limit_k_nm_rad*(torch.clamp(tq-p.q_max_rad,min=0)**2+torch.clamp(p.q_min_rad-tq,min=0)**2)+ \
        .5*torch.sum(m.history_k*(tq[:,None]-tx)**2,dim=1)
    Hq,Hv,Hx=[a.detach().numpy() for a in torch.autograd.grad(H.sum(),(tq,tv,tx))]
    _,ng,nd,nmu=n.slots_batch(q,P);tau=torque(q,P,p)
    f=n.k*(q[:,None]-xi);R=n.rv+n.rho*np.abs(v[:,None])
    a=(tau-gradient(q,p)-ng-friction(v,tau,p)-nd*v-nmu*np.tanh(v/p.epsilon_rad_s)-f.sum(axis=1))/p.inertia_kg_m2
    hd=Hq*v+Hv*a+np.sum(Hx*R*f,axis=1)
    port=np.sum(P*np.column_stack((-p.reel_radius_m*area_minus(p.x1_zero_m-p.reel_radius_m*q,p)*v,p.reel_radius_m*p.area_plus*v)),axis=1)
    diss=v*(friction(v,tau,p)+nd*v+nmu*np.tanh(v/p.epsilon_rad_s))+np.sum(R*f*f,axis=1)
    physics=dict(states=N,power_identity_max_w=float(np.max(np.abs(port-tau*v))),
        energy_balance_max_w=float(np.max(np.abs(hd-port+diss))),violations=int(np.sum(hd-port>1e-9)),
        minimum_dissipation_w=float(np.min(diss)),gradient_disagreement_nm=float(np.max(np.abs(ng-g.detach().numpy()))),
        nominal_unchanged=True,baseline_unchanged=True,paper_unchanged=True,all_source_data_unchanged=True,
        history_k=n.k.tolist(),history_rho=n.rho.tolist(),history_rv=n.rv.tolist())
    assert physics['violations']==0 and physics['energy_balance_max_w']<1e-8
    t=np.arange(0,3.0001,.01)
    b=dict(t=t,P=np.zeros((len(t),2)),qs=np.full(len(t),.5),v=np.zeros(len(t)))
    z=simulate_block(b,m,rtol=1e-9,max_step=.002)
    E=energy(z[:,0],z[:,1],p)+n.slots_batch(z[:,0],b['P'])[0]+.5*np.sum(n.k*(z[:,0,None]-z[:,2:])**2,axis=1)
    physics['unforced_max_energy_increase_j']=float(np.max(np.diff(E)))
    assert physics['unforced_max_energy_increase_j']<1e-8
    dump(out/'physics_audit.json',physics)
    name='20261006_150506_052414_S6'
    with np.load(out/'artifacts'/f'residual_{name}.npz') as saved:
        z=saved['block0_prediction'];b=dict(t=saved['block0_t'],P=saved['block0_P'],qs=z[:,0],v=z[:,1])
        fast=simulate(b,obj,substeps=2);fine=simulate(b,obj,substeps=10)
        tight=simulate_block(b,m,rtol=2e-7,max_step=.01)
        numerical=dict(midpoint_50_vs_10ms=metrics(np.rad2deg(fast[:,0]-fine[:,0])),
            midpoint_10ms_vs_adaptive=metrics(np.rad2deg(fine[:,0]-tight[:,0])),
            adaptive_tolerance=metrics(np.rad2deg(z[:,0]-tight[:,0])))
        dump(out/'numerical_audit.json',numerical)
    old=json.loads((root/'comparison_metrics.json').read_text());new=json.loads((out/'metrics.json').read_text())
    comparison={}
    for name,v in new.items():
        prior=old[name]['selected_residual'];nom=old[name]['strong_nominal'];score=v['residual']
        comparison[name]=dict(role=v['role'],previous_residual=prior,new_residual=score,nominal=nom,
            reduction_vs_previous_percent=100*(1-score['rmse_deg']/prior['rmse_deg']))
    dump(out/'comparison.json',comparison)
    # New CSV only. Preserve the baseline and USB files byte-for-byte.
    fields=['time_s','run_id','role','block_id','t_mono_s','p_pos_kpa_abs','p_neg_kpa_abs',
            'angle_pred_deg','angle_actual_deg','angle_nominal_deg','angle_previous_residual_deg',
            'error_deg','block_start']
    row_id=0
    with (out/'all_runs.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(fields)
        for name,entry in comparison.items():
            with np.load(out/'artifacts'/f'residual_{name}.npz') as new, \
                 np.load(root/'ve0_baseline/artifacts'/f'residual_{name}.npz') as old, \
                 np.load(root/'artifacts'/f'nominal_{name}.npz') as nominal:
                i=0
                while f'block{i}_t' in new:
                    key=f'block{i}_';t=new[key+'t'];P=new[key+'P']
                    for k in ('t','q','P'):
                        np.testing.assert_array_equal(new[key+k],old[key+k])
                        np.testing.assert_array_equal(new[key+k],nominal[key+k])
                    q=np.rad2deg(new[key+'q']);pred=np.rad2deg(new[key+'prediction'][:,0])
                    prior=np.rad2deg(old[key+'prediction'][:,0]);base=np.rad2deg(nominal[key+'prediction'][:,0])
                    for j in range(len(t)):
                        writer.writerow([f'{row_id/10:.1f}',name,entry['role'],i,t[j],
                            P[j,1]/1000+101.325,P[j,0]/1000+101.325,pred[j],q[j],base[j],prior[j],pred[j]-q[j],int(j==0)])
                        row_id+=1
                    i+=1
    dump(out/'audit_provenance.json',dict(selected_model_sha256=sha256(out/'selected_model.json'),
        code={str(f):sha256(f) for f in Path('ph_model').glob('*') if f.suffix in ('.py','.cpp')}))
    print('LONG ROLLOUT AUDIT COMPLETE',physics,numerical,flush=True)


if __name__=='__main__':main()
