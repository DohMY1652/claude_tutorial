"""Post-selection mechanism plots and fixed-parameter ablations, not selection."""
import copy
import json
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .structured_model import Model
from .refit_structured import OUT,scores
from .diagnose_structure import load_blocks,write_csv
from .nominal import gradient
from .fit_nominal import dump
from .plot_predictions import read_csv,block_slices


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--directory',type=Path,default=OUT)
    ap.add_argument('--previous-label',default='Previous full V4');ap.add_argument('--new-label',default='New full V6')
    ap.add_argument('--substeps',type=int,default=5,help='Offline integration substeps per 0.1s sample')
    args=ap.parse_args();out=args.directory
    obj=json.loads((out/'selected_model.json').read_text());m=Model(obj)
    data=read_csv(out/'all_runs.csv');runs=list(dict.fromkeys(data['run_id']))[1:10]
    fig,axs=plt.subplots(3,3,figsize=(18,11))
    for number,(ax,run) in enumerate(zip(axs.flat,runs),start=4):
        group={k:v[data['run_id']==run] for k,v in data.items()}
        for i,sl in enumerate(block_slices(group)):
            for key,label,color,style in [('angle_actual_deg','Measured','black','-'),
                ('angle_residual_previous_deg',args.previous_label,'#b46d2a','--'),
                ('angle_residual_selected_deg',args.new_label,'#1579b8','-')]:
                ax.plot(group['t_mono_s'][sl],group[key][sl],color=color,ls=style,lw=1.2,label=label if i==0 else None)
        ax.set_title(f"Graph {number}: {run.rsplit('_',1)[-1]}");ax.set_xlabel('Logged time [s]');ax.set_ylabel('Angle [deg]');ax.grid(alpha=.2)
    handles,labels=axs.flat[0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',ncol=3)
    fig.suptitle('Full-model comparison: same measured chamber pressures and initial-state policy',y=.96)
    fig.tight_layout(rect=(0,0,1,.94));fig.savefig(out/'before_after_04_12.png',dpi=170)
    fig.savefig(out/'before_after_04_12.pdf');plt.close(fig)
    q=np.deg2rad(np.linspace(5,80,301));Psets=[[-30000.,30000.],[-2000.,50000.],[-55000.,2000.]]
    fig,axs=plt.subplots(3,1,figsize=(10,10),sharex=True);rows=[]
    g=np.array([gradient(x,m.p)+m.potential_gradient(x) for x in q])
    axs[0].plot(np.rad2deg(q),g,label='Total pressure-independent restoring torque')
    axs[0].set_ylabel('Torque [Nm]');axs[0].legend()
    for P,label in zip(Psets,('Balanced: -30/+30 kPa gauge','Positive dominant: -2/+50','Vacuum dominant: -55/+2')):
        P=np.array(P);ports=np.array([m.port(x,P) for x in q]);tau=ports@P
        balance_stiffness=np.gradient(g-tau,q)
        axs[1].plot(np.rad2deg(q),-ports[:,0]/ports[:,1],label=label)
        axs[2].plot(np.rad2deg(q),balance_stiffness,label=label)
        for i in range(len(q)):
            rows.append(dict(angle_deg=float(np.rad2deg(q[i])),p1_pa=float(P[0]),p2_pa=float(P[1]),
                restoring_torque_nm=float(g[i]),input_torque_nm=float(tau[i]),
                area_ratio=float(-ports[i,0]/ports[i,1]),input_balance_stiffness_nm_rad=float(balance_stiffness[i])))
    axs[1].set_ylabel('Effective A_minus / A_plus');axs[2].set_ylabel('d(restoring - input)/dq\n[Nm/rad]')
    axs[2].set_xlabel('Angle [deg]');axs[1].legend();axs[2].legend()
    for ax in axs:ax.grid(alpha=.2)
    fig.suptitle('Physical slots: pressure dependence enters the input port, not stored energy')
    fig.tight_layout();fig.savefig(out/'physical_slots.png',dpi=160);plt.close(fig)
    write_csv(out/'physical_slots.csv',rows)
    bb=[b for b in load_blocks() if b['role'] in ('train','development')]
    variants={}
    for name in ('selected','without_area','without_history','without_potential','without_pressure_rate','history_tau_half','history_tau_double'):
        candidate=copy.deepcopy(obj)
        if name=='without_area':candidate['area_coefficients']=[[0.]*4,[0.]*4]
        if name=='without_history':candidate['history_k']=[0.,0.]
        if name=='without_potential':candidate['potential_coefficients']=[0.]*4
        if name=='without_pressure_rate':candidate['history_rate_coefficients']=[[0.,0.],[0.,0.]]
        if name=='history_tau_half':candidate['history_tau_s']=[x/2 for x in obj['history_tau_s']]
        if name=='history_tau_double':candidate['history_tau_s']=[x*2 for x in obj['history_tau_s']]
        try:
            variants[name]=scores(bb,candidate,substeps=args.substeps)
            print('ABLATION',name,'dev',variants[name][-1]['rmse_deg'],flush=True)
        except ValueError as error:
            variants[name]={'numerical_failure':str(error)}
            print('ABLATION',name,'FAILED',error,flush=True)
        dump(out/'postselection_ablations.json',dict(variants=variants,
            substeps=args.substeps,
            caveat='No refitting; joint parameters compensate each other. Not independent causal estimates or physical parameter confidence intervals. Tau doubling is post-selection sensitivity only, not a new selected fit.'))


if __name__=='__main__':main()
