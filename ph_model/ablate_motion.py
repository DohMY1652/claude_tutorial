"""Training-only refit with old epsilon, isolating it from the new loss terms."""
import copy
import json
from pathlib import Path
import numpy as np
import torch
from .data import load_run
from .fit_nominal import RUNS,dump
from .refit_motion import OUT,prepared,objective_factory,optimize,fastscore,score_predictions
from .fit_residual import simulate_block
from .residual import PassiveResidual


def main():
    torch.set_num_threads(1)
    original=json.loads((OUT/'nominal_selected.json').read_text())
    train=[];dev=[]
    for name,role in RUNS:
        if role not in ('train','development'):continue
        r=load_run(Path('/home/risebrl/result/ph/4')/name)
        (train if role=='train' else dev).extend(prepared(r))
    names=['alpha','damping_nm_s_rad','elastic_k_nm_rad','elastic_bias_nm']
    def decode(z):
        obj=copy.deepcopy(original)
        obj['nominal'].update(dict(zip(names,map(float,z))))
        obj['nominal']['epsilon_rad_s']=.02
        return obj
    rows=[]
    starts=[[original['nominal'][n] for n in names],[.6,1.,1.74,-1.49]]
    for i,start in enumerate(starts):
        fit=optimize(objective_factory(train,decode,f'epsilon_ablation{i}'),np.array(start),
                     np.array([0,.0001,0,-3]),np.array([2,20,8,3]),40)
        obj=decode(fit.x)
        m=PassiveResidual.restore(obj)
        score=score_predictions(dev,[simulate_block(b,m) for b in dev])
        rows.append(dict(parameters=obj['nominal'],training_loss_deg=float(np.rad2deg(np.linalg.norm(fit.fun))),
                         success=bool(fit.success),development=score))
        dump(OUT/'epsilon_ablation.json',dict(purpose='Diagnostic only; same loss and fixed geometry/Mg; epsilon fixed to previous .02',candidates=rows))
        print('EPSILON ABLATION',rows[-1],flush=True)


if __name__=='__main__':main()
