import numpy as np
import pytest
torch=pytest.importorskip('torch')
from ph_model.residual import PassiveResidual
from ph_model.nominal import Parameters
from ph_model.fit_residual import simulate_block
from ph_model.residual_fast import simulate


def test_compiled_offline_solver_matches_adaptive_all_states():
    torch.manual_seed(29)
    m=PassiveResidual(Parameters(),(0,1.3),(60000,70000))
    with torch.no_grad():
        for p in m.parameters():p.normal_(0,.05)
        m.history_k.copy_(torch.tensor([5.,20.]))
    m.project()
    t=np.arange(0,10.01,.1)
    b=dict(t=t,P=np.column_stack((-20000-2000*np.sin(t),10000+1000*np.sin(t))),
           qs=np.full(len(t),.4),v=np.full(len(t),.01))
    fast=simulate(b,m.export(),substeps=10)
    slow=simulate_block(b,m,rtol=1e-9,max_step=.005)
    np.testing.assert_allclose(fast,slow,atol=3e-4,rtol=0)


def test_compiled_solver_rejects_nonuniform_time():
    m=PassiveResidual(Parameters(),(0,1),(60000,70000))
    with pytest.raises(AssertionError):simulate(dict(t=[0,.2]),m.export())
