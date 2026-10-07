import numpy as np
import pytest
torch=pytest.importorskip('torch')
from ph_model.residual import PassiveResidual, NumpyResidual
from ph_model.nominal import Parameters
from ph_model.improve_residual import parameterization


def test_long_rollout_parameterization_preserves_physical_slots():
    m=PassiveResidual(Parameters(),(0,1.3),(60000,70000))
    with torch.no_grad():
        m.history_k.copy_(torch.tensor([.02,20.]))
        m.mu_mix.fill_(.1)
    obj=m.export();initial,lo,hi,decode=parameterization(obj)
    for z in (initial,lo,hi,(lo+hi)/2):
        restored=decode(z);n=NumpyResidual(restored)
        assert restored['nominal']==obj['nominal']
        for key in ('history_rv','d_mix','potential_net.0.weight','dissipation_net.0.weight'):
            assert restored['state'][key]==obj['state'][key]
        assert np.all(n.k>0) and np.all(n.rho>0)
        q=np.linspace(0,1.3,10);P=np.tile([-20000,30000],(10,1))
        V,g,d,mu=n.slots_batch(q,P)
        assert np.all(np.abs(V)<=2) and np.all(d>=0) and np.all(mu>=0)
