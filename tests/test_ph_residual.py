"""Offline residual mathematics only. Run with local optional torch dependency."""
import dataclasses
import numpy as np
import pytest
torch=pytest.importorskip('torch')
from ph_model.nominal import Parameters, torque, gradient, friction, energy
from ph_model.residual import PassiveResidual, NumpyResidual


def fixture_model():
    torch.manual_seed(17)
    return PassiveResidual(Parameters(),q_bounds=(0.,1.3),pressure_scale=(60000.,70000.))


def test_initial_residual_is_exactly_zero():
    m=fixture_model()
    q=torch.linspace(0,1.3,40,dtype=torch.float64,requires_grad=True)
    P=torch.tensor([[-10000.,20000.]]*40,dtype=torch.float64)
    pot,grad,d,mu=m.slots(q,P)
    assert all(torch.count_nonzero(x)==0 for x in (pot,grad,d,mu,m.history_k))
    assert all(p.dtype==torch.float64 for p in m.parameters())


def test_numpy_export_matches_autodiff_for_nonzero_network():
    m=fixture_model()
    with torch.no_grad():
        for p in m.parameters():p.normal_(0,.2)
    m.project()
    q=torch.linspace(.1,1.2,11,dtype=torch.float64,requires_grad=True)
    P=torch.tensor([[-11000.,23000.]]*len(q),dtype=torch.float64)
    arrays=[x.detach().numpy() for x in m.slots(q,P)]
    n=NumpyResidual(m.export())
    batch=n.slots_batch(q.detach().numpy(),P.numpy())
    np.testing.assert_allclose(batch,arrays,atol=2e-12)
    for i,(qi,pi) in enumerate(zip(q.detach().numpy(),P.numpy())):
        np.testing.assert_allclose(n.slots(qi,pi),[x[i] for x in arrays],atol=2e-12)
        h=1e-6
        assert (n.slots(qi+h,pi)[0]-n.slots(qi-h,pi)[0])/(2*h)==pytest.approx(arrays[1][i],abs=1e-7)


def test_full_energy_balance_with_history():
    m=fixture_model()
    with torch.no_grad():
        for p in m.parameters():p.normal_(0,.15)
        m.history_k.copy_(torch.tensor([5.,20.]))
    m.project();n=NumpyResidual(m.export());p=m.nominal
    rng=np.random.default_rng(8)
    for _ in range(100):
        q=rng.uniform(.1,1.3);v=rng.uniform(-.4,.4);xi=rng.uniform(0,1.3,2)
        P=np.array([-rng.uniform(0,60000),rng.uniform(0,70000)])
        potential,g,d,mu=n.slots(q,P)
        f=n.k*(q-xi);R=n.rv+n.rho*abs(v)
        a=(torque(q,P,p)-gradient(q,p)-g-friction(v,torque(q,P,p),p)-d*v-mu*np.tanh(v/p.epsilon_rad_s)-sum(f))/p.inertia_kg_m2
        Hdot=p.inertia_kg_m2*v*a+(gradient(q,p)+g+sum(f))*v-np.sum(f*R*f)
        expected=torque(q,P,p)*v-v*(friction(v,torque(q,P,p),p)+d*v+mu*np.tanh(v/p.epsilon_rad_s))-np.sum(R*f*f)
        assert Hdot==pytest.approx(expected,abs=1e-12)
        assert Hdot<=torque(q,P,p)*v+1e-12


def test_potential_independent_of_pressure_and_bounded():
    m=fixture_model()
    with torch.no_grad():
        for p in m.parameters():p.normal_(0,3)
    m.project()
    q=torch.linspace(-100,100,40,dtype=torch.float64,requires_grad=True)
    p0=torch.zeros((40,2),dtype=torch.float64)
    p1=torch.ones((40,2),dtype=torch.float64)*50000
    V0=m.slots(q,p0)[0];V1=m.slots(q,p1)[0]
    torch.testing.assert_close(V0,V1)
    assert torch.max(torch.abs(V0))<=2


def test_batched_residual_midpoint_against_independent_adaptive_solver():
    from ph_model.residual_rollout import prepare,batch_rollout
    from ph_model.fit_residual import simulate_block
    m=fixture_model()
    with torch.no_grad():
        for p in m.parameters():p.normal_(0,.05)
        m.history_k.copy_(torch.tensor([5.,10.]))
    m.project()
    t=np.linspace(0,10,101);P=np.column_stack((-20000-2000*np.sin(t),10000+1000*np.sin(t)))
    wins=[dict(t=t,pressure=P,z0=[.4,.01],q=np.zeros(len(t)))]
    batch=batch_rollout(prepare(wins,dt=.02),m.export())[0]
    adaptive=simulate_block(dict(t=t,P=P,qs=np.full(len(t),.4),v=np.full(len(t),.01)),m,rtol=1e-8,max_step=.005)
    assert np.max(np.abs(batch-adaptive[:,0]))<np.deg2rad(.03)
