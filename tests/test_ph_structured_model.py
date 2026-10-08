import copy
import numpy as np
import pytest
from ph_model.structured_model import Model, zero_model, simulate
from ph_model.nominal import torque, gradient, friction


def test_zero_heads_recover_nominal():
    m=Model(zero_model())
    q,v=.6,.02;P=np.array([-30000.,20000.]);z=np.array([q,v,q,q])
    expected=(torque(q,P,m.p)-gradient(q,m.p)-friction(v,torque(q,P,m.p),m.p))/m.p.inertia_kg_m2
    assert m.rhs(z,P)[1]==pytest.approx(expected)
    assert m.potential(q)==0


def test_power_and_energy_identity_random_states():
    obj=zero_model();obj['area_coefficients']=[[.1,.15,-.1,.1],[0,.1,.1,-.1]]
    obj['potential_coefficients']=[.02,-.03,.01,.04]
    obj['mu_coefficients']=[.1,.2];obj['history_k']=[.3,2.]
    m=Model(obj);rng=np.random.default_rng(38)
    for _ in range(100):
        z=np.r_[rng.uniform(.1,1.4),rng.uniform(-.5,.5),rng.uniform(.1,1.4,2)]
        P=np.array([-rng.uniform(0,70000),rng.uniform(0,85000)])
        terms=m.audit(z,P)
        assert abs(terms['power_error'])<1e-12
        assert abs(terms['balance_error'])<1e-10
        assert terms['dissipation']>=0
        assert terms['hdot']<=terms['supply']+1e-10


def test_potential_force_is_autodiff_gradient():
    import torch
    obj=zero_model();obj['potential_coefficients']=[.02,-.03,.01,.04];m=Model(obj)
    q=torch.tensor(.72,dtype=torch.float64,requires_grad=True)
    g=torch.autograd.grad(m.potential_torch(q),q)[0]
    assert float(g)==pytest.approx(m.potential_gradient(.72),abs=1e-12)


def test_invalid_coefficients_rejected():
    obj=zero_model();obj['history_k'][0]=-1
    with pytest.raises(ValueError):Model(obj)
    obj=zero_model();obj['area_coefficients'][0]=[2.,0,0,0]
    with pytest.raises(ValueError):Model(obj)


def test_energy_has_no_pressure_and_directional_derivative_matches_rhs():
    obj=zero_model();obj['history_k']=[.5,2.];obj['potential_coefficients']=[.03,-.01,.02,0]
    obj['mu_coefficients']=[.1,.2];m=Model(obj)
    z=np.array([.6,.04,.5,.7]);P=np.array([-35000.,22000.]);dz=m.rhs(z,P);h=1e-7
    finite=(m.energy(z+h*dz)-m.energy(z-h*dz))/(2*h)
    assert finite==pytest.approx(m.audit(z,P)['hdot'],abs=1e-7)


def test_no_input_energy_nonincrease_with_memory():
    from scipy.integrate import solve_ivp
    obj=zero_model();obj['history_k']=[.5,2.];m=Model(obj);z=[.5,0.,.3,.4]
    sol=solve_ivp(lambda t,z:m.rhs(z,np.zeros(2)),[0,3],z,t_eval=np.linspace(0,3,301),
        method='LSODA',rtol=1e-9,atol=1e-11)
    assert sol.success
    assert np.max(np.diff([m.energy(x) for x in sol.y.T]))<1e-7


def test_fast_solver_converges_to_independent_solver():
    from scipy.integrate import solve_ivp
    obj=zero_model();obj['history_k']=[.2,1.]
    obj['potential_coefficients']=[.01,-.01,.01,0.];obj['area_coefficients'][0]=[.05,.1,0,0]
    obj['nominal']['epsilon_rad_s']=.002;m=Model(obj)
    t=np.arange(0,2.01,.1);P=np.tile([-20000.,15000.],(len(t),1))
    b=dict(t=t,P=P,qs=np.array([.5]),v=np.array([0.]))
    z0=np.array([.5,0.,.5,.5]);ref=solve_ivp(lambda t,z:m.rhs(z,P[0]),[t[0],t[-1]],z0,
        t_eval=t,method='LSODA',rtol=1e-9,atol=1e-11).y.T
    coarse=simulate(b,obj,substeps=5);fine=simulate(b,obj,substeps=100)
    assert np.max(np.abs(fine[:,0]-ref[:,0]))<.003
    assert np.linalg.norm(fine-ref)<np.linalg.norm(coarse-ref)


def test_global_positive_area_and_pressure_torque_direction():
    obj=zero_model();obj['area_coefficients']=[[-.2,.2,-.2,-.2],[0,-.25,.25,-.25]]
    m=Model(obj)
    for q in np.linspace(0,1.5,8):
        for amplitude in (1000,30000,80000):
            P=np.array([-float(amplitude),float(amplitude)])
            g=m.port(q,P);assert g[0]<0<g[1]
            for j,sign in [(0,-1),(1,1)]:
                d=np.zeros(2);d[j]=1
                derivative=(m.port(q,P+d)@(P+d)-m.port(q,P-d)@(P-d))/2
                assert sign*derivative>0


def test_selection_cannot_hide_profile_regression_in_average():
    from ph_model.audit_structured import gate
    old={'bad':{'full':{'rmse_deg':2.}},'good':{'full':{'rmse_deg':10.}}}
    failures=gate([{'run':'bad','rmse_deg':3.},{'run':'good','rmse_deg':1.}],old)
    assert [r['run'] for r in failures]==['bad']
