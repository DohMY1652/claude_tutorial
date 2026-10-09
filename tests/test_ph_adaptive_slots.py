import numpy as np
import pytest
from ph_model.structured_model import Model,zero_model,simulate


def adaptive():
    obj=zero_model();obj['feature_reference']['area_scales_m']=[.005,.012]
    obj['feature_reference']['x1_center_m']+=.003
    obj['area_coefficients']=[[.1,.1,-.1,.1],[0,.1,-.1,.1]]
    obj['history_k']=[.3,2.]
    obj['history_rate_coefficients']=[[.2,-.3],[-.5,.7]]
    return obj


def test_rate_changes_with_pressure_but_energy_does_not():
    m=Model(adaptive());z=np.array([.6,.03,.4,.7])
    r1=m.forces(z,np.array([-10000.,20000.]))[2]
    r2=m.forces(z,np.array([-50000.,20000.]))[2]
    assert not np.allclose(r1,r2)
    assert np.all(r1>0) and np.all(r2>0)
    for P in [np.array([-10000.,20000.]),np.array([-50000.,20000.])]:
        a=m.audit(z,P);assert abs(a['balance_error'])<1e-10
        assert a['hdot']<=a['supply']+1e-10


def test_new_fast_slots_agree_with_independent_solver():
    from scipy.integrate import solve_ivp
    obj=adaptive();obj['nominal']['epsilon_rad_s']=.002;m=Model(obj)
    t=np.arange(0,2.01,.1);P=np.column_stack((-20000-1000*t,15000+2000*t))
    b=dict(t=t,P=P,qs=np.array([.5]),v=np.array([0.]))
    ref=solve_ivp(lambda time,z:m.rhs(z,np.array([np.interp(time,t,P[:,i]) for i in range(2)])),
        [t[0],t[-1]],[.5,0,.5,.5],t_eval=t,method='LSODA',rtol=1e-9,atol=1e-11).y.T
    result=simulate(b,obj,substeps=100)
    assert np.max(abs(result[:,0]-ref[:,0]))<.003


def test_invalid_new_features_rejected():
    obj=adaptive();obj['feature_reference']['area_scales_m'][0]=0
    with pytest.raises(ValueError):Model(obj)
    obj=adaptive();obj['history_rate_coefficients'][0][0]=float('nan')
    with pytest.raises(ValueError):Model(obj)


def test_default_extensions_are_exact_v6_identity():
    obj=zero_model();other=zero_model()
    other['history_rate_coefficients']=[[0.,0.],[0.,0.]]
    other['feature_reference']['area_scales_m']=[obj['feature_reference']['area_scale_m']]*2
    for q in (.2,.6,1.2):
        P=np.array([-30000.,35000.])
        np.testing.assert_array_equal(Model(obj).port(q,P),Model(other).port(q,P))


def test_long_windows_never_cross_short_blocks():
    from ph_model.refit_adaptive_slots import add_long_windows
    bb=[dict(run='a',q=np.zeros(n)) for n in (100,1300)]
    add_long_windows(bb)
    for b in bb:
        for lag,idx in b['long'].items():assert np.all(idx+lag<len(b['q']))
    assert not len(bb[0]['long'][300])
    assert len(bb[1]['long'][1200])==1


def test_v7_selection_rejects_hidden_regression():
    import copy
    from ph_model.audit_adaptive_slots import assess
    names=['20261006_141418_972450_S6','20261006_142450_704718_S6','20261006_143524_612945_S6']
    def row(name):return dict(run=name,rmse_deg=1.,slow_missed_fraction=.2,hold_increment_rmse_deg=.1,slow_increment_rmse_deg=.3)
    reference=dict(training=[row(n) for n in names],development=[row('dev')])
    result=copy.deepcopy(reference);result['name']='test'
    for r in result['training']+result['development']:r['rmse_deg']=.8
    assert assess(result,reference)['eligible']
    result['training'][0]['rmse_deg']=1.3
    assert not assess(result,reference)['eligible']


def test_passive_balance_for_random_pressure_rate_weights():
    rng=np.random.default_rng(716)
    for _ in range(10):
        obj=adaptive();obj['history_rate_coefficients']=rng.uniform(-1.5,1.5,(2,2)).tolist()
        m=Model(obj)
        for _ in range(20):
            z=np.r_[rng.uniform(.1,1.4),rng.uniform(-.5,.5),rng.uniform(0,1.5,2)]
            P=np.array([-rng.uniform(0,75000),rng.uniform(0,85000)])
            a=m.audit(z,P)
            assert a['dissipation']>=0
            assert abs(a['balance_error'])<1e-9
            assert a['hdot']<=a['supply']+1e-9
