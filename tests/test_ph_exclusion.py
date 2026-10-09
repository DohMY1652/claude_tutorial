import numpy as np
from ph_model.exclusion_study import exclusion_mask, valid_windows


def test_exclusion_is_fixed_time_rule_not_prediction_error():
    t=np.array([79.,80.,150.,280.,281.])
    b=dict(run='20261006_141418_972450_S6',t=t)
    np.testing.assert_array_equal(exclusion_mask(b),[False,True,True,True,False])
    b['run']='20261006_150506_052414_S6'
    assert not exclusion_mask(b).any()


def test_mask_does_not_delete_time_or_pressure_history():
    t=np.arange(0.,400.,.1);P=np.column_stack((t,-t));q=np.ones(len(t))
    b=dict(run='20261006_141418_972450_S6',t=t,P=P,q=q)
    exclusion_mask(b)
    np.testing.assert_array_equal(b['t'],t)
    np.testing.assert_array_equal(b['P'],P)
    assert len(b['q'])==4000


def test_motion_windows_do_not_bridge_excluded_samples():
    keep=np.ones(20,dtype=bool);keep[8]=False
    idx=np.array([0,4,9,14])
    np.testing.assert_array_equal(valid_windows(keep,idx,5),[True,False,True,True])


def test_robust_solver_preserves_simple_trajectory():
    from ph_model.structured_model import zero_model,simulate
    obj=zero_model();t=np.arange(0,2.01,.1)
    b=dict(t=t,P=np.tile([-20000.,20000.],(len(t),1)),qs=[.5],v=[0.])
    np.testing.assert_allclose(simulate(b,obj,substeps=20,robust=True),simulate(b,obj,substeps=20),atol=1e-7,rtol=0)
