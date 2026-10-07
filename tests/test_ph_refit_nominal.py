import numpy as np
import pytest
import dataclasses
pytest.importorskip('torch')
from ph_model.refit_nominal import pair_branches,central_residual
from ph_model.nominal import Parameters,torque,area_minus,friction


def test_pairing_uses_common_angle_and_merges_float_centers():
    points=[]
    for direction,angles in [(1,[.1,.3,.5]),(-1,[.2,.4,.6])]:
        for i,q in enumerate(angles):
            points.append(dict(q=q,pressure=np.array([-10000.,q*10000+direction*1000]),
                               direction=direction,center=101.3+(1e-14 if i%2 else 0)))
    groups=pair_branches(points,'S2',count=7)
    assert len(groups)==1
    g=groups[0]
    np.testing.assert_allclose(g['q'][[0,-1]],[.2,.5])
    np.testing.assert_allclose((g['up'][:,1]+g['down'][:,1])/2,g['q']*10000)
    np.testing.assert_allclose((g['up'][:,1]-g['down'][:,1])/2,1000)


def test_single_chamber_pairs_are_not_split_by_changing_center():
    points=[dict(q=q,pressure=np.array([-1000.,q*10000+d*100]),direction=d,
                 center=101+q) for d in (1,-1) for q in (.1,.2,.3)]
    assert len(pair_branches(points,'S1a'))==1


def test_pairing_does_not_extrapolate_nonoverlapping_branches():
    points=[dict(q=q,pressure=np.array([-1000.,1000.]),direction=d,center=101.3)
            for d,qs in [(1,[.1,.2]),(-1,[.3,.4])] for q in qs]
    assert pair_branches(points,'S2')==[]


def test_radius_scan_updates_power_port_and_preserves_dissipation():
    q=np.array([.2,.7,1.2]);v=np.array([.1,-.2,.3]);P=np.tile([-20000.,30000.],(3,1))
    for radius in (.02,.025,.03):
        p=dataclasses.replace(Parameters(),reel_radius_m=radius)
        tau=torque(q,P,p)
        y=np.column_stack((-radius*area_minus(p.x1_zero_m-radius*q,p)*v,radius*p.area_plus*v))
        np.testing.assert_allclose(np.sum(P*y,axis=1),tau*v,atol=1e-14)
        assert np.all(v*friction(v,tau,p)>=0)


def test_force_normalized_loss_does_not_reward_smaller_torque_scale():
    p=dataclasses.replace(Parameters(),elastic_k_nm_rad=1.,elastic_bias_nm=-.2)
    q=np.array([.2,.5,.8]);P=np.column_stack((np.zeros(3),np.array([10000.,20000.,30000.])))
    groups=[dict(q=q,up=P,down=P)]
    scaled=dataclasses.replace(p,reel_radius_m=.8*p.reel_radius_m,gravity_nm=.8*p.gravity_nm,
                               elastic_k_nm_rad=.8*p.elastic_k_nm_rad,elastic_bias_nm=.8*p.elastic_bias_nm)
    np.testing.assert_allclose(central_residual(p,groups),central_residual(scaled,groups),atol=1e-13)
