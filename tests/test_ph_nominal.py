"""Offline mathematical tests, not tests of hardware control code."""
import dataclasses

import numpy as np
import pytest
from scipy.integrate import solve_ivp

from ph_model.nominal import Parameters, area_minus, torque, friction, gradient, energy, rhs
from ph_model.data import gauge_pressures, differentiate


def test_original_paper_geometry_regression():
    p = dataclasses.replace(Parameters(), diameter_m=0.0545)
    phi = np.deg2rad([0, 10, 30, 45, 50])
    x = 2 * p.folds * p.fold_length_m * np.sin(phi)
    np.testing.assert_allclose(area_minus(x, p) * 1e6, [1838, 1860, 2040, 2323, 2456], atol=3)


def test_units_and_pressure_order():
    np.testing.assert_allclose(gauge_pressures([110], [80]), [[-21325, 8675]])


def test_power_and_dissipation():
    p = Parameters()
    rng = np.random.default_rng(42)
    q = rng.uniform(0, 1.4, 100)
    v = rng.uniform(-1, 1, 100)
    pressure = np.column_stack((-rng.uniform(0, 60000, 100), rng.uniform(0, 80000, 100)))
    a1 = area_minus(p.x1_zero_m - p.reel_radius_m*q, p)
    port = np.column_stack((-p.reel_radius_m*a1*v, p.reel_radius_m*p.area_plus*v))
    tau = torque(q, pressure, p)
    np.testing.assert_allclose(np.sum(pressure*port, axis=1), tau*v, atol=1e-14)
    assert np.all(v*friction(v, tau, p) >= 0)
    acc = (tau - gradient(q, p) - friction(v, tau, p))/p.inertia_kg_m2
    np.testing.assert_allclose(p.inertia_kg_m2*v*acc + gradient(q, p)*v,
                               np.sum(pressure*port, axis=1)-v*friction(v, tau, p), atol=1e-13)


def test_unforced_energy_decreases():
    p = dataclasses.replace(Parameters(), damping_nm_s_rad=0.1, elastic_k_nm_rad=0.2,
                            elastic_bias_nm=-0.1)
    t = np.linspace(0, 3, 1500)
    sol = solve_ivp(lambda t, z: rhs(z, np.zeros(2), p), [0, 3], [0.5, 0.1],
                    t_eval=t, rtol=1e-9, atol=1e-11, max_step=0.005)
    assert sol.success
    assert np.max(np.diff(energy(sol.y[0], sol.y[1], p))) < 1e-8


def test_potential_gradient_and_geometry_domain():
    p = dataclasses.replace(Parameters(), elastic_k_nm_rad=0.3, elastic_bias_nm=-0.2)
    q = np.array([-0.2, 0.5, 1.6])
    h = 1e-6
    np.testing.assert_allclose((energy(q+h, 0, p)-energy(q-h, 0, p))/(2*h), gradient(q, p), atol=1e-7)
    with pytest.raises(ValueError):
        area_minus(0.2, p)


def test_filter_derivatives_polynomial():
    t = np.arange(0, 3, .01)
    q = 0.2*t*t + 0.3*t
    qs, v, a = differentiate(q, .01, .31)
    np.testing.assert_allclose(v[20:-20], (0.4*t+0.3)[20:-20], atol=1e-10)
    np.testing.assert_allclose(a[20:-20], .4, atol=1e-9)


def test_x2_zero_does_not_affect_nominal():
    p = Parameters()
    p2 = dataclasses.replace(p, x2_zero_m=.12)
    assert torque(.5, np.array([-30000., 20000.]), p) == torque(.5, np.array([-30000., 20000.]), p2)


def test_nominal_parameter_recovery_from_exact_offline_equations():
    from ph_model.fit_nominal import fit_equation
    p=dataclasses.replace(Parameters(),x1_zero_m=.105,alpha=.24,
                          damping_nm_s_rad=1.3,gravity_nm=3.3)
    rng=np.random.default_rng(17)
    q=rng.uniform(.1,1.4,300)
    v=rng.uniform(-.3,.3,300)
    pressure=np.column_stack((-rng.uniform(1000,60000,300),rng.uniform(1000,80000,300)))
    tau=torque(q,pressure,p)
    a=(tau-gradient(q,p)-friction(v,tau,p))/p.inertia_kg_m2
    data=np.column_stack((q,v,a,pressure))
    fit,info=fit_equation(data,np.ones(len(q))/np.sqrt(len(q)),Parameters(),starts=2)
    for name in info['fitted_names']:
        assert getattr(fit,name)==pytest.approx(getattr(p,name),rel=1e-5,abs=1e-7)


def test_scalar_rollout_matches_nominal_rhs():
    from ph_model.fit_nominal import simulate
    p=Parameters()
    t=np.arange(0,2,.01)
    pressure=np.tile([-20000.,10000.],(len(t),1))
    z0=[.4,.01]
    predicted=simulate(t,pressure,z0,p,rtol=1e-8,max_step=.005)
    direct=solve_ivp(lambda tt,z:rhs(z,pressure[0],p),[t[0],t[-1]],z0,t_eval=t,
                     rtol=1e-9,atol=1e-11,max_step=.005)
    np.testing.assert_allclose(predicted,direct.y.T,atol=1e-6)
