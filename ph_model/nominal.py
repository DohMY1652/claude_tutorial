"""main.tex nominal equations only; q [rad], velocity [rad/s], P=[P1,P2] [Pa].

The optional quadratic V_e is a declared sensitivity model, NOT a residual NN.
Numerical ODE integration is not itself a discrete passivity proof.
"""
from dataclasses import dataclass, fields
from pathlib import Path
import numpy as np
import yaml


@dataclass(frozen=True)
class Parameters:
    diameter_m: float = .05
    fold_length_m: float = .006
    folds: int = 14
    reel_radius_m: float = .025
    atmosphere_kpa: float = 101.325
    gravity_nm: float = 2.943
    inertia_kg_m2: float = .045
    x1_zero_m: float = .084
    x2_zero_m: float = .084
    alpha: float = .134
    damping_nm_s_rad: float = .1
    epsilon_rad_s: float = .02
    elastic_k_nm_rad: float = 0.
    elastic_bias_nm: float = 0.
    q_min_rad: float = -0.08726646259971647
    q_max_rad: float = 1.53588974175501
    limit_k_nm_rad: float = 100.

    def __post_init__(self):
        vals = [getattr(self, f.name) for f in fields(self)]
        if not np.all(np.isfinite(vals)):
            raise ValueError('Non-finite nominal parameter')
        if min(self.diameter_m, self.fold_length_m, self.folds, self.reel_radius_m,
               self.inertia_kg_m2, self.epsilon_rad_s, self.limit_k_nm_rad) <= 0:
            raise ValueError('Geometry, inertia, epsilon and limit stiffness must be positive')
        if min(self.alpha, self.damping_nm_s_rad, self.elastic_k_nm_rad) < 0:
            raise ValueError('Dissipation and quadratic elastic stiffness must be nonnegative')

    @property
    def area_plus(self):
        return np.pi * self.diameter_m**2 / 4

    @classmethod
    def load(cls, path=None):
        path = Path(path) if path else Path(__file__).with_name('params.yaml')
        return cls(**yaml.safe_load(path.read_text()))


def area_minus(x, p):
    s = np.asarray(x, dtype=np.float64)/(2*p.folds*p.fold_length_m)
    if np.any((s < 0) | (s >= 1)):
        raise ValueError('Bellows geometry outside 0 <= x/(2 n L0) < 1; no clipping')
    c = np.sqrt(1-s*s)
    ds = p.diameter_m - p.fold_length_m*c/3
    S = np.sqrt((ds/2)**2 + (p.fold_length_m*s/np.pi)**2)
    return p.area_plus - np.pi*p.fold_length_m*((1-2*s*s)/c*S +
        p.fold_length_m*s*s/S*(ds/12+p.fold_length_m*c/np.pi**2))


def torque(q, pressure, p):
    pressure = np.asarray(pressure, dtype=np.float64)
    a1 = area_minus(p.x1_zero_m-p.reel_radius_m*np.asarray(q), p)
    return p.reel_radius_m*(-a1*pressure[..., 0]+p.area_plus*pressure[..., 1])


def friction(v, tau, p):
    return p.alpha*np.abs(tau)*np.tanh(np.asarray(v)/p.epsilon_rad_s)+p.damping_nm_s_rad*np.asarray(v)


def gradient(q, p):
    q = np.asarray(q)
    return p.gravity_nm*np.sin(q)+p.elastic_k_nm_rad*q+p.elastic_bias_nm + \
        p.limit_k_nm_rad*(np.maximum(q-p.q_max_rad, 0)-np.maximum(p.q_min_rad-q, 0))


def energy(q, v, p):
    q = np.asarray(q)
    return .5*p.inertia_kg_m2*np.asarray(v)**2-p.gravity_nm*np.cos(q) + \
        .5*p.elastic_k_nm_rad*q*q+p.elastic_bias_nm*q + \
        .5*p.limit_k_nm_rad*(np.maximum(q-p.q_max_rad, 0)**2+np.maximum(p.q_min_rad-q, 0)**2)


def rhs(z, pressure, p):
    q, v = z
    tau = torque(q, pressure, p)
    return [v, (tau-gradient(q, p)-friction(v, tau, p))/p.inertia_kg_m2]
