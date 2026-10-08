"""Smooth, interpretable pH residual slots; pure offline float64 inference.

Fixed tanh feature neurons have zero-initialized linear output heads. This is a
single-actuator basis fit, NOT a learned multi-actuator shared representation.
All actuator feature coordinates are chamber lengths. No pressure in energy.
"""
import ctypes
import dataclasses
import json
from pathlib import Path
import subprocess
import tempfile
import numpy as np
from .nominal import Parameters, area_minus, torque, gradient, friction, energy


def zero_model():
    base=Path('reports/residual_actuator4_20261007/motion_refit_v4/nominal_selected.json')
    p=json.loads(base.read_text())['nominal'] if base.exists() else dataclasses.asdict(Parameters(x1_zero_m=.05))
    return dict(schema='structured_ph_v6',nominal=p,area_coefficients=[[0.]*4,[0.]*4],
        potential_coefficients=[0.]*4,mu_coefficients=[0.,0.],history_k=[0.,0.],
        history_tau_s=[10.,100.],history_rho=[1.,1.],
        feature_reference=dict(x1_center_m=p['x1_zero_m']-p['reel_radius_m']*.65,
            x2_center_m=p['x2_zero_m']+p['reel_radius_m']*.65,
            area_scale_m=p['reel_radius_m']*.35,
            potential_centers_m=[p['x2_zero_m']+p['reel_radius_m']*q for q in (.2,.5,.8,1.1)],
            potential_scale_m=p['reel_radius_m']*.25))


class Model:
    def __init__(self,obj):
        self.obj=obj;self.p=Parameters(**obj['nominal']);self.f=obj['feature_reference']
        self.area=np.asarray(obj['area_coefficients'],dtype=float)
        self.c=np.asarray(obj['potential_coefficients'],dtype=float)
        self.mu=np.asarray(obj['mu_coefficients'],dtype=float)
        self.k=np.asarray(obj['history_k'],dtype=float)
        self.tau=np.asarray(obj['history_tau_s'],dtype=float)
        self.rho=np.asarray(obj['history_rho'],dtype=float)
        if self.area.shape!=(2,4) or self.c.shape!=(4,) or any(x.shape!=(2,) for x in (self.mu,self.k,self.tau,self.rho)):
            raise ValueError('Invalid structured coefficient shape')
        values=np.r_[self.area.ravel(),self.c,self.mu,self.k,self.tau,self.rho]
        if not np.all(np.isfinite(values)) or np.any(np.r_[self.mu,self.k,self.rho]<0) or np.any(self.tau<=0):
            raise ValueError('Invalid passive coefficients')
        if np.any(np.sum(np.abs(self.area),axis=1)>=.95):
            raise ValueError('Area correction must preserve positive area globally')

    def area_features(self,q,P):
        x=np.array([self.p.x1_zero_m-self.p.reel_radius_m*q,self.p.x2_zero_m+self.p.reel_radius_m*q])
        z=(x-np.array([self.f['x1_center_m'],self.f['x2_center_m']]))/self.f['area_scale_m']
        return np.column_stack((np.ones(2),np.tanh(z),np.tanh(2*z),np.tanh(np.abs(P)/50000.)))

    def port(self,q,P):
        A=np.array([area_minus(self.p.x1_zero_m-self.p.reel_radius_m*q,self.p),self.p.area_plus])
        A*=1+np.sum(self.area*self.area_features(q,P),axis=1)
        return self.p.reel_radius_m*A*np.array([-1.,1.])

    def potential(self,q):
        z=(self.p.x2_zero_m+self.p.reel_radius_m*q-np.asarray(self.f['potential_centers_m']))/self.f['potential_scale_m']
        return float(self.c@np.tanh(z))

    def potential_gradient(self,q):
        z=(self.p.x2_zero_m+self.p.reel_radius_m*q-np.asarray(self.f['potential_centers_m']))/self.f['potential_scale_m']
        return float(self.c@(1-np.tanh(z)**2)*self.p.reel_radius_m/self.f['potential_scale_m'])

    def potential_torch(self,q):
        import torch
        centers=torch.as_tensor(self.f['potential_centers_m'],dtype=q.dtype,device=q.device)
        c=torch.as_tensor(self.c,dtype=q.dtype,device=q.device)
        return c@torch.tanh((self.p.x2_zero_m+self.p.reel_radius_m*q-centers)/self.f['potential_scale_m'])

    def forces(self,z,P):
        q,v=z[:2];xi=z[2:];fh=self.k*(q-xi)
        # Nominal friction remains tied to nominal tau_A, as in main.tex.
        extra_mu=self.mu@(np.logaddexp(0,np.abs(P)/50000.)-np.log(2.))
        f=float(friction(v,torque(q,P,self.p),self.p)+extra_mu*np.tanh(v/self.p.epsilon_rad_s))
        rate=np.where(self.k>0,1/self.tau+self.rho*self.k*abs(v),0.)
        return fh,f,rate

    def rhs(self,z,P):
        fh,f,rate=self.forces(z,P);q,v=z[:2]
        return np.r_[v,(self.port(q,P)@P-gradient(q,self.p)-self.potential_gradient(q)-fh.sum()-f)/self.p.inertia_kg_m2,
                     rate*(q-z[2:])]

    def energy(self,z):
        return float(energy(z[0],z[1],self.p)+self.potential(z[0])+.5*np.sum(self.k*(z[0]-z[2:])**2))

    def audit(self,z,P):
        q,v=z[:2];fh,f,rate=self.forces(z,P);rhs=self.rhs(z,P)
        supply=float(P@(self.port(q,P)*v))
        diss=float(v*f+np.sum(rate*self.k*(q-z[2:])**2))
        hdot=float(self.p.inertia_kg_m2*v*rhs[1]+(gradient(q,self.p)+self.potential_gradient(q)+fh.sum())*v-fh@rhs[2:])
        return dict(supply=supply,dissipation=diss,hdot=hdot,
            power_error=float(supply-(self.port(q,P)@P)*v),balance_error=hdot-supply+diss)


_LIB=None


def simulate(b,obj,substeps=5):
    global _LIB
    m=Model(obj)
    if int(substeps)!=substeps or substeps<1:raise ValueError('Invalid substeps')
    t=np.asarray(b['t']);np.testing.assert_allclose(np.diff(t),.1,atol=1e-7,rtol=0)
    P=np.ascontiguousarray(b['P'],dtype=np.float64)
    if len(t)<2 or P.shape!=(len(t),2) or not np.all(np.isfinite(P)):raise ValueError('Invalid input')
    if _LIB is None:
        src=Path(__file__).with_suffix('.cpp');binary=Path(tempfile.mkdtemp(prefix='ph_structured_'))/'solver.so'
        subprocess.run(['g++','-O3','-std=c++17','-shared','-fPIC',str(src),'-o',str(binary)],check=True)
        _LIB=ctypes.CDLL(str(binary));arr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
        _LIB.rollout.argtypes=[ctypes.c_int,ctypes.c_int]+[arr]*5;_LIB.rollout.restype=ctypes.c_int
    p=m.p
    params=np.array([getattr(p,k) for k in ('diameter_m','fold_length_m','folds','reel_radius_m','gravity_nm',
        'inertia_kg_m2','x1_zero_m','alpha','damping_nm_s_rad','epsilon_rad_s','elastic_k_nm_rad',
        'elastic_bias_nm','q_min_rad','q_max_rad','limit_k_nm_rad','x2_zero_m')]+
        [m.f['x1_center_m'],m.f['x2_center_m'],m.f['area_scale_m'],*m.f['potential_centers_m'],m.f['potential_scale_m']],dtype=np.float64)
    coeff=np.r_[m.area.ravel(),m.c,m.mu,m.k,m.tau,m.rho].astype(np.float64)
    initial=np.array([b['qs'][0],b['v'][0],*b.get('xi0',[b['qs'][0]]*2)],dtype=np.float64)
    out=np.empty((len(t),4),dtype=np.float64)
    code=_LIB.rollout(len(t),int(substeps),P,initial,params,coeff,out)
    if code or not np.all(np.isfinite(out)):raise ValueError(f'Structured offline solver failed: {code}')
    return out
