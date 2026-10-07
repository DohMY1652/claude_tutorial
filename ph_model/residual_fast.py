"""Compiled accelerator for offline model fitting; same pH equations, no ROS."""
import ctypes
from pathlib import Path
import subprocess
import tempfile

import numpy as np

_LIB = None


def library():
    global _LIB
    if _LIB is None:
        src = Path(__file__).with_suffix('.cpp')
        # Private temporary build directory; no fixed shared-library path.
        directory = Path(tempfile.mkdtemp(prefix='ph_offline_solver_'))
        binary = directory/'solver.so'
        subprocess.run(['g++','-O3','-std=c++17','-shared','-fPIC',str(src),'-o',str(binary)],check=True)
        _LIB = ctypes.CDLL(str(binary))
        arr = np.ctypeslib.ndpointer(dtype=np.float64, flags='C_CONTIGUOUS')
        _LIB.rollout.argtypes = [ctypes.c_int,ctypes.c_int]+[arr]*7
        _LIB.rollout.restype = ctypes.c_int
    return _LIB


def simulate(b, obj, substeps=2):
    if substeps<1 or int(substeps)!=substeps: raise ValueError('Positive integer substeps required')
    if obj['width']!=16: raise ValueError('Accelerator supports width 16 only')
    t = np.asarray(b['t'])
    if t.ndim!=1 or len(t)<2 or not np.all(np.isfinite(t)):raise ValueError('Invalid time grid')
    np.testing.assert_allclose(np.diff(t),.1,atol=1e-7,rtol=0)
    p=obj['nominal'];s=obj['state']
    params=np.array([p[k] for k in ('diameter_m','fold_length_m','folds','reel_radius_m',
        'gravity_nm','inertia_kg_m2','x1_zero_m','alpha','damping_nm_s_rad','epsilon_rad_s',
        'elastic_k_nm_rad','elastic_bias_nm','q_min_rad','q_max_rad','limit_k_nm_rad')]
        +list(obj['q_bounds'])+list(obj['pressure_scale']),dtype=np.float64)
    coeff=np.concatenate([s[k] for k in ('d_mix','mu_mix','history_k','history_rv','history_rho')]).astype(np.float64)
    if coeff.shape!=(14,) or not np.all(np.isfinite(coeff)) or np.any(coeff<0): raise ValueError('Invalid passive coefficients')
    nets=[np.concatenate([np.asarray(s[f'{name}.{layer}.{kind}']).ravel()
        for layer in (0,2,4) for kind in ('weight','bias')]).astype(np.float64)
        for name in ('potential_net','dissipation_net')]
    if any(a.shape!=(size,) or not np.all(np.isfinite(a)) for a,size in zip(nets,(337,488))):
        raise ValueError('Invalid network dimensions or weights')
    P=np.ascontiguousarray(b['P'],dtype=np.float64)
    if P.shape!=(len(t),2) or not np.all(np.isfinite(P)):raise ValueError('Invalid pressure input')
    initial=np.array([b['qs'][0],b['v'][0],*b.get('xi0',[b['qs'][0]]*2)],dtype=np.float64)
    if initial.shape!=(4,) or not np.all(np.isfinite(initial)):raise ValueError('Invalid initial state')
    out=np.empty((len(t),4),dtype=np.float64)
    code=library().rollout(len(t),substeps,P,initial,params,coeff,*nets,out)
    if code:raise ValueError(f'Offline midpoint failed (geometry/nonfinite=1, solve=2): {code}')
    if not np.all(np.isfinite(out)):raise ValueError('Nonfinite solution')
    return out
