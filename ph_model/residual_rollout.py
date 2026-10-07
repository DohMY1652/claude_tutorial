"""Fixed-nominal, NN output-layer short-rollout refinement.

Hidden layers are learned by the preceding equation stage. Here only the
potential output layer and nonnegative dissipation output coefficients change.
History parameters, when present, are fixed during this ablation.
"""
import copy
import numpy as np
from scipy.optimize import least_squares
from .nominal import torque,gradient,friction
from .residual import NumpyResidual,PassiveResidual


def prepare(wins,dt=.05):
    grid=np.arange(0,10+dt/2,dt)
    P=np.array([np.column_stack([np.interp(grid,w['t']-w['t'][0],w['pressure'][:,j]) for j in (0,1)]) for w in wins])
    return dict(P=P,z0=np.array([w['z0'] for w in wins]),
                xi0=np.array([w.get('xi0',[w['z0'][0]]*2) for w in wins]),
                truth=np.array([w['q'] for w in wins]),dt=dt)


def batch_rollout(prepared,obj):
    n=NumpyResidual(obj);p=n.nominal;dt=prepared['dt'];P=prepared['P']
    z=prepared['z0'].copy();xi=prepared['xi0'].copy()
    out=np.empty((len(z),P.shape[1]));out[:,0]=z[:,0]
    for i in range(P.shape[1]-1):
        pressure=(P[:,i]+P[:,i+1])/2
        q0,v0=z[:,0].copy(),z[:,1].copy()
        def force(vm):
            qm=q0+.5*dt*vm
            tau=torque(qm,pressure,p)
            _,g,d,mu=n.slots_batch(qm,pressure)
            A=.5*dt*(n.rv[None,:]+n.rho[None,:]*np.abs(vm[:,None]))*n.k[None,:]
            fm=n.k[None,:]*(qm[:,None]-xi)/(1+A)
            total=tau-gradient(qm,p)-g-friction(vm,tau,p)-d*vm-mu*np.tanh(vm/p.epsilon_rad_s)-fm.sum(axis=1)
            return 2*(vm-v0)-dt*total/p.inertia_kg_m2,A
        vm=v0.copy();lo=np.full(len(z),-5.);hi=-lo
        for iteration in range(30):
            F,A=force(vm)
            if np.max(np.abs(F))<1e-8:break
            lo=np.where(F<0,vm,lo);hi=np.where(F>=0,vm,hi)
            h=1e-5
            deriv=(force(vm+h)[0]-force(vm-h)[0])/(2*h)
            proposal=vm-F/deriv
            update=np.where((proposal>lo)&(proposal<hi),proposal,(lo+hi)/2)
            vm=np.where(np.abs(F)<1e-8,vm,update)
        else:raise RuntimeError('Residual midpoint did not converge')
        qm=q0+.5*dt*vm
        xim=(xi+A*qm[:,None])/(1+A)
        xi=2*xim-xi
        z[:,0]=q0+dt*vm;z[:,1]=2*vm-v0
        out[:,i+1]=z[:,0]
    return out[:,::int(round(.1/dt))]


def refine(model,wins,max_nfev=25):
    original=copy.deepcopy(model.export());s=original['state']
    potential=np.r_[np.array(s['potential_net.4.weight']).ravel(),s['potential_net.4.bias']]
    m=len(potential)
    initial=np.r_[potential,s['d_mix'],s['mu_mix']]
    lo=np.r_[potential-2,np.zeros(8)];hi=np.r_[potential+2,np.full(4,10.),np.full(4,3.)]
    cache=prepare(wins)
    def decode(z):
        obj=copy.deepcopy(original)
        obj['state']['potential_net.4.weight']=[z[:m-1].tolist()]
        obj['state']['potential_net.4.bias']=[float(z[m-1])]
        obj['state']['d_mix']=z[m:m+4].tolist();obj['state']['mu_mix']=z[m+4:].tolist()
        return obj
    calls=0
    def fun(z):
        nonlocal calls
        pred=batch_rollout(cache,decode(z))
        error=(pred-cache['truth']).ravel()/np.sqrt(pred.size)
        calls+=1
        if calls%50==0:print('NN rollout objective',calls,'RMSE deg',np.rad2deg(np.linalg.norm(error)),flush=True)
        # A small anchor discourages large corrections and ill-conditioned feature cancellation.
        return np.r_[error,.0002*(z-initial)]
    # An absolute finite-difference stencil avoids near-zero relative steps.
    def jac(z):
        f0=fun(z);cols=[]
        for i in range(len(z)):
            h=1e-4 if z[i]+1e-4<=hi[i] else -1e-4
            zp=z.copy();zp[i]+=h
            cols.append((fun(zp)-f0)/h)
        return np.column_stack(cols)
    res=least_squares(fun,np.clip(initial,lo+1e-8,hi-1e-8),jac=jac,bounds=(lo,hi),
                      max_nfev=max_nfev,ftol=5e-4,xtol=5e-4,gtol=1e-5)
    return PassiveResidual.restore(decode(res.x)),dict(success=bool(res.success),nfev=res.nfev,
        objective_calls=calls,message=res.message,dt_s=.05,n_windows=len(wins),
        window_rmse_deg=float(np.rad2deg(np.linalg.norm(res.fun[:cache['truth'].size]))),
        parameters='potential output layer and nonnegative dissipation mixing; nominal and history fixed')
