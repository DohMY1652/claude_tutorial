"""Passive residual slots, float64, fixed nominal; no ROS or hardware imports.

Potential is pressure-independent and globally bounded by +/-2 J. Its force is
computed by autograd during fitting. Dissipation is positive softplus features
times projected nonnegative output coefficients. That output layer is zero at
initialization (unlike softplus(0), which is not zero).
Two OPTIONAL constant-stiffness history elements are a subset of main.tex.
Their states follow xi_dot=(rv+rho*abs(v))*k*(q-xi), not a free RNN.
"""
import dataclasses
import numpy as np
import torch
from torch import nn
from scipy.special import expit

from .nominal import Parameters


class PassiveResidual(nn.Module):
    def __init__(self,nominal,q_bounds,pressure_scale,width=16):
        super().__init__()
        self.nominal=nominal
        self.q_bounds=tuple(q_bounds);self.pressure_scale=tuple(pressure_scale);self.width=width
        self.potential_net=nn.Sequential(nn.Linear(2,width),nn.SiLU(),nn.Linear(width,width),nn.SiLU(),nn.Linear(width,1))
        self.dissipation_net=nn.Sequential(nn.Linear(4,width),nn.SiLU(),nn.Linear(width,width),nn.SiLU(),nn.Linear(width,8))
        nn.init.zeros_(self.potential_net[-1].weight);nn.init.zeros_(self.potential_net[-1].bias)
        self.d_mix=nn.Parameter(torch.zeros(4))
        self.mu_mix=nn.Parameter(torch.zeros(4))
        self.history_k=nn.Parameter(torch.zeros(2),requires_grad=False)
        self.register_buffer('history_rv',torch.tensor([1e-4,1e-4]))
        self.register_buffer('history_rho',torch.tensor([.8,3.]))
        self.double()

    def features(self,q):
        # Explicit chamber displacements, normalized using training q range only.
        p=self.nominal;lo,hi=self.q_bounds
        x1=p.x1_zero_m-p.reel_radius_m*q;x2=p.x2_zero_m+p.reel_radius_m*q
        center1=p.x1_zero_m-p.reel_radius_m*(lo+hi)/2
        center2=p.x2_zero_m+p.reel_radius_m*(lo+hi)/2
        half=p.reel_radius_m*(hi-lo)/2
        return torch.stack(((x1-center1)/half,(x2-center2)/half),dim=-1)

    def slots(self,q,pressure,create_graph=True):
        if not q.requires_grad:q=q.detach().requires_grad_(True)
        x=self.features(q)
        V=2*torch.tanh(self.potential_net(x).squeeze(-1)/2)
        g=torch.autograd.grad(V.sum(),q,create_graph=create_graph,retain_graph=True)[0]
        scale=pressure.new_tensor(self.pressure_scale)
        h=nn.functional.softplus(self.dissipation_net(torch.cat((x,pressure.abs()/scale),dim=-1)))
        return V,g,(h[...,:4]*self.d_mix).sum(-1),(h[...,4:]*self.mu_mix).sum(-1)

    def project(self):
        with torch.no_grad():
            self.d_mix.clamp_(min=0);self.mu_mix.clamp_(min=0);self.history_k.clamp_(min=0)

    def export(self):
        return dict(nominal=dataclasses.asdict(self.nominal),q_bounds=self.q_bounds,
                    pressure_scale=self.pressure_scale,width=self.width,
                    state={k:v.detach().cpu().numpy().tolist() for k,v in self.state_dict().items()})

    @classmethod
    def restore(cls,obj):
        m=cls(Parameters(**obj['nominal']),obj['q_bounds'],obj['pressure_scale'],obj['width'])
        m.load_state_dict({k:torch.tensor(v,dtype=torch.float64) for k,v in obj['state'].items()})
        if min(float(m.d_mix.min()),float(m.mu_mix.min()),float(m.history_k.min()))<0:
            raise ValueError('Negative constrained residual coefficient')
        return m


class NumpyResidual:
    """Fast independent inference, analytically differentiated SiLU chain.

    Tests compare this derivative to torch autograd, including nonzero weights.
    """
    def __init__(self,obj):
        self.obj=obj;self.nominal=Parameters(**obj['nominal'])
        self.bounds=obj['q_bounds'];self.scale=np.asarray(obj['pressure_scale'])
        self.s={k:np.asarray(v,dtype=float) for k,v in obj['state'].items()}
        self.k=self.s['history_k'];self.rv=self.s['history_rv'];self.rho=self.s['history_rho']
        if any(np.any(self.s[k]<0) for k in ('d_mix','mu_mix','history_k','history_rv','history_rho')):
            raise ValueError('Nonpassive coefficient')

    def network(self,x,name,derivative=None):
        for layer in (0,2,4):
            W=self.s[f'{name}.{layer}.weight'];b=self.s[f'{name}.{layer}.bias']
            x=W@x+b
            if derivative is not None:derivative=W@derivative
            if layer!=4:
                sig=expit(x)
                if derivative is not None:derivative=derivative*(sig+x*sig*(1-sig))
                x=x*sig
        return x,derivative

    def slots(self,q,P):
        lo,hi=self.bounds;z=2*(q-(lo+hi)/2)/(hi-lo)
        x=np.array([-z,z]);dx=np.array([-2/(hi-lo),2/(hi-lo)])
        raw,dv=self.network(x,'potential_net',dx)
        tv=np.tanh(raw[0]/2)
        V=2*tv;g=(1-tv*tv)*dv[0]
        h,_=self.network(np.r_[x,np.abs(P)/self.scale],'dissipation_net')
        h=np.logaddexp(0,h)
        return V,g,float(h[:4]@self.s['d_mix']),float(h[4:]@self.s['mu_mix'])

    def slots_batch(self,q,P):
        q=np.asarray(q);lo,hi=self.bounds;z=2*(q-(lo+hi)/2)/(hi-lo)
        x=np.column_stack((-z,z))
        dx=np.broadcast_to([-2/(hi-lo),2/(hi-lo)],x.shape).copy()
        def net(x,name,dx=None):
            for layer in (0,2,4):
                W=self.s[f'{name}.{layer}.weight'];b=self.s[f'{name}.{layer}.bias']
                x=x@W.T+b
                if dx is not None:dx=dx@W.T
                if layer!=4:
                    sig=expit(x)
                    if dx is not None:dx=dx*(sig+x*sig*(1-sig))
                    x=x*sig
            return x,dx
        raw,dv=net(x,'potential_net',dx);tv=np.tanh(raw[:,0]/2)
        h,_=net(np.column_stack((x,np.abs(P)/self.scale)),'dissipation_net')
        h=np.logaddexp(0,h)
        return 2*tv,(1-tv*tv)*dv[:,0],h[:,:4]@self.s['d_mix'],h[:,4:]@self.s['mu_mix']
