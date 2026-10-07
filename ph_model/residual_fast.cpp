// Offline float64 implicit-midpoint model integrator; no hardware interfaces.
#include <cmath>
#include <algorithm>
#include <vector>

namespace {
struct Net {
    const double* w; int input, output;
    void eval(const double* x, const double* dx, double* y, double* dy) const {
        double a[16], da[16], b[16], db[16]; int off=0;
        for(int j=0;j<16;j++) {
            double z=0,dz=0;
            for(int i=0;i<input;i++){z+=w[j*input+i]*x[i];dz+=w[j*input+i]*dx[i];}
            z+=w[16*input+j]; double s=1/(1+std::exp(-z));
            a[j]=z*s;da[j]=dz*(s+z*s*(1-s));
        }
        off=16*input+16;
        for(int j=0;j<16;j++) {
            double z=0,dz=0;
            for(int i=0;i<16;i++){z+=w[off+j*16+i]*a[i];dz+=w[off+j*16+i]*da[i];}
            z+=w[off+256+j]; double s=1/(1+std::exp(-z));
            b[j]=z*s;db[j]=dz*(s+z*s*(1-s));
        }
        off+=272;
        for(int j=0;j<output;j++) {
            y[j]=w[off+output*16+j];dy[j]=0;
            for(int i=0;i<16;i++){y[j]+=w[off+j*16+i]*b[i];dy[j]+=w[off+j*16+i]*db[i];}
        }
    }
};
double softplus(double x){return std::max(0.,x)+std::log1p(std::exp(-std::abs(x)));}
struct Model {
    const double* p; const double* c; Net pot, diss;
    bool force(double q,double v,const double* xi,double p1,double p2,double dt,double &F,double* A) const {
        // p: D,L,n,r,Mg,J,x10,alpha,b,epsilon,kel,bias,qmin,qmax,klim,qlo,qhi,scale1,scale2
        double s=(p[6]-p[3]*q)/(2*p[2]*p[1]);
        if(s<0||s>=1) return false;
        double cc=std::sqrt(1-s*s), ds=p[0]-p[1]*cc/3;
        double S=std::sqrt(ds*ds/4+std::pow(p[1]*s/M_PI,2));
        double ap=M_PI*p[0]*p[0]/4;
        double am=ap-M_PI*p[1]*((1-2*s*s)/cc*S+p[1]*s*s/S*(ds/12+p[1]*cc/(M_PI*M_PI)));
        double tau=p[3]*(-am*p1+ap*p2);
        double z=2*(q-(p[15]+p[16])/2)/(p[16]-p[15]);
        double x[4]={-z,z,std::abs(p1)/p[17],std::abs(p2)/p[18]};
        double dx[4]={-2/(p[16]-p[15]),2/(p[16]-p[15]),0,0}, y[8],dy[8];
        pot.eval(x,dx,y,dy); double t=std::tanh(y[0]/2),g=(1-t*t)*dy[0];
        diss.eval(x,dx,y,dy);double d=0,mu=0;
        for(int j=0;j<4;j++){d+=c[j]*softplus(y[j]);mu+=c[j+4]*softplus(y[j+4]);}
        double fh=0;
        for(int j=0;j<2;j++){A[j]=dt/2*(c[10+j]+c[12+j]*std::abs(v))*c[8+j];fh+=c[8+j]*(q-xi[j])/(1+A[j]);}
        double grad=p[4]*std::sin(q)+p[10]*q+p[11]+p[14]*(std::max(q-p[13],0.)-std::max(p[12]-q,0.));
        F=(tau-grad-g-(p[7]*std::abs(tau)+mu)*std::tanh(v/p[9])-(p[8]+d)*v-fh)/p[5];
        return std::isfinite(F);
    }
};
}

extern "C" int rollout(int count,int substeps,const double* pressure,const double* initial,
                       const double* params,const double* coefficients,const double* potential,
                       const double* dissipation,double* output) {
    Model m{params,coefficients,Net{potential,2,1},Net{dissipation,4,8}};
    double q=initial[0],v=initial[1],xi[2]={initial[2],initial[3]},dt=.1/substeps;
    output[0]=q;output[1]=v;output[2]=xi[0];output[3]=xi[1];
    for(int i=1;i<count;i++) {
        for(int sub=0;sub<substeps;sub++) {
            double fraction=(sub+.5)/substeps;
            double p1=pressure[2*(i-1)]*(1-fraction)+pressure[2*i]*fraction;
            double p2=pressure[2*(i-1)+1]*(1-fraction)+pressure[2*i+1]*fraction;
            double vm=v,lo=-10,hi=10,A[2],f=0;
            bool converged=false;
            for(int iter=0;iter<50;iter++) {
                if(!m.force(q+.5*dt*vm,vm,xi,p1,p2,dt,f,A))return 1;
                double residual=2*(vm-v)-dt*f;
                if(std::abs(residual)<1e-10){converged=true;break;}
                if(residual<0)lo=vm;else hi=vm;
                double fp,fm,B[2],h=1e-5;
                if(!m.force(q+.5*dt*(vm+h),vm+h,xi,p1,p2,dt,fp,B))return 1;
                if(!m.force(q+.5*dt*(vm-h),vm-h,xi,p1,p2,dt,fm,B))return 1;
                double der=2-dt*(fp-fm)/(2*h),next=vm-residual/der;
                vm=(next>lo&&next<hi&&std::isfinite(next))?next:(lo+hi)/2;
            }
            if(!converged)return 2;
            double qm=q+.5*dt*vm;
            for(int j=0;j<2;j++)xi[j]=2*(xi[j]+A[j]*qm)/(1+A[j])-xi[j];
            q+=dt*vm;v=2*vm-v;
        }
        output[4*i]=q;output[4*i+1]=v;output[4*i+2]=xi[0];output[4*i+3]=xi[1];
    }
    return 0;
}
