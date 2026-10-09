// Offline implicit Euler integration of passive slots; no hardware interfaces.
#include <cmath>
#include <algorithm>
namespace {
double softplus(double x){return std::max(0.,x)+std::log1p(std::exp(-std::abs(x)));}
struct Model {
 const double* p; const double* c;
 bool force(double q,double v,const double* xi,double p1,double p2,double dt,double& F,double* A) const {
  double s=(p[6]-p[3]*q)/(2*p[2]*p[1]);if(s<0||s>=1)return false;
  double cc=std::sqrt(1-s*s),ds=p[0]-p[1]*cc/3,S=std::sqrt(ds*ds/4+std::pow(p[1]*s/M_PI,2));
  double ap=M_PI*p[0]*p[0]/4;
  double am=ap-M_PI*p[1]*((1-2*s*s)/cc*S+p[1]*s*s/S*(ds/12+p[1]*cc/(M_PI*M_PI)));
  double nominal=p[3]*(-am*p1+ap*p2),pres[2]={p1,p2},area[2]={am,ap};
  double x[2]={p[6]-p[3]*q,p[15]+p[3]*q};
  for(int i=0;i<2;i++){
   double z=(x[i]-p[16+i])/p[24+i];
   area[i]*=1+c[4*i]+c[4*i+1]*std::tanh(z)+c[4*i+2]*std::tanh(2*z)+c[4*i+3]*std::tanh(std::abs(pres[i])/50000.);
  }
  double tau=p[3]*(-area[0]*p1+area[1]*p2),g=0;
  for(int j=0;j<4;j++){double tt=std::tanh((x[1]-p[19+j])/p[23]);g+=c[8+j]*(1-tt*tt)*p[3]/p[23];}
  double mu=0;for(int j=0;j<2;j++)mu+=c[12+j]*(softplus(std::abs(pres[j])/50000.)-std::log(2.));
  double fh=0;
  for(int j=0;j<2;j++){
   double k=c[14+j],multiplier=std::exp(c[20+2*j]*std::tanh(std::abs(p1)/50000.)+c[21+2*j]*std::tanh(std::abs(p2)/50000.));
   A[j]=k>0?dt*(1/c[16+j]+c[18+j]*k*std::abs(v))*multiplier:0.;fh+=k*(q-xi[j])/(1+A[j]);
  }
  double grad=p[4]*std::sin(q)+p[10]*q+p[11]+p[14]*(std::max(q-p[13],0.)-std::max(p[12]-q,0.));
  F=(tau-grad-g-(p[7]*std::abs(nominal)+mu)*std::tanh(v/p[9])-p[8]*v-fh)/p[5];
  return std::isfinite(F);
 }
};
}
int rollout_impl(int n,int substeps,const double* P,const double* initial,const double* p,const double* c,double* output,bool robust){
 Model m{p,c};double q=initial[0],v=initial[1],xi[2]={initial[2],initial[3]},dt=.1/substeps;
 for(int j=0;j<4;j++)output[j]=initial[j];
 for(int i=1;i<n;i++){
  for(int sub=0;sub<substeps;sub++){
   double f=(sub+1.)/substeps,p1=P[2*i-2]*(1-f)+P[2*i]*f,p2=P[2*i-1]*(1-f)+P[2*i+1]*f;
   double vm=v,lo=-10,hi=10,A[2],acc;bool converged=false;
   for(int iter=0;iter<(robust?200:80);iter++){
    if(!m.force(q+dt*vm,vm,xi,p1,p2,dt,acc,A))return 1;
    double res=vm-v-dt*acc;if(std::abs(res)<(robust?1e-9:1e-10)){converged=true;break;}
    if(res<0)lo=vm;else hi=vm;
    double fp,fm,B[2],h=std::min(1e-5,p[9]*.01);
    if(!m.force(q+dt*(vm+h),vm+h,xi,p1,p2,dt,fp,B)||!m.force(q+dt*(vm-h),vm-h,xi,p1,p2,dt,fm,B))return 1;
    double der=1-dt*(fp-fm)/(2*h),next=vm-res/der;
    vm=(next>lo&&next<hi&&std::isfinite(next)&&!(robust&&iter%4==3))?next:(lo+hi)/2;
   }
   if(!converged)return 2;
   q+=dt*vm;v=vm;for(int j=0;j<2;j++)xi[j]=(xi[j]+A[j]*q)/(1+A[j]);
  }
  output[4*i]=q;output[4*i+1]=v;output[4*i+2]=xi[0];output[4*i+3]=xi[1];
 }
 return 0;
}
extern "C" int rollout(int n,int substeps,const double* P,const double* initial,const double* p,const double* c,double* output){
 return rollout_impl(n,substeps,P,initial,p,c,output,false);
}
extern "C" int rollout_robust(int n,int substeps,const double* P,const double* initial,const double* p,const double* c,double* output){
 return rollout_impl(n,substeps,P,initial,p,c,output,true);
}
