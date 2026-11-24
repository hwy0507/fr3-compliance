#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace esn_cycle {
using Q=std::array<double,7>;
struct Reference {
 Q home,target,start;
 double leg;
 Reference(Q h,Q p,Q s,double speed):home(h),target(p),start(s),leg(0) {
  if(!std::isfinite(speed)||speed<=0||speed>.15)throw std::runtime_error("invalid cycle speed");
  double md=0;
  for(int j=0;j<7;++j){
   if(!std::isfinite(h[j])||!std::isfinite(p[j])||!std::isfinite(s[j])||std::abs(s[j]-h[j])>.02)throw std::runtime_error("cycle start mismatch");
   md=std::max(md,std::abs(p[j]-h[j]));
  }
  leg=std::acos(-1.)*md/(2*speed);
  if(leg<2||2*leg>85)throw std::runtime_error("invalid cycle duration");
 }
 double duration()const{return 2*leg;}
 Q at(double tau,int derivative=0)const{
  tau=std::clamp(tau,0.,duration());
  if(derivative<0||derivative>3)throw std::runtime_error("invalid derivative");
  const double w=std::acos(-1.)/leg;
  const double sh[4]={.5*(1-std::cos(w*tau)),.5*w*std::sin(w*tau),.5*w*w*std::cos(w*tau),-.5*w*w*w*std::sin(w*tau)};
  const double u=std::clamp(tau,0.,1.);
  double fade[4]={1-10*u*u*u+15*u*u*u*u-6*u*u*u*u*u,-30*u*u+60*u*u*u-30*u*u*u*u,-60*u+180*u*u-120*u*u*u,-60+360*u-360*u*u};
  if(tau>=1)for(auto& v:fade)v=0;
  Q q{};
  for(int j=0;j<7;++j)q[j]=(derivative==0?home[j]:0)+(target[j]-home[j])*sh[derivative]+(start[j]-home[j])*fade[derivative];
  return q;
 }
};
}
