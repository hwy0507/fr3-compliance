#pragma once
// Fixed-size port of ParameterTeacher + StableReadoutTeacher + SixDVirtualDynamics.
// Profile: special_scene_vmc_readout06_20261028.json. No robot/network API.
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fr3_contact {
using V3 = Eigen::Vector3d;
using V6 = Eigen::Matrix<double,6,1>;
using V7 = Eigen::Matrix<double,7,1>;
using Obs = Eigen::Matrix<double,48,1>;
using J67 = Eigen::Matrix<double,6,7>;
using M6 = Eigen::Matrix<double,6,6>;
using M7 = Eigen::Matrix<double,7,7>;
inline double clip(double v,double lo,double hi) {return std::clamp(v,lo,hi);}
template<class T> inline T norm_cap(T v,double cap) {
  const double n=v.norm(); if(n>cap)v*=cap/n; return v;
}

// Generated from the original JSON and resolved original Python defaults.
#include "profile_generated.inc"

class ActiveVmc {
 public:
  V6 offset=V6::Zero(), velocity=V6::Zero(), filtered=V6::Zero();
  V6 previous_wrench=V6::Zero();
  double recovery_memory=0;
  V7 step(const Obs& obs,double dt=0.04) {
    if(!obs.allFinite()||!std::isfinite(dt)||dt<=0||dt>0.04)
      throw std::invalid_argument("invalid VMC observation/dt");
    const V6 wrench=obs.segment<6>(39);
    const V6 effective=(wrench+previous_wrench)*0.5;
    previous_wrench=wrench;
    filtered+=(1-std::exp(-dt/kFilterTau))*(effective-filtered);
    V6 w;
    for(int i=0;i<6;++i) {
      const double dead=std::copysign(std::max(0.0,std::abs(filtered[i])-kDeadband[i]),filtered[i]);
      w[i]=kSigma[i]*std::tanh(dead/kSigma[i]);
    }
    for(int sub=0;sub<20;++sub)for(int i=0;i<6;++i) {
      const double spring=kSigma[i]*std::tanh(kStiffness[i]*offset[i]/kSigma[i]);
      velocity[i]=clip(velocity[i]+(w[i]-kDamping[i]*velocity[i]-spring)/kMass[i]*(dt/20),-kVmax[i],kVmax[i]);
      offset[i]=clip(offset[i]+velocity[i]*(dt/20),-kXmax[i],kXmax[i]);
    }
    V7 action=V7::Zero();
    for(int i=0;i<6;++i) action[i+1]=clip((kVelocityWeight*velocity[i]+kOffsetGain*offset[i])/kActionLimits[i],-1,1);
    action[0]=clip(action.tail<6>().norm()/std::sqrt(6.0),0,1);
    const Eigen::Vector2d path=obs.segment<2>(45);
    const double horizontal=std::min(1.0,path.norm());
    const Eigen::Vector2d direction=path/std::max(path.norm(),1e-12);
    const V3 command=obs.segment<3>(14),actual=command-obs.segment<3>(26);
    const double desired=command.head<2>().dot(direction), progress=actual.head<2>().dot(direction);
    const double opposition=std::max(0.0,-wrench.head<2>().dot(direction)-kRecoveryThreshold);
    const double evidence=clip(opposition,0,1)*clip((desired-progress)/.015,0,1)*horizontal*horizontal;
    const double rising=evidence/kRecoveryAttack;
    const double intent=clip(desired/.015,0,1)*horizontal*horizontal;
    const double falling=std::max(0.0,progress)/kRecoveryDistance+(1-intent)/kRecoveryRelease;
    const double eq=rising/std::max(rising+falling,1e-12);
    recovery_memory+=(1-std::exp(-dt*(rising+falling)))*(eq-recovery_memory);
    const double activation=recovery_memory*horizontal*horizontal;
    const double correction=clip(kRecoveryGain*(kRecoveryOffset+obs[22])-command[2],-kRecoveryVelocity,kRecoveryVelocity);
    action[3]=(1-activation)*action[3]+activation*correction/.32;
    for(int i=0;i<7;++i)action[i]=clip(action[i],-1,1);
    if(!action.allFinite()||!offset.allFinite()||!velocity.allFinite()||!std::isfinite(recovery_memory))
      throw std::runtime_error("VMC numerical failure");
    return action;
  }
};
} // namespace fr3_contact
