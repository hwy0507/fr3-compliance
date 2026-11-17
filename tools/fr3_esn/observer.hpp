#pragma once
#include "../fr3_contact/contact_core.hpp"
#include <Eigen/SVD>
namespace fr3_esn {
using namespace fr3_contact;
// Exact native_physics40 equations; 1kHz on hardware is a documented sample-rate change.
// motor_total is measured total joint torque, never FCI tau_J_d.
// No invented friction, passive model, bias removal, or SDK-wrench substitution.
class NativeObserver {
 public:
  NativeObserver(const V7& velocity,const V7& friction):previous_(velocity),friction_(friction) {
    if(!velocity.allFinite()||!friction.allFinite()||(friction.array()<0).any())throw std::invalid_argument("observer initialization");
  }
  V7 filtered=V7::Zero();
  V7 update(const V7& velocity,const M7& mass,const V7& bias,const V7& passive,const V7& motor_total,double dt) {
    if(!velocity.allFinite()||!mass.allFinite()||!bias.allFinite()||!passive.allFinite()||!motor_total.allFinite()||
       !std::isfinite(dt)||dt<=0||dt>.002)throw std::invalid_argument("observer packet");
    if((mass-mass.transpose()).norm()>1e-8||mass.llt().info()!=Eigen::Success)throw std::invalid_argument("invalid mass matrix");
    V7 residual=mass*((velocity-previous_)/dt)+bias-passive-motor_total;
    for(int i=0;i<7;++i) {
      const double motion=std::tanh(velocity[i]/.005);
      const double corrected=residual[i]+friction_[i]*motion;
      residual[i]=std::copysign(std::max(0.,std::abs(corrected)-friction_[i]*(1-std::abs(motion))),corrected);
    }
    previous_=velocity;filtered+=(-std::expm1(-dt/.04))*(residual-filtered);
    if(!filtered.allFinite())throw std::runtime_error("observer overflow");return filtered;
  }
 private: V7 previous_,friction_;
};
inline V6 equivalent_wrench(const J67& j,const V7& load) {
  if(!j.allFinite()||!load.allFinite())throw std::invalid_argument("invalid wrench inputs");
  const Eigen::Matrix<double,7,6> jt=j.transpose();
  Eigen::JacobiSVD<Eigen::Matrix<double,7,6>> svd(jt,Eigen::ComputeFullU|Eigen::ComputeFullV);
  svd.setThreshold(.03);return svd.solve(load); // numpy lstsq(J.T, load, rcond=.03)
}
} // namespace fr3_esn
