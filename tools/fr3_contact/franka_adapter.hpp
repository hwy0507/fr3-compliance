#pragma once
// Integration seam only: this file NEVER constructs Robot or starts robot.control.
// No robot setters, automatic recovery, Desk, Hand, or camera operations.
#include "contact_core.hpp"
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <exception>
#include <atomic>

namespace fr3_contact {
template<size_t N> inline bool same_array(const std::array<double,N>& a,const std::array<double,N>& b) {
  for(size_t i=0;i<N;++i)if(!std::isfinite(a[i])||!std::isfinite(b[i])||std::abs(a[i]-b[i])>1e-9)return false;
  return true;
}
template<size_t N> inline bool flags_valid(const std::array<double,N>& a) {
  for(double x:a)if(!std::isfinite(x)||x<0)return false;
  return true;
}
template<size_t N> inline bool any_flag(const std::array<double,N>& a) {
  for(double x:a)if(x!=0)return true;
  return false;
}
inline bool rigid(const Eigen::Matrix4d& t) {
  return t.allFinite()&&rotation_ok(t.topLeftCorner<3,3>())&&
    (t.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-9;
}
inline bool same_tool(const franka::RobotState& a,const franka::RobotState& b) {
  return std::isfinite(a.m_total)&&std::isfinite(a.m_ee)&&std::isfinite(a.m_load)&&
    std::isfinite(b.m_total)&&std::isfinite(b.m_ee)&&std::isfinite(b.m_load)&&
    std::abs(a.m_total-b.m_total)<1e-9&&std::abs(a.m_ee-b.m_ee)<1e-9&&std::abs(a.m_load-b.m_load)<1e-9&&
    same_array(a.F_T_EE,b.F_T_EE)&&same_array(a.EE_T_K,b.EE_T_K)&&
    same_array(a.F_T_NE,b.F_T_NE)&&same_array(a.NE_T_EE,b.NE_T_EE)&&
    same_array(a.F_x_Ctotal,b.F_x_Ctotal)&&same_array(a.I_total,b.I_total)&&
    same_array(a.F_x_Cee,b.F_x_Cee)&&same_array(a.F_x_Cload,b.F_x_Cload)&&
    same_array(a.I_ee,b.I_ee)&&same_array(a.I_load,b.I_load);
}
inline Reference hold_reference(const franka::RobotState& initial) {
  const Eigen::Map<const Eigen::Matrix4d> t(initial.O_T_EE.data());
  if(!rigid(t))throw std::invalid_argument("invalid EE pose");
  Reference r;r.position=t.topRightCorner<3,1>();r.rotation=t.topLeftCorner<3,3>();return r;
}
inline Sample convert_state(const franka::RobotState& s,const franka::RobotState& expected,
                            const J67& jacobian,const V7& coriolis,const V7& lower,const V7& upper) {
  Sample out;out.time=s.time.toSec();out.success_rate=s.control_command_success_rate;
  out.q=Eigen::Map<const V7>(s.q.data());out.dq=Eigen::Map<const V7>(s.dq.data());
  out.tau_measured=Eigen::Map<const V7>(s.tau_J.data());out.tau_previous=Eigen::Map<const V7>(s.tau_J_d.data());
  out.external_joint=Eigen::Map<const V7>(s.tau_ext_hat_filtered.data());
  out.jacobian=jacobian;out.coriolis=coriolis;out.velocity_lower=lower;out.velocity_upper=upper;
  const Eigen::Map<const Eigen::Matrix4d> t(s.O_T_EE.data()),ee_k(s.EE_T_K.data());
  const Eigen::Map<const Eigen::Matrix4d> f_ee(s.F_T_EE.data());
  out.position=t.topRightCorner<3,1>();out.rotation=t.topLeftCorner<3,3>();
  // Use the explicit K-origin, K-axis wrench. Do not interpret the O field's
  // moment as already being about K: our recorded firmware data includes p_O_K x F.
  // API sign is robot-on-environment; actor/guards require environment-on-robot.
  const V6 api_k=Eigen::Map<const V6>(s.K_F_ext_hat_K.data());
  const V6 api_o=Eigen::Map<const V6>(s.O_F_ext_hat_K.data());
  const Eigen::Matrix3d o_r_k=out.rotation*ee_k.topLeftCorner<3,3>();
  out.raw_wrench.head<3>()=-o_r_k*api_k.head<3>();
  out.raw_wrench.tail<3>()=-o_r_k*api_k.tail<3>(); // still about K, now O axes
  out.wrench=out.raw_wrench;
  const V3 arm=out.rotation*ee_k.topRightCorner<3,1>(); // O(p_K-p_EE)
  out.wrench.tail<3>()+=arm.cross(out.wrench.head<3>());
  // O moment is diagnostic only; its point convention is not inferred at runtime.
  // Still reject corrupt fields or inconsistent forces rather than silently hiding them.
  const bool force_agrees=api_k.allFinite()&&api_o.allFinite()&&
    (api_o.head<3>()+out.raw_wrench.head<3>()).norm()<=1e-4*(1+api_o.head<3>().norm());
  // No auto-zero or bias subtraction. Both K and EE moments remain guarded.
  out.configuration_ok=same_tool(s,expected)&&rigid(t)&&rigid(ee_k)&&rigid(f_ee)&&force_agrees;
  out.errors=bool(s.current_errors)||bool(s.last_motion_errors)||
    !flags_valid(s.joint_contact)||!flags_valid(s.cartesian_contact)||
    !flags_valid(s.joint_collision)||!flags_valid(s.cartesian_collision);
  out.collision=any_flag(s.joint_collision)||any_flag(s.cartesian_collision);
  out.contact=any_flag(s.joint_contact)||any_flag(s.cartesian_contact);
  out.mode_ok=s.robot_mode==franka::RobotMode::kIdle||s.robot_mode==franka::RobotMode::kMove;
  return out;
}
class ContactAbort final: public std::exception {
 public:
  explicit ContactAbort(Fault reason): reason(reason) {}
  Fault reason;
  const char* what() const noexcept override {return "bounded VMC stopped; inspect latched fault; no automatic retry";}
};
// The live launcher is deliberately absent pending reviewed physical limits and user authorization.
// In a future launcher, throwing from this callback cancels the FCI motion through libfranka.
// Do NOT catch and keep sending the last command, return zeros, or automatically recover/restart.
class FrankaHoldAdapter {
 public:
  FrankaHoldAdapter(franka::Robot& robot,franka::Model& model,const franka::RobotState& initial,
                    const Eigen::Matrix3d& world_R_base,Limits limits)
    :robot_(robot),model_(model),expected_(initial),core_(hold_reference(initial),
      Eigen::Map<const V7>(initial.q.data()),world_R_base,limits) {}
  franka::Torques operator()(const franka::RobotState& state,franka::Duration period) {
    // All arrays/model calculations are local. Never call a non-realtime setter in this callback.
    const auto j=model_.zeroJacobian(franka::Frame::kEndEffector,state);
    const auto c=model_.coriolis(state);
    const auto lo=robot_.getLowerJointVelocityLimits(state.q),hi=robot_.getUpperJointVelocityLimits(state.q);
    Sample sample=convert_state(state,expected_,Eigen::Map<const J67>(j.data()),Eigen::Map<const V7>(c.data()),
      Eigen::Map<const V7>(lo.data()),Eigen::Map<const V7>(hi.data()));
    if(callback_count>0)sample.mode_ok=state.robot_mode==franka::RobotMode::kMove;
    sample.stop_requested=stop_requested.load(std::memory_order_relaxed);
    last_sample=sample;last_period=period.toSec();++callback_count;
    last=core_.step(sample,period.toSec());
    if(!last.valid)throw ContactAbort(last.fault);
    std::array<double,7> tau{};Eigen::Map<V7>(tau.data())=last.torque;
    return franka::Torques(tau);
  }
  static_assert(std::atomic<bool>::is_always_lock_free,"stop flag must be lock free");
  std::atomic<bool> stop_requested{false};
  Result last;
  Sample last_sample;
  double last_period=0;
  size_t callback_count=0;
 private:
  franka::Robot& robot_;franka::Model& model_;franka::RobotState expected_;ContactCore core_;
};
} // namespace fr3_contact
