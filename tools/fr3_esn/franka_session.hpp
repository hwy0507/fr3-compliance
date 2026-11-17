#pragma once
#include "esn_core.hpp"
#include "../fr3_contact/reviewed_session.hpp"
namespace fr3_esn {
// These calibration values MUST be supported by physical evidence. Zeros are not a calibration.
struct ObserverCalibration {
  bool reviewed=false;
  V7 friction_magnitude=V7::Constant(std::numeric_limits<double>::quiet_NaN());
  V7 passive_damping=V7::Constant(std::numeric_limits<double>::quiet_NaN());
  void validate() const {
    if(!reviewed||!friction_magnitude.allFinite()||!passive_damping.allFinite()||
       (friction_magnitude.array()<0).any()||(passive_damping.array()<0).any())
      throw std::invalid_argument("observer friction/passive model unreviewed");
  }
};
struct EsnReview : SessionReview {
  bool observer_sign_rate_and_noise_verified=false;
  bool actor_world_frame_and_tcp_verified=false;
  bool unexplained_reflex_resolved=false;
  bool conservative_servo_transfer_reviewed=false;
  void validate() const {
    validate_session_review(*this);
    if(!observer_sign_rate_and_noise_verified||!actor_world_frame_and_tcp_verified||
       !unexplained_reflex_resolved||!conservative_servo_transfer_reviewed)
      throw std::invalid_argument("ESN sim2real qualification incomplete");
  }
};
struct TraceFrame {
  franka::RobotState robot_state{};
  M7 mass=M7::Zero();V7 gravity=V7::Zero(),passive=V7::Zero();bool observer_updated=false;
  EsnSample sample;Result result;Obs observation=Obs::Zero();
  Eigen::Matrix<double,64,1> reservoir=Eigen::Matrix<double,64,1>::Zero();
  double dt=0;size_t policy_updates=0;
};
struct Trace {
  std::vector<TraceFrame> frames;size_t used=0;bool control_entered=false;
  // Keep the full SDK ControlException (including its tail log) after control exits.
  std::exception_ptr failure;
  void prepare(double seconds) {
    if(!std::isfinite(seconds)||seconds<=0||seconds>30)throw std::invalid_argument("session duration");
    frames.resize(static_cast<size_t>(std::ceil(seconds*1000))+8);used=0;control_entered=false;failure=nullptr;
  }
};
inline EsnSample read_sample(franka::Robot& robot,franka::Model& model,
                            const franka::RobotState& s,const franka::RobotState& expected) {
  const auto j=model.zeroJacobian(franka::Frame::kEndEffector,s);
  const auto c=model.coriolis(s);
  const auto lo=robot.getLowerJointVelocityLimits(s.q),hi=robot.getUpperJointVelocityLimits(s.q);
  EsnSample out;
  static_cast<Sample&>(out)=convert_state(s,expected,Eigen::Map<const J67>(j.data()),
      Eigen::Map<const V7>(c.data()),Eigen::Map<const V7>(lo.data()),Eigen::Map<const V7>(hi.data()));
  return out;
}
// Library integration seam only; NO live CLI, no invocation in tests.
// Every call is a new explicitly reviewed session, with a freshly reset ESN/observer.
// No gripper, recovery, collision setter, payload setter, or automatic zero return.
inline void run_reviewed_esn_hold(franka::Robot& robot,franka::Model& model,
  const franka::RobotState& expected,const Eigen::Matrix3d& world_R_base,
  const ObserverCalibration& calibration,Limits limits,const EsnReview& review,
  std::atomic<bool>& cancel,Trace& trace) {
  review.validate();calibration.validate();limits.validate();trace.prepare(limits.session_timeout);
  if(cancel.load(std::memory_order_relaxed))throw ContactAbort(Fault::OperatorStop);
  const auto initial=robot.readOnce();
  if(initial.robot_mode!=franka::RobotMode::kIdle||!same_tool(initial,expected)||
     (Eigen::Map<const V7>(initial.q.data())-Eigen::Map<const V7>(expected.q.data())).cwiseAbs().maxCoeff()>.003)
    throw ContactAbort(Fault::Startup);
  EsnCore core(hold_reference(initial),Eigen::Map<const V7>(initial.q.data()),world_R_base,limits);
  // Preflight on independent SDK force/moment; a zeroed observer cannot hide a bias.
  auto sample=read_sample(robot,model,initial,expected);
  EsnCore preflight=core;const auto check=preflight.step(sample,0);
  if(!check.valid)throw ContactAbort(check.fault);
  NativeObserver observer(Eigen::Map<const V7>(initial.dq.data()),calibration.friction_magnitude);
  bool first=true;double last_time=initial.time.toSec();
  if(cancel.load(std::memory_order_relaxed))throw ContactAbort(Fault::OperatorStop);
  try {robot.control([&](const franka::RobotState& state,franka::Duration period)->franka::Torques {
    trace.control_entered=true;
    if(trace.used>=trace.frames.size())throw ContactAbort(Fault::Timeout);
    auto& row=trace.frames[trace.used++]; // include failing callback, already allocated
    row.dt=period.toSec();row.robot_state=state;
    try {
      row.sample=read_sample(robot,model,state,expected);
      row.sample.stop_requested=cancel.load(std::memory_order_relaxed);
      if(!first)row.sample.mode_ok=state.robot_mode==franka::RobotMode::kMove;
      if(first) {
        if(row.dt!=0||(row.sample.q-Eigen::Map<const V7>(initial.q.data())).norm()>.005)
          throw ContactAbort(Fault::Startup);
        observer=NativeObserver(row.sample.dq,calibration.friction_magnitude);
      } else {
        if(row.dt<=0||row.dt>limits.max_period||std::abs(state.time.toSec()-last_time-row.dt)>1e-6)
          throw ContactAbort(Fault::Timing);
        const auto mass=model.mass(state);const auto g=model.gravity(state);
        row.mass=Eigen::Map<const M7>(mass.data());row.gravity=Eigen::Map<const V7>(g.data());
        row.passive=-calibration.passive_damping.cwiseProduct(row.sample.dq);
        row.sample.policy_load=observer.update(row.sample.dq,row.mass,
          row.gravity+row.sample.coriolis,row.passive,row.sample.tau_measured,row.dt);
        row.observer_updated=true;
        row.sample.policy_wrench=equivalent_wrench(row.sample.jacobian,row.sample.policy_load);
      }
      row.result=core.step(row.sample,row.dt);
      row.observation=core.observation();row.reservoir=core.reservoir();row.policy_updates=core.policy_updates();
      first=false;last_time=state.time.toSec();
      if(!row.result.valid)throw ContactAbort(row.result.fault);
      std::array<double,7> tau{};Eigen::Map<V7>(tau.data())=row.result.torque;
      return franka::Torques(tau); // excludes gravity/friction; SDK adds them
    } catch(const ContactAbort& e) {row.result.valid=false;row.result.fault=e.reason;throw;}
      catch(...) {row.result.valid=false;row.result.fault=Fault::Numerical;throw;}
  },true); // retain libfranka limiter + default filter, never retry after exception
  } catch(...) {trace.failure=std::current_exception();throw;} // outside realtime callback
}
} // namespace fr3_esn
