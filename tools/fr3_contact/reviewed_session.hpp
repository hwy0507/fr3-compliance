#pragma once
// Explicit integration function for a future authorized hold test. Never invoked by offline checks.
#include "franka_adapter.hpp"
#include <vector>

namespace fr3_contact {
struct SessionReview {
  // Defaults deliberately do not grant permission. Fill only after a concrete reviewed test plan.
  bool motion_authorized=false;
  bool tool_and_force_frame_verified=false;
  bool application_envelope_reviewed=false;
  bool existing_robot_protection_reviewed=false;
  bool workspace_and_operator_confirmed=false;
  bool realtime_and_stopping_reviewed=false;
  bool ready() const noexcept {
    return motion_authorized&&tool_and_force_frame_verified&&application_envelope_reviewed&&
      existing_robot_protection_reviewed&&workspace_and_operator_confirmed&&realtime_and_stopping_reviewed;
  }
};
struct TraceFrame {Sample sample;Result result;double period=0;};
struct SessionTrace {
  std::vector<TraceFrame> frames;
  size_t used=0;
  bool control_entered=false;
  // Allocated before control. No file I/O/resize/string formatting in the callback.
  void prepare(double seconds) {
    if(!std::isfinite(seconds)||seconds<=0||seconds>30)throw std::invalid_argument("session length outside reviewed implementation");
    frames.resize(static_cast<size_t>(std::ceil(seconds*1000))+8);
    used=0;control_entered=false;
  }
};
inline void validate_session_review(const SessionReview& review) {
  if(!review.ready())throw std::invalid_argument("hardware review/authorization incomplete: no control allowed");
}
inline void run_reviewed_hold(franka::Robot& robot,franka::Model& model,
                              const franka::RobotState& reviewed_initial,
                              const Eigen::Matrix3d& world_R_base,Limits limits,
                              const SessionReview& review,std::atomic<bool>& cancel,SessionTrace& trace) {
  validate_session_review(review); // BEFORE even the read; false is the shipped state.
  limits.validate();trace.prepare(limits.session_timeout);
  if(cancel.load(std::memory_order_relaxed))throw ContactAbort(Fault::OperatorStop);
  // Fresh state is checked against the specifically reviewed starting pose/tool.
  const auto fresh=robot.readOnce();
  if(!same_tool(fresh,reviewed_initial)||
     (Eigen::Map<const V7>(fresh.q.data())-Eigen::Map<const V7>(reviewed_initial.q.data())).cwiseAbs().maxCoeff()>.003)
    throw ContactAbort(Fault::Startup);
  auto preflight=ContactCore(hold_reference(reviewed_initial),Eigen::Map<const V7>(reviewed_initial.q.data()),world_R_base,limits);
  const auto j=model.zeroJacobian(franka::Frame::kEndEffector,fresh);
  const auto c=model.coriolis(fresh);
  const auto lo=robot.getLowerJointVelocityLimits(fresh.q),hi=robot.getUpperJointVelocityLimits(fresh.q);
  const auto sample=convert_state(fresh,reviewed_initial,Eigen::Map<const J67>(j.data()),Eigen::Map<const V7>(c.data()),
    Eigen::Map<const V7>(lo.data()),Eigen::Map<const V7>(hi.data()));
  const auto checked=preflight.step(sample,0);
  if(!checked.valid||fresh.robot_mode!=franka::RobotMode::kIdle)throw ContactAbort(checked.valid?Fault::Startup:checked.fault);
  FrankaHoldAdapter adapter(robot,model,fresh,world_R_base,limits);
  if(cancel.load(std::memory_order_relaxed))throw ContactAbort(Fault::OperatorStop);
  robot.control([&](const franka::RobotState& s,franka::Duration dt)->franka::Torques {
    trace.control_entered=true;
    if(trace.used>=trace.frames.size())throw ContactAbort(Fault::Timeout);
    adapter.stop_requested.store(cancel.load(std::memory_order_relaxed),std::memory_order_relaxed);
    const size_t before=adapter.callback_count;
    try {
      const auto command=adapter(s,dt);
      trace.frames[trace.used++]={adapter.last_sample,adapter.last,dt.toSec()};
      return command;
    } catch(...) {
      if(adapter.callback_count!=before)trace.frames[trace.used++]={adapter.last_sample,adapter.last,dt.toSec()};
      throw; // no retry, clear-error, zero-torque fallback, or parameter setter
    }
  },true); // Keep libfranka rate limiting and default low-pass filter.
}
} // namespace fr3_contact
