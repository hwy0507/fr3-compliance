#pragma once
// Native Cartesian impedance proposal. Pure guard + explicitly reviewed session.
// No Robot constructor, live CLI, auto-recovery, collision setter or VMC instance.
#include "../fr3_contact/franka_adapter.hpp"
#include "../fr3_wrench_trace.hpp"
#include <franka/rate_limiting.h>
#include <atomic>
#include <chrono>
#include <functional>
#include <exception>
#include <optional>

namespace fr3_native {
using fr3_contact::V3;using fr3_contact::V6;using fr3_contact::V7;
using fr3_contact::J67;using fr3_contact::Sample;
struct Proposal {
  bool operator_ended=false;
  std::array<double,6> stiffness{300,300,800,50,50,50}; // K-frame axes
  double measured_linear=.05,measured_angular=.2,measured_joint=.15;
  double displacement=.015,vertical_displacement=.005,angle=.05235987756;
  double joint_displacement=.17,joint_margin=.17;
  double force_stop=6,moment_stop=1.5,delta_force_stop=4,delta_moment_stop=.25,delta_vertical_stop=1;
  double startup_force=2,startup_moment=1,force_on=1,force_off=.5;
  double settle_time=3,release_dwell=.25,contact_timeout=2,session_timeout=15;
  double max_period=.002,min_success=.99,communication_startup_grace=.1,min_singular=.025;
  static void validate_stiffness(const std::array<double,6>& k) {
    for(size_t i=0;i<6;++i)if(!std::isfinite(k[i])||k[i]<(i<3?10:1)||k[i]>(i<3?3000:300))
      throw std::invalid_argument("stiffness outside native API range");
  }
  void validate() const {
    validate_stiffness(stiffness);
    if(stiffness!=std::array<double,6>{300,300,800,50,50,50})throw std::invalid_argument("only frozen trial stiffness supported");
    const double positive[]={measured_linear,measured_angular,measured_joint,displacement,vertical_displacement,
      angle,joint_displacement,joint_margin,force_stop,moment_stop,delta_force_stop,delta_moment_stop,
      delta_vertical_stop,startup_force,startup_moment,force_on,force_off,settle_time,release_dwell,
      contact_timeout,session_timeout,max_period,min_success,communication_startup_grace,min_singular};
    for(double x:positive)if(!std::isfinite(x)||x<=0)throw std::invalid_argument("invalid native test limit");
    if(force_off>=force_on||force_on>=delta_force_stop||startup_force>=force_stop||startup_moment>=moment_stop||
       delta_force_stop>force_stop||delta_moment_stop>moment_stop||settle_time>=session_timeout||
       session_timeout>60||contact_timeout>2||max_period>.005||min_success>1||min_success<.90||
       displacement>.015||vertical_displacement>displacement||angle>.05236||measured_linear>.05||
       measured_angular>.2||measured_joint>.15||communication_startup_grace>.1||
       force_stop>6||moment_stop>1.5||delta_force_stop>4||delta_moment_stop>1||delta_vertical_stop>1||
       startup_force>2||startup_moment>1||joint_displacement>.17||joint_margin<.17||min_singular<.025)
      throw std::invalid_argument("native proposal exceeds implemented test envelope");
  }
};
struct Baseline {
  V6 raw=V6::Zero(),ee=V6::Zero();
  V7 q=V7::Zero();V3 position=V3::Zero();Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
  bool qualified=false;
  // Explicitly reviewed static means in O axes, at K and EE respectively.
  // Used for contact recognition only; absolute guards never subtract these.
};
inline Baseline qualify_static_baseline(const std::vector<Sample>& samples,
                                      const std::vector<franka::RobotMode>& modes,Proposal p={}) {
  p.validate();if(samples.size()<900||modes.size()!=samples.size()||samples.back().time-samples.front().time<.9)
    throw std::invalid_argument("need at least 0.9s/900 complete static samples");
  Baseline b;b.q=samples.front().q;b.position=samples.front().position;b.rotation=samples.front().rotation;
  for(size_t i=0;i<samples.size();++i) {
    const auto& s=samples[i];
    if(modes[i]!=franka::RobotMode::kIdle||!std::isfinite(s.time)||!s.raw_wrench.allFinite()||!s.wrench.allFinite()||!s.q.allFinite()||!s.dq.allFinite()||
       !s.position.allFinite()||!fr3_contact::rotation_ok(s.rotation)||!s.jacobian.allFinite()||
       s.errors||s.contact||s.collision||!s.mode_ok||!s.configuration_ok||s.dq.cwiseAbs().maxCoeff()>.005||
       (s.q-samples.front().q).cwiseAbs().maxCoeff()>.001||
       (i&&(s.time<=samples[i-1].time||s.time-samples[i-1].time>.003)))
      throw std::invalid_argument("invalid, moving or contacted baseline");
    for(const V6* w:{&s.raw_wrench,&s.wrench})
      if(w->head<3>().norm()>p.startup_force||w->tail<3>().norm()>p.startup_moment)
        throw std::invalid_argument("absolute baseline above startup bound");
    b.raw+=s.raw_wrench;b.ee+=s.wrench;
  }
  b.raw/=samples.size();b.ee/=samples.size();
  for(const auto& s:samples)if((s.raw_wrench-b.raw).head<3>().norm()>.2||
      (s.raw_wrench-b.raw).tail<3>().norm()>.1||(s.wrench-b.ee).head<3>().norm()>.2||
      (s.wrench-b.ee).tail<3>().norm()>.1)throw std::invalid_argument("baseline unstable");
  b.qualified=true;return b;
}
enum class Fault {None,Nonfinite,Robot,Configuration,Startup,Timing,Bounds,Speed,Wrench,Singularity,
                  ContactTimeout,SessionTimeout,OperatorStop};
enum class Phase {Settling,Ready,Contact,Recovering,Finished,Stopped};
struct Result {bool valid=false,finished=false;Fault fault=Fault::None;Phase phase=Phase::Stopped;double elapsed=0;
  const char* reason="none";};
struct Abort final:std::exception {
  explicit Abort(Fault value):fault(value){}Fault fault;
  const char* what() const noexcept override{return "native hold stopped; inspect fault and trace; no automatic retry";}
};
class Guard {
 public:
  Guard(const Sample& anchor,const Baseline& baseline,const Eigen::Matrix3d& o_r_k,Proposal proposal={},bool guiding=false)
    :a_(anchor),b_(baseline),r_(o_r_k),p_(proposal),recovery_position_(anchor.position),recovery_rotation_(anchor.rotation),guiding_(guiding) {
    p_.validate();
    if(!b_.qualified||!fr3_contact::rotation_ok(r_)||!b_.raw.allFinite()||!b_.ee.allFinite()||
       !b_.q.allFinite()||!b_.position.allFinite()||!fr3_contact::rotation_ok(b_.rotation)||
       (anchor.q-b_.q).cwiseAbs().maxCoeff()>.001||(anchor.position-b_.position).norm()>.001||
       fr3_contact::rotation_error(anchor.rotation,b_.rotation).norm()>.005||
       b_.raw.head<3>().norm()>p_.startup_force||b_.raw.tail<3>().norm()>p_.startup_moment||
       b_.ee.head<3>().norm()>p_.startup_force||b_.ee.tail<3>().norm()>p_.startup_moment)
      throw std::invalid_argument("reviewed baseline/frame invalid or above startup bound");
  }
  void guiding_release(const Sample& sample) {
    if(!guiding_||(phase_!=Phase::Contact&&phase_!=Phase::Recovering)||!sample.position.allFinite()||
       !fr3_contact::rotation_ok(sample.rotation)||(sample.position-a_.position).norm()>p_.displacement||
       std::abs((r_.transpose()*(sample.position-a_.position))[2])>p_.vertical_displacement||
       fr3_contact::rotation_error(sample.rotation,a_.rotation).norm()>p_.angle)throw Abort(Fault::Bounds);
    recovery_position_=sample.position;recovery_rotation_=sample.rotation;
    // Only the released holding target changes. ALL global bounds and raw force
    // baseline remain tied to the original actual anchor, never re-anchored.
  }
  Result step(const Sample& s,double dt) {
    if(fault_!=Fault::None)return stop(fault_);
    if(!std::isfinite(dt)||!std::isfinite(s.time)||!std::isfinite(s.success_rate)||!s.q.allFinite()||
       !s.dq.allFinite()||!s.position.allFinite()||!fr3_contact::rotation_ok(s.rotation)||
       !s.wrench.allFinite()||!s.raw_wrench.allFinite()||!s.jacobian.allFinite()||
       !s.velocity_lower.allFinite()||!s.velocity_upper.allFinite()||!s.tau_measured.allFinite()||
       !s.external_joint.allFinite())return stop(Fault::Nonfinite);
    if(s.stop_requested)return stop(Fault::OperatorStop);
    if(s.errors||s.collision||!s.mode_ok)return stop(Fault::Robot);
    if(!s.configuration_ok)return stop(Fault::Configuration);
    if(!started_) {
      if(dt!=0||s.contact||s.dq.cwiseAbs().maxCoeff()>.005||(s.q-a_.q).cwiseAbs().maxCoeff()>.001||
         (s.position-a_.position).norm()>.001||fr3_contact::rotation_error(s.rotation,a_.rotation).norm()>.005)
        return stop(Fault::Startup);
    }else {
      if(dt<=0||dt>p_.max_period||s.time<=last_time_||std::abs(s.time-last_time_-dt)>1e-6)return stop(Fault::Timing);
      elapsed_+=dt;
      if(elapsed_>=p_.communication_startup_grace&&s.success_rate+1e-7<p_.min_success)return stop(Fault::Timing);
    }
    last_time_=s.time;
    const V7 lo=(V7()<<-2.9007,-1.8361,-2.9007,-3.077,-2.8763,.4398,-3.0508).finished();
    const V7 hi=(V7()<<2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508).finished();
    const V3 offset=s.position-a_.position;const double angle=fr3_contact::rotation_error(s.rotation,a_.rotation).norm();
    if((s.q.array()<lo.array()+p_.joint_margin).any()||(s.q.array()>hi.array()-p_.joint_margin).any()||
       (s.q-a_.q).cwiseAbs().maxCoeff()>p_.joint_displacement||offset.norm()>p_.displacement||
       std::abs((r_.transpose()*offset)[2])>p_.vertical_displacement||angle>p_.angle||
       (s.velocity_lower.array()>0).any()||(s.velocity_upper.array()<0).any())return stop(Fault::Bounds);
    const V6 twist=s.jacobian*s.dq;
    if(s.dq.cwiseAbs().maxCoeff()>p_.measured_joint||twist.head<3>().norm()>p_.measured_linear||
       twist.tail<3>().norm()>p_.measured_angular||(s.dq.array()<s.velocity_lower.array()).any()||
       (s.dq.array()>s.velocity_upper.array()).any())return stop(Fault::Speed);
    const V7 measured=(V7()<<60,60,60,60,9,9,9).finished(),external=(V7()<<5,5,5,5,2,2,2).finished();
    if((s.tau_measured.cwiseAbs().array()>measured.array()).any())return stop(Fault::Wrench,"measured_joint_torque");
    if((s.external_joint.cwiseAbs().array()>external.array()).any())return stop(Fault::Wrench,"external_joint_torque");
    // Independent absolute checks on BOTH points; baseline cannot mask overload.
    for(const V6* w:{&s.raw_wrench,&s.wrench})
      if(w->head<3>().norm()>p_.force_stop)return stop(Fault::Wrench,w==&s.raw_wrench?"absolute_K_force":"absolute_EE_force");
      else if(w->tail<3>().norm()>p_.moment_stop)return stop(Fault::Wrench,w==&s.raw_wrench?"absolute_K_moment":"absolute_EE_moment");
    const V6 delta=s.raw_wrench-b_.raw,delta_ee=s.wrench-b_.ee;
    if(delta.head<3>().norm()>p_.delta_force_stop)return stop(Fault::Wrench,"relative_K_force");
    if(delta_ee.head<3>().norm()>p_.delta_force_stop)return stop(Fault::Wrench,"relative_EE_force");
    if(delta.tail<3>().norm()>p_.delta_moment_stop)return stop(Fault::Wrench,"relative_K_moment");
    if(delta_ee.tail<3>().norm()>p_.delta_moment_stop)return stop(Fault::Wrench,"relative_EE_moment");
    if(std::abs((r_.transpose()*delta.head<3>())[2])>p_.delta_vertical_stop)return stop(Fault::Wrench,"relative_vertical_force");
    Eigen::SelfAdjointEigenSolver<fr3_contact::M6> eig(s.jacobian*s.jacobian.transpose(),Eigen::EigenvaluesOnly);
    if(eig.info()!=Eigen::Success||eig.eigenvalues().minCoeff()<p_.min_singular*p_.min_singular)return stop(Fault::Singularity);
    if(!started_) {
      for(const V6* w:{&s.raw_wrench,&s.wrench})
        if(w->head<3>().norm()>p_.startup_force||w->tail<3>().norm()>p_.startup_moment)return stop(Fault::Startup);
      if(delta.head<3>().norm()>.2||delta.tail<3>().norm()>.1)return stop(Fault::Startup);
      started_=true;
    }
    const bool contact=s.contact||delta.head<3>().norm()>p_.force_on||delta_ee.head<3>().norm()>p_.force_on;
    if(elapsed_<p_.settle_time) {
      if(contact)return stop(Fault::Startup);
      phase_=Phase::Settling;
    }else {
      if(!armed_){
        if(offset.norm()>.003||angle>.01||twist.head<3>().norm()>.005||s.dq.cwiseAbs().maxCoeff()>.02)
          return stop(Fault::Startup);
        armed_=true;phase_=Phase::Ready;
      }
      if(contact) {if(!contact_started_){contact_started_=true;contact_begin_=elapsed_;}phase_=Phase::Contact;release_=0;}
      else if(phase_==Phase::Contact) {
        if(delta.head<3>().norm()<p_.force_off&&delta_ee.head<3>().norm()<p_.force_off)release_+=dt;else release_=0;
        if(release_>=p_.release_dwell)phase_=Phase::Recovering;
      }
      const bool settled=(s.position-recovery_position_).norm()<.002&&
        fr3_contact::rotation_error(s.rotation,recovery_rotation_).norm()<.01&&
        twist.head<3>().norm()<.005&&s.dq.cwiseAbs().maxCoeff()<.02;
      if(phase_==Phase::Recovering&&settled){phase_=Phase::Ready;contact_started_=false;}
      if(contact_started_&&elapsed_-contact_begin_>=p_.contact_timeout&&!(guiding_&&p_.operator_ended))return stop(Fault::ContactTimeout);
      if(!p_.operator_ended&&elapsed_>=p_.session_timeout) {
        if(!settled||phase_!=Phase::Ready)return stop(Fault::SessionTimeout);
        phase_=Phase::Finished;return {true,true,Fault::None,phase_,elapsed_};
      }
    }
    return {true,false,Fault::None,phase_,elapsed_};
  }
 private:
  Result stop(Fault f,const char* reason=nullptr){if(fault_==Fault::None)fault_reason_=reason?reason:"other_guard";
    fault_=f;phase_=Phase::Stopped;return {false,false,f,phase_,elapsed_,fault_reason_};}
  const char* fault_reason_="none";
  Sample a_;Baseline b_;Eigen::Matrix3d r_;Proposal p_;Fault fault_=Fault::None;Phase phase_=Phase::Settling;
  V3 recovery_position_;Eigen::Matrix3d recovery_rotation_;bool guiding_=false;
  bool started_=false,armed_=false,contact_started_=false;double last_time_=0,elapsed_=0,contact_begin_=0,release_=0;
};
enum class EndPolicy {RestoreOriginal,RetainTrial};
struct Review {
  EndPolicy end_policy=EndPolicy::RestoreOriginal;
  bool retain_trial_authorized=false,collision_settings_preserved=false,collision_values_unverified=false;
  bool tool_configuration_and_static_reviewed=false,host_realtime_and_stop_reviewed=false;
  bool static_baseline_conditions_reviewed=false;
  bool free_guiding_authorized=false;
  bool minimum_success_95_authorized=false;
  bool minimum_success_90_authorized=false;
  bool native_gap_5ms_reviewed=false;
  bool extended_session_reviewed=false;
  bool operator_ended_authorized=false;
  bool guiding_moment_05_reviewed=false;
  bool guiding_moment_10_reviewed=false;
  bool motion_authorized=false,stiffness_write_authorized=false,restore_authorized=false;
  bool previous_stiffness_verified=false,static_baseline_verified=false,tool_and_frames_verified=false;
  bool existing_collision_protection_verified=false,realtime_and_stopping_verified=false,onsite_ready=false;
  std::array<double,6> previous_stiffness{}; // no public getter: do NOT invent this value
  void validate() const {
    if(end_policy==EndPolicy::RetainTrial){
      if(!(motion_authorized&&stiffness_write_authorized&&static_baseline_conditions_reviewed&&onsite_ready&&
           retain_trial_authorized&&collision_settings_preserved&&collision_values_unverified&&
           tool_configuration_and_static_reviewed&&host_realtime_and_stop_reviewed))
        throw std::invalid_argument("retain-trial review/authorization incomplete");
      return; // Unknown original stiffness/thresholds stay UNKNOWN, never labelled verified.
    }
    if(!(motion_authorized&&stiffness_write_authorized&&restore_authorized&&previous_stiffness_verified&&
         static_baseline_verified&&tool_and_frames_verified&&existing_collision_protection_verified&&
         realtime_and_stopping_verified&&onsite_ready))throw std::invalid_argument("native hold review/authorization incomplete");
    Proposal::validate_stiffness(previous_stiffness);
  }
};
struct Session {
  fr3_trace::Trace trace{64010};std::atomic<int> phase{int(Phase::Stopped)};
  std::atomic<bool> active{false},contact_enabled{false};
  std::atomic<unsigned long long> heartbeat{0};
  std::vector<double> callback_us=std::vector<double>(64010);
  size_t callback_time_count=0;
  size_t callback_total_count=0;double callback_total_us=0,callback_max_us=0;
  bool stiffness_write_attempted=false,stiffness_applied=false,control_entered=false,restore_attempted=false,restore_succeeded=false;
  bool trial_stiffness_retained=false;
  bool free_guiding_requested=false;
  Fault fault=Fault::None;
  const char* fault_reason="none";
  std::exception_ptr restoration_failure;
  // A UI/supervisor may read phase atomically; no printing or file I/O in callback.
};
inline void enforce_operator_contact(const Result& result,Session& session) {
  if(result.valid&&(result.phase==Phase::Contact||result.phase==Phase::Recovering)&&
     !session.contact_enabled.load(std::memory_order_relaxed)){
    session.fault=Fault::OperatorStop;session.phase.store(int(Phase::Stopped));
    throw Abort(Fault::OperatorStop);
  }
}
namespace detail {
template<class Setter,class Body>
void stiffness_transaction(Setter set,Body body,const Review& review,const Proposal& proposal,Session& session) {
  review.validate();proposal.validate();
  if(session.stiffness_write_attempted)throw std::invalid_argument("transaction cannot be reused");
  std::exception_ptr primary;
  try{
    session.stiffness_write_attempted=true;set(proposal.stiffness);session.stiffness_applied=true;body();
  }catch(const Abort& e){session.fault=e.fault;session.phase.store(int(Phase::Stopped));primary=std::current_exception();}
   catch(...){session.phase.store(int(Phase::Stopped));primary=std::current_exception();}
  if(review.end_policy==EndPolicy::RetainTrial){
    // Explicit user-selected outcome: no restoration or second setter. If the
    // first acknowledgement was lost, retain no false claim of a known setting.
    session.trial_stiffness_retained=session.stiffness_applied;
  }else{
    // Restore after unwinding, including an uncertain first setter acknowledgement.
    session.restore_attempted=true;
    try{set(review.previous_stiffness);session.restore_succeeded=true;}
    catch(...){session.restoration_failure=std::current_exception();if(!primary)primary=session.restoration_failure;}
  }
  if(primary)std::rethrow_exception(primary);
}
} // namespace detail
inline Sample convert(const franka::RobotState& s,const franka::RobotState& expected,franka::Robot& robot,franka::Model& model) {
  const auto j=model.zeroJacobian(franka::Frame::kEndEffector,s);
  const auto c=model.coriolis(s);
  const auto lo=robot.getLowerJointVelocityLimits(s.q),hi=robot.getUpperJointVelocityLimits(s.q);
  return fr3_contact::convert_state(s,expected,Eigen::Map<const J67>(j.data()),Eigen::Map<const V7>(c.data()),
    Eigen::Map<const V7>(lo.data()),Eigen::Map<const V7>(hi.data()));
}
class FixedCommand {
 public:
  explicit FixedCommand(const Sample& anchor):anchor_q_(anchor.q) {}
  std::array<double,7> get(const franka::RobotState& s) {
    if(!latched_){
      const Eigen::Map<const V7> q(s.q_d.data()),v(s.dq_d.data()),a(s.ddq_d.data());
      if(!q.allFinite()||!v.allFinite()||!a.allFinite()||(q-anchor_q_).cwiseAbs().maxCoeff()>.001||
         v.cwiseAbs().maxCoeff()>.001||a.cwiseAbs().maxCoeff()>.001)throw Abort(Fault::Startup);
      frozen_=s.q_d;latched_=true;
    }
    return frozen_; // not re-anchored to a pushed pose; reference speed is zero
  }
 private:
  V7 anchor_q_;std::array<double,7> frozen_{};bool latched_=false;
};
class GuidedCommand {
 public:
  explicit GuidedCommand(const Sample& anchor):fixed_(anchor) {}
  struct Output {std::array<double,7> q;bool released=false;};
  Output get(const franka::RobotState& state,const Sample& sample,const Baseline& baseline,
             bool permitted,const std::array<double,7>& lo,const std::array<double,7>& hi) {
    if(!initialized_){goal_=fixed_.get(state);initialized_=true;return {goal_,false};}
    if(!sample.raw_wrench.allFinite()||!sample.wrench.allFinite()||!sample.q.allFinite())throw Abort(Fault::Nonfinite);
    const double force=(sample.raw_wrench-baseline.raw).head<3>().norm();
    const double ee_force=(sample.wrench-baseline.ee).head<3>().norm();
    bool released=false;
    if(permitted) {
      if(sample.contact||force>1||ee_force>1)dragging_=true;
      if(dragging_){
        for(size_t j=0;j<7;++j)goal_[j]=sample.q[j]; // direction comes from actual displacement, NOT estimated-force sign
        if(!sample.contact&&force<.5&&ee_force<.5){dragging_=false;released=true;}
      }
    }
    std::array<double,7> upper{},lower{},accel{},jerk{};
    for(size_t j=0;j<7;++j){upper[j]=std::min(.015,hi[j]);lower[j]=std::max(-.015,lo[j]);accel[j]=.2;jerk[j]=1;
      if(!std::isfinite(upper[j])||!std::isfinite(lower[j])||upper[j]<0||lower[j]>0)throw Abort(Fault::Bounds);}
    // Local reference shaping; SDK limit-rate remains enabled. The FCI lowpass
    // is disabled ONLY for this already rate-limited reference (API requirement).
    std::array<double,7> wanted{},command{};
    // Triple-pole critically damped reference dynamics (poles at -6/s).
    // A position step passed straight into a bang-bang jerk limiter can cycle
    // near its target; explicitly damping velocity AND acceleration avoids it.
    for(size_t j=0;j<7;++j){
      const double requested_jerk=std::clamp(216*(goal_[j]-state.q_d[j])-108*state.dq_d[j]-18*state.ddq_d[j],-1.,1.);
      wanted[j]=state.dq_d[j]+.001*(state.ddq_d[j]+.001*requested_jerk);
    }
    const auto velocity=franka::limitRate(upper,lower,accel,jerk,wanted,state.dq_d,state.ddq_d);
    for(size_t j=0;j<7;++j)command[j]=state.q_d[j]+.001*velocity[j];
    return {command,released};
  }
 private:
  FixedCommand fixed_;bool initialized_=false,dragging_=false;std::array<double,7> goal_{};
};
using LinkPoses=std::array<Eigen::Matrix4d,7>;
inline LinkPoses link_poses(franka::Model& model,const franka::RobotState& state){
  const std::array<franka::Frame,7> frames{franka::Frame::kJoint1,franka::Frame::kJoint2,franka::Frame::kJoint3,
    franka::Frame::kJoint4,franka::Frame::kJoint5,franka::Frame::kJoint6,franka::Frame::kJoint7};
  LinkPoses result;
  for(size_t j=0;j<7;++j){const auto pose=model.pose(frames[j],state);result[j]=Eigen::Map<const Eigen::Matrix4d>(pose.data());}
  return result;
}
inline bool guiding_links_within_bounds(const LinkPoses& initial,const LinkPoses& actual){
  for(size_t j=0;j<7;++j)if(!fr3_contact::rigid(initial[j])||!fr3_contact::rigid(actual[j])||
      (actual[j].topRightCorner<3,1>()-initial[j].topRightCorner<3,1>()).norm()>.02||
      fr3_contact::rotation_error(actual[j].topLeftCorner<3,3>(),initial[j].topLeftCorner<3,3>()).norm()>.0872664626)
    return false;
  return true;
}
inline void run_reviewed_native_hold(franka::Robot& robot,franka::Model& model,const franka::RobotState& expected,
                                    const Baseline& baseline,const Review& review,const Proposal& proposal,
                                    std::atomic<bool>& cancel,Session& session) {
  review.validate();proposal.validate(); // before ANY read, setter or control
  if(proposal.delta_moment_stop!=.25&&
     (!((proposal.delta_moment_stop==.5&&review.guiding_moment_05_reviewed)||
        (proposal.delta_moment_stop==1&&review.guiding_moment_10_reviewed))||!proposal.operator_ended||
      !session.free_guiding_requested||review.end_policy!=EndPolicy::RetainTrial))
    throw std::invalid_argument("0.5/1Nm relative-moment candidates require their own reviewed guiding profile");
  if(proposal.operator_ended&&(!review.operator_ended_authorized||!session.free_guiding_requested||review.end_policy!=EndPolicy::RetainTrial))
    throw std::invalid_argument("operator-ended mode requires explicit supervised native guiding authorization");
  if(proposal.session_timeout>15&&(!review.extended_session_reviewed||review.end_policy!=EndPolicy::RetainTrial))
    throw std::invalid_argument("extended coordination window requires native retain review");
  if(proposal.max_period>.002&&(!review.native_gap_5ms_reviewed||review.end_policy!=EndPolicy::RetainTrial))
    throw std::invalid_argument("5ms receive gap is restricted to reviewed low-speed native retain sessions");
  if(proposal.min_success<.99&&
     (review.end_policy!=EndPolicy::RetainTrial||
      (proposal.min_success>=.95?!review.minimum_success_95_authorized:!review.minimum_success_90_authorized)))
    throw std::invalid_argument("lower success floor requires its explicit native retain-profile authorization");
  if(session.free_guiding_requested&&(!review.free_guiding_authorized||review.end_policy!=EndPolicy::RetainTrial))
    throw std::invalid_argument("free guiding requires separate retain-policy authorization");
  if(session.stiffness_write_attempted||session.trace.size())throw std::invalid_argument("session cannot be reused");
  if(cancel.load())throw Abort(Fault::OperatorStop);
  const auto fresh=robot.readOnce();
  if(fresh.robot_mode!=franka::RobotMode::kIdle||!fr3_contact::same_tool(fresh,expected)||
     (Eigen::Map<const V7>(fresh.q.data())-Eigen::Map<const V7>(expected.q.data())).cwiseAbs().maxCoeff()>.003)
    throw Abort(Fault::Startup);
  auto anchor=convert(fresh,expected,robot,model);
  const Eigen::Map<const Eigen::Matrix4d> t(fresh.O_T_EE.data()),k(fresh.EE_T_K.data());
  const Eigen::Matrix3d r=t.topLeftCorner<3,3>()*k.topLeftCorner<3,3>();
  // First proposal is horizontal-only; K z must be close to vertical, either sign.
  if(std::abs(r.col(2).dot(V3::UnitZ()))<std::cos(.0872664626))throw Abort(Fault::Configuration);
  Guard check(anchor,baseline,r,proposal);const auto first_check=check.step(anchor,0);
  if(!first_check.valid)throw Abort(first_check.fault);
  const auto desired=model.pose(franka::Frame::kEndEffector,fresh.q_d,fresh.F_T_EE,fresh.EE_T_K);
  const Eigen::Map<const Eigen::Matrix4d> td(desired.data());
  if(!fr3_contact::rigid(td)||(td.topRightCorner<3,1>()-anchor.position).norm()>.001||
     fr3_contact::rotation_error(td.topLeftCorner<3,3>(),anchor.rotation).norm()>.005||
     (Eigen::Map<const V7>(fresh.q_d.data())-anchor.q).cwiseAbs().maxCoeff()>.001||
     Eigen::Map<const V7>(fresh.dq_d.data()).cwiseAbs().maxCoeff()>.001||
     Eigen::Map<const V7>(fresh.ddq_d.data()).cwiseAbs().maxCoeff()>.001)throw Abort(Fault::Startup);
  Guard guard(anchor,baseline,r,proposal,session.free_guiding_requested);FixedCommand reference(anchor);GuidedCommand guiding(anchor);
  const auto initial_links=session.free_guiding_requested?link_poses(model,fresh):LinkPoses{};
  if(cancel.load())throw Abort(Fault::OperatorStop);
  detail::stiffness_transaction([&](const std::array<double,6>& stiffness){robot.setCartesianImpedance(stiffness);},[&]{
    if(cancel.load())throw Abort(Fault::OperatorStop);
    struct ActiveScope {
      Session& session;
      ~ActiveScope(){session.active.store(false);session.contact_enabled.store(false);}
    } active_scope{session};
    robot.control([&](const franka::RobotState& s,franka::Duration dt)->franka::JointPositions {
      struct CallbackTimer {
        Session& session;std::chrono::steady_clock::time_point begin=std::chrono::steady_clock::now();
        ~CallbackTimer(){const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();
          ++session.callback_total_count;session.callback_total_us+=us;session.callback_max_us=std::max(session.callback_max_us,us);
          if(session.callback_time_count<session.callback_us.size())session.callback_us[session.callback_time_count++]=us;}
      } timer{session}; // includes trace, conversion, guards and command creation, even on throw
      session.control_entered=true;
      session.heartbeat.fetch_add(1,std::memory_order_relaxed);
      if(!session.trace.capture(s,dt.toSec()))throw Abort(Fault::SessionTimeout);
      auto sample=convert(s,expected,robot,model);sample.stop_requested=cancel.load(std::memory_order_relaxed);
      sample.mode_ok=s.robot_mode==franka::RobotMode::kMove;
      std::array<double,7> command{};
      if(session.free_guiding_requested){
        if(!guiding_links_within_bounds(initial_links,link_poses(model,s)))throw Abort(Fault::Bounds);
        const auto lo=robot.getLowerJointVelocityLimits(s.q_d),hi=robot.getUpperJointVelocityLimits(s.q_d);
        const auto output=guiding.get(s,sample,baseline,session.contact_enabled.load(),lo,hi);
        if(output.released)guard.guiding_release(sample);
        command=output.q;
      }else command=reference.get(s); // validates and latches first actual q_d
      const auto result=guard.step(sample,dt.toSec());session.fault=result.fault;session.fault_reason=result.reason;
      enforce_operator_contact(result,session);
      session.phase.store(int(result.phase),std::memory_order_relaxed);
      if(!result.valid)throw Abort(result.fault);
      session.active.store(true,std::memory_order_relaxed);
      const franka::JointPositions output(command);
      return result.finished?franka::MotionFinished(output):output;
    },franka::ControllerMode::kCartesianImpedance,true,
       session.free_guiding_requested?franka::kMaxCutoffFrequency:franka::kDefaultCutoffFrequency);
  },review,proposal,session); // SDK exceptions retain their dynamic type and log
}
} // namespace fr3_native
