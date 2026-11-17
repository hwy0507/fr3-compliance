#pragma once
#include "active_vmc.hpp"
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <array>

namespace fr3_contact {
// OFFLINE PROPOSAL ONLY. These are application guards, NOT robot safety settings.
struct Limits {
  double command_linear=.03, measured_linear=.06;
  double command_angular=.15, measured_angular=.25;
  double command_joint=.08, measured_joint=.12, joint_acceleration=.25;
  double displacement=.04, angle=.15, joint_displacement=.25, joint_margin=.17;
  double force_on=2, force_off=1, force_stop=12, moment_stop=1;
  double release_dwell=.25, contact_timeout=3, session_timeout=30;
  double torque_rate=50, min_singular=.025, max_period=.002, min_success_rate=.99;
  V7 external_joint_limit=(V7()<<5,5,5,5,2,2,2).finished();
  V7 command_torque_limit=(V7()<<8,8,8,8,3,3,3).finished();
  V7 measured_torque_limit=(V7()<<60,60,60,60,9,9,9).finished();
  void validate() const {
    const double values[]={command_linear,measured_linear,command_angular,measured_angular,
      command_joint,measured_joint,joint_acceleration,displacement,angle,joint_displacement,
      joint_margin,force_on,force_off,force_stop,moment_stop,release_dwell,contact_timeout,
      session_timeout,torque_rate,min_singular,max_period,min_success_rate};
    for(double v:values)if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid limits");
    if(command_linear>=measured_linear||command_angular>=measured_angular||command_joint>=measured_joint||
       force_off>=force_on||force_on>=force_stop||min_success_rate>1||max_period>.003||torque_rate>1000)
      throw std::invalid_argument("inconsistent limits");
    for(const V7* v:{&external_joint_limit,&command_torque_limit,&measured_torque_limit})
      if(!v->allFinite()||(v->array()<=0).any())throw std::invalid_argument("invalid torque bounds");
    const V7 nominal=(V7()<<87,87,87,87,12,12,12).finished();
    if((command_torque_limit.array()>nominal.array()).any()||
       (measured_torque_limit.array()>nominal.array()).any()||
       (external_joint_limit.array()>measured_torque_limit.array()).any())
      throw std::invalid_argument("torque envelope exceeds nominal references");
  }
};
struct Sample {
  double time=0, success_rate=1;
  V7 q=V7::Zero(),dq=V7::Zero(),tau_measured=V7::Zero(),tau_previous=V7::Zero();
  V7 coriolis=V7::Zero(),external_joint=V7::Zero();
  V7 velocity_lower=V7::Constant(-1),velocity_upper=V7::Constant(1);
  V3 position=V3::Zero(); Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
  J67 jacobian=J67::Zero();
  // Environment-on-robot, expressed in O: wrench at EE, raw_wrench at K (independent guard).
  V6 wrench=V6::Zero(),raw_wrench=V6::Zero();
  bool errors=false,collision=false,contact=false,mode_ok=true,configuration_ok=true;
  bool stop_requested=false;
};
struct Reference {
  V3 position=V3::Zero(); Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
  V6 feedforward=V6::Zero(); V3 direction=V3::Zero();
};
enum class Phase { Free, Contact, Recovering, Stopped };
enum class Fault { None, Nonfinite, Timing, Robot, Configuration, OperatorStop,
  Bounds, Speed, Wrench, Torque, Singularity, Timeout, Startup, Reference, Numerical };
struct Result {
  bool valid=false; Fault fault=Fault::None; Phase phase=Phase::Free;
  V7 torque=V7::Zero(), qdot=V7::Zero(), action=V7::Zero();
  double recovery_memory=0, elapsed=0; bool limited=false;
};
inline bool rotation_ok(const Eigen::Matrix3d& r) {
  // Robot transforms are transported at float precision (observed orthogonality
  // error 1.23e-7). Numerical validity only, not a physical orientation allowance.
  return r.allFinite()&&(r.transpose()*r-Eigen::Matrix3d::Identity()).norm()<1e-6&&std::abs(r.determinant()-1)<1e-6;
}
inline V3 rotation_error(const Eigen::Matrix3d& target,const Eigen::Matrix3d& actual) {
  const Eigen::AngleAxisd a(target*actual.transpose()); return a.angle()*a.axis();
}

class ContactCore {
 public:
  ContactCore(const Reference& initial, const V7& nominal_q,const Eigen::Matrix3d& world_R_base,
              Limits limits=Limits()): target_(initial),anchor_(initial),q0_(nominal_q),R_(world_R_base),limits_(limits) {
    limits_.validate();
    if(!q0_.allFinite()||!target_.position.allFinite()||!rotation_ok(target_.rotation)||
       !target_.feedforward.allFinite()||!target_.direction.allFinite()||!rotation_ok(R_))
      throw std::invalid_argument("invalid reference/frame");
    // Always start stationary. A separately reviewed reference feeder may advance later.
    if(target_.feedforward.norm()!=0||target_.direction.norm()!=0)
      throw std::invalid_argument("first hardware adapter requires fixed-point reference");
  }
  Result step(const Sample& s,double dt) {
    if(fault_!=Fault::None)return stopped(fault_);
    try {return advance(s,dt);} catch(const std::exception&) {return stopped(Fault::Numerical);}
  }
  Result step(const Sample& s,double dt,const Reference& next) {
    if(fault_!=Fault::None)return stopped(fault_);
    if(!started_||!std::isfinite(dt)||dt<=0||!next.position.allFinite()||!rotation_ok(next.rotation)||
       !next.feedforward.allFinite()||!next.direction.allFinite()||next.direction.norm()>1.00000001||
       (next.position-target_.position).norm()>limits_.command_linear*dt+1e-10||
       rotation_error(next.rotation,target_.rotation).norm()>limits_.command_angular*dt+1e-10||
       (next.position-anchor_.position).norm()>limits_.displacement||
       rotation_error(next.rotation,anchor_.rotation).norm()>limits_.angle||
       next.feedforward.head<3>().norm()>limits_.command_linear||next.feedforward.tail<3>().norm()>limits_.command_angular)
      return stopped(Fault::Reference);
    target_=next;
    return step(s,dt);
  }
  Fault fault() const {return fault_;}
 private:
  Reference target_,anchor_; V7 q0_; Eigen::Matrix3d R_; Limits limits_;
  ActiveVmc actor_; V7 action_=V7::Zero(),previous_qdot_=V7::Zero();
  V6 yield_=V6::Zero(); double scale_=1, policy_time_=0,elapsed_=0,last_time_=0;
  double contact_time_=0,release_time_=0; bool started_=false;
  Phase phase_=Phase::Free; Fault fault_=Fault::None;
  Result stopped(Fault f) {fault_=f;phase_=Phase::Stopped;Result o;o.fault=f;o.phase=phase_;o.elapsed=elapsed_;
    o.action=action_;o.recovery_memory=actor_.recovery_memory;return o;}
  bool finite(const Sample& s) const {
    return std::isfinite(s.time)&&std::isfinite(s.success_rate)&&s.q.allFinite()&&s.dq.allFinite()&&
      s.tau_measured.allFinite()&&s.tau_previous.allFinite()&&s.coriolis.allFinite()&&s.external_joint.allFinite()&&
      s.velocity_lower.allFinite()&&s.velocity_upper.allFinite()&&s.position.allFinite()&&rotation_ok(s.rotation)&&
      s.jacobian.allFinite()&&s.wrench.allFinite()&&s.raw_wrench.allFinite();
  }
  Result advance(const Sample& s,double dt) {
    if(!finite(s)||!std::isfinite(dt))return stopped(Fault::Nonfinite);
    if(s.stop_requested)return stopped(Fault::OperatorStop);
    if(s.errors||s.collision||!s.mode_ok)return stopped(Fault::Robot);
    if(!s.configuration_ok)return stopped(Fault::Configuration);
    if((s.velocity_lower.array()>0).any()||(s.velocity_upper.array()<0).any()||
       (s.velocity_lower.array()>s.velocity_upper.array()).any())return stopped(Fault::Bounds);
    const V7 lo=(V7()<<-2.9007,-1.8361,-2.9007,-3.077,-2.8763,.4398,-3.0508).finished();
    const V7 hi=(V7()<<2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508).finished();
    if((s.q.array()<lo.array()+limits_.joint_margin).any()||(s.q.array()>hi.array()-limits_.joint_margin).any()||
       (s.q-q0_).cwiseAbs().maxCoeff()>limits_.joint_displacement)return stopped(Fault::Bounds);
    const V3 ep=target_.position-s.position,er=rotation_error(target_.rotation,s.rotation);
    if(ep.norm()>limits_.displacement||er.norm()>limits_.angle||
       (s.position-anchor_.position).norm()>limits_.displacement||
       rotation_error(s.rotation,anchor_.rotation).norm()>limits_.angle)return stopped(Fault::Bounds);
    const V6 measured=s.jacobian*s.dq;
    if(s.dq.cwiseAbs().maxCoeff()>limits_.measured_joint||measured.head<3>().norm()>limits_.measured_linear||
       measured.tail<3>().norm()>limits_.measured_angular||(s.dq.array()<s.velocity_lower.array()).any()||
       (s.dq.array()>s.velocity_upper.array()).any())return stopped(Fault::Speed);
    if((s.tau_measured.cwiseAbs().array()>limits_.measured_torque_limit.array()).any()||
       (s.tau_previous.cwiseAbs().array()>limits_.command_torque_limit.array()).any())return stopped(Fault::Torque);
    for(const V6* w:{&s.wrench,&s.raw_wrench})
      if(w->head<3>().norm()>limits_.force_stop||w->tail<3>().norm()>limits_.moment_stop)return stopped(Fault::Wrench);
    if((s.external_joint.cwiseAbs().array()>limits_.external_joint_limit.array()).any())return stopped(Fault::Wrench);
    const M6 jgram=s.jacobian*s.jacobian.transpose();
    Eigen::SelfAdjointEigenSolver<M6> eigen(jgram,Eigen::EigenvaluesOnly);
    if(eigen.info()!=Eigen::Success||!eigen.eigenvalues().allFinite()||
       eigen.eigenvalues().minCoeff()<limits_.min_singular*limits_.min_singular)
      return stopped(Fault::Singularity);
    if(!started_) {
      if(dt!=0||s.contact||s.raw_wrench.head<3>().norm()>limits_.force_off||s.raw_wrench.tail<3>().norm()>.15||
         s.dq.cwiseAbs().maxCoeff()>.005||ep.norm()>.001||er.norm()>.005||(s.q-q0_).norm()>.005)
        return stopped(Fault::Startup);
      started_=true;last_time_=s.time;
      Result o;o.valid=true;o.torque=s.tau_previous;return o; // first-cycle continuity, no invented dt
    }
    if(dt<=0||dt>limits_.max_period||s.time<=last_time_||std::abs(s.time-last_time_-dt)>1e-6||
       s.success_rate<limits_.min_success_rate||s.success_rate>1)return stopped(Fault::Timing);
    elapsed_+=dt;last_time_=s.time;
    if(elapsed_>limits_.session_timeout)return stopped(Fault::Timeout);
    const double force=std::max(s.wrench.head<3>().norm(),s.raw_wrench.head<3>().norm());
    const bool touching=s.contact||force>=limits_.force_on;
    if(touching) {phase_=Phase::Contact;release_time_=0;}
    if(phase_==Phase::Contact) {
      contact_time_+=dt;
      if(contact_time_>limits_.contact_timeout)return stopped(Fault::Timeout);
      if(!s.contact&&force<limits_.force_off)release_time_+=dt; else release_time_=0;
      if(release_time_>=limits_.release_dwell)phase_=Phase::Recovering;
    }
    if(phase_==Phase::Recovering&&ep.norm()<.002&&er.norm()<.01&&measured.norm()<.01) {
      phase_=Phase::Free;contact_time_=0;
    }
    // Common WBC structure; conservative hardware envelope applied AFTER residual composition.
    V6 nominal;nominal.head<3>()=norm_cap(V3(target_.feedforward.head<3>()+3*ep),limits_.command_linear);
    nominal.tail<3>()=norm_cap(V3(target_.feedforward.tail<3>()+2.5*er),limits_.command_angular);
    const M6 gram=s.jacobian*s.jacobian.transpose()+.035*.035*M6::Identity();
    const Eigen::Matrix<double,7,6> pinv=s.jacobian.transpose()*gram.ldlt().solve(M6::Identity());
    const V7 nominal_qdot=pinv*nominal+(M7::Identity()-pinv*s.jacobian)*(.20*(q0_-s.q));
    Obs obs=Obs::Zero();obs.head<7>()=s.q;obs.segment<7>(7)=s.dq;
    const V6 actual_nominal=s.jacobian*nominal_qdot;
    obs.segment<3>(14)=R_*actual_nominal.head<3>();obs.segment<3>(17)=R_*actual_nominal.tail<3>();
    obs.segment<3>(20)=R_*ep;obs.segment<3>(23)=R_*er;
    obs.segment<3>(26)=R_*(actual_nominal-measured).head<3>();obs.segment<3>(29)=R_*(actual_nominal-measured).tail<3>();
    obs.segment<7>(32)=s.external_joint;obs.segment<3>(39)=R_*s.wrench.head<3>();obs.segment<3>(42)=R_*s.wrench.tail<3>();
    obs.segment<3>(45)=R_*target_.direction;
    // Fixed-point adapter supplies zero direction; moving-intent core is verified offline only.
    policy_time_+=dt;
    if(policy_time_>=.04-1e-10){action_=actor_.step(obs,.04);policy_time_-=.04;}
    scale_+=clip((1-.8*std::max(0.0,action_[0]))-scale_,-4*dt,4*dt);
    for(int i=0;i<6;++i) {
      const double cap=i<3?.32:.6, rate=i<3?1.2:4;
      yield_[i]+=clip(action_[i+1]*cap-yield_[i],-rate*dt,rate*dt);
    }
    V6 yield_base;yield_base.head<3>()=R_.transpose()*yield_.head<3>();yield_base.tail<3>()=R_.transpose()*yield_.tail<3>();
    const M6 yield_gram=s.jacobian*s.jacobian.transpose()+.04*.04*M6::Identity();
    V7 desired=scale_*nominal_qdot+s.jacobian.transpose()*yield_gram.ldlt().solve(yield_base);
    double fraction=1;
    const V6 desired_twist=s.jacobian*desired;
    fraction=std::min({fraction,limits_.command_joint/std::max(desired.cwiseAbs().maxCoeff(),1e-12),
      limits_.command_linear/std::max(desired_twist.head<3>().norm(),1e-12),
      limits_.command_angular/std::max(desired_twist.tail<3>().norm(),1e-12)});
    for(int i=0;i<7;++i) {
      if(desired[i]>0)fraction=std::min(fraction,s.velocity_upper[i]/desired[i]);
      if(desired[i]<0)fraction=std::min(fraction,s.velocity_lower[i]/desired[i]);
    }
    desired*=fraction;
    const V7 delta=desired-previous_qdot_;
    const double slew=std::min(1.0,limits_.joint_acceleration*dt/std::max(delta.cwiseAbs().maxCoeff(),1e-12));
    const V7 qdot=previous_qdot_+slew*delta;
    const V6 applied_twist=s.jacobian*qdot;
    // Changing J can make yesterday's limited qdot unsafe. Reject conflicting constraints instead of re-clipping past slew bounds.
    if(applied_twist.head<3>().norm()>limits_.command_linear+1e-10||applied_twist.tail<3>().norm()>limits_.command_angular+1e-10||
       qdot.cwiseAbs().maxCoeff()>limits_.command_joint+1e-10||(qdot.array()<s.velocity_lower.array()).any()||
       (qdot.array()>s.velocity_upper.array()).any())return stopped(Fault::Speed);
    const V7 gains=(V7()<<42,42,36,32,9,8,6).finished();
    const V7 demand=s.coriolis+gains.cwiseProduct(qdot-s.dq); // FCI: NO gravity/friction added
    V7 torque;
    for(int i=0;i<7;++i) {
      const double lo_tau=std::max(-limits_.command_torque_limit[i],s.tau_previous[i]-limits_.torque_rate*dt);
      const double hi_tau=std::min(limits_.command_torque_limit[i],s.tau_previous[i]+limits_.torque_rate*dt);
      if(lo_tau>hi_tau)return stopped(Fault::Torque);
      torque[i]=clip(demand[i],lo_tau,hi_tau);
    }
    if(!torque.allFinite()||!qdot.allFinite())return stopped(Fault::Numerical);
    previous_qdot_=qdot;
    Result o;o.valid=true;o.phase=phase_;o.torque=torque;o.qdot=qdot;o.action=action_;
    o.recovery_memory=actor_.recovery_memory;o.elapsed=elapsed_;o.limited=fraction<1||slew<1||(torque-demand).norm()>1e-10;
    return o;
  }
};
} // namespace fr3_contact
