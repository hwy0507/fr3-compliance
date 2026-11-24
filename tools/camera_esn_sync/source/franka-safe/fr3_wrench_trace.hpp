#pragma once
// Client-side diagnostics only. No Robot, model, setter, control or recovery calls.
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <array>
#include <atomic>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>
#include <utility>

namespace fr3_trace {
struct Frame {
  double robot_time_s=0,dt=0,success=0;
  std::array<double,7> q{},dq{},q_d{},dq_d{},ddq_d{},tau{},tau_d{},dtau{},tau_ext{};
  std::array<double,6> o_wrench{},k_wrench{},cart_contact{},cart_collision{};
  std::array<double,7> joint_contact{},joint_collision{};
  std::array<double,16> o_t_ee{},ee_t_k{};
  int mode=0;
  bool errors=false,last_errors=false,cartesian_reflex=false,joint_reflex=false,communication_error=false;
  bool last_cartesian_reflex=false,last_joint_reflex=false,last_communication_error=false;
};
inline Frame frame(const franka::RobotState& s,double dt) noexcept {
  Frame f;f.robot_time_s=s.time.toSec();f.dt=dt;f.success=s.control_command_success_rate;
  f.q=s.q;f.dq=s.dq;f.q_d=s.q_d;f.dq_d=s.dq_d;f.ddq_d=s.ddq_d;
  f.tau=s.tau_J;f.tau_d=s.tau_J_d;f.dtau=s.dtau_J;f.tau_ext=s.tau_ext_hat_filtered;
  f.o_wrench=s.O_F_ext_hat_K;f.k_wrench=s.K_F_ext_hat_K;
  f.cart_contact=s.cartesian_contact;f.cart_collision=s.cartesian_collision;
  f.joint_contact=s.joint_contact;f.joint_collision=s.joint_collision;
  f.o_t_ee=s.O_T_EE;f.ee_t_k=s.EE_T_K;f.mode=static_cast<int>(s.robot_mode);
  f.errors=bool(s.current_errors);f.last_errors=bool(s.last_motion_errors);
  f.cartesian_reflex=s.current_errors.cartesian_reflex;f.joint_reflex=s.current_errors.joint_reflex;
  f.communication_error=s.current_errors.communication_constraints_violation;
  f.last_cartesian_reflex=s.last_motion_errors.cartesian_reflex;
  f.last_joint_reflex=s.last_motion_errors.joint_reflex;
  f.last_communication_error=s.last_motion_errors.communication_constraints_violation;
  return f;
}
class Trace {
 public:
  static_assert(std::atomic<size_t>::is_always_lock_free,"realtime trace cursors must be lock free");
  // Allocate and touch memory before entering the realtime control loop.
  explicit Trace(size_t capacity):frames_(capacity) {}
  void enable_streaming(){if(used_||produced_.load())throw std::logic_error("trace already used");streaming_=true;}
  bool streaming() const noexcept{return streaming_;}
  bool capture(const franka::RobotState& s,double dt) noexcept {
    if(streaming_){
      const size_t p=produced_.load(std::memory_order_relaxed),c=consumed_.load(std::memory_order_acquire);
      if(p-c==frames_.size()){overflow_=true;return false;}
      frames_[p%frames_.size()]=frame(s,dt);produced_.store(p+1,std::memory_order_release);return true;
    }
    if(used_==frames_.size()){overflow_=true;return false;}
    frames_[used_++]=frame(s,dt);return true;
  }
  size_t size() const noexcept{return streaming_?produced_.load():used_;}
  size_t pending() const noexcept{return produced_.load(std::memory_order_acquire)-consumed_.load(std::memory_order_relaxed);}
  bool overflowed() const noexcept{return overflow_;}
  size_t allocated_bytes() const noexcept{return frames_.size()*sizeof(Frame);}
  const Frame& at(size_t i) const {if(streaming_||i>=used_)throw std::out_of_range("trace row");return frames_[i];}
  static void header(std::ostream& f) {
    f<<"index,robot_time_s,dt,success_rate,mode,errors,last_errors,cartesian_reflex,joint_reflex,communication_error,last_cartesian_reflex,last_joint_reflex,last_communication_error";
    for(const auto& group:std::array<std::pair<const char*,int>,17>{{
      {"q",7},{"dq",7},{"q_d",7},{"dq_d",7},{"ddq_d",7},{"tau_J",7},{"tau_J_d",7},
      {"dtau_J",7},{"tau_ext",7},{"O_wrench",6},{"K_wrench",6},{"joint_contact",7},
      {"cartesian_contact",6},{"joint_collision",7},{"cartesian_collision",6},{"O_T_EE",16},{"EE_T_K",16}}})
      for(int i=0;i<group.second;++i)f<<','<<group.first<<'_'<<i;
    f<<'\n';
  }
  static void row(std::ostream& f,size_t index,const Frame& s) {
    f<<index<<','<<s.robot_time_s<<','<<s.dt<<','<<s.success<<','<<s.mode<<','<<s.errors<<','<<s.last_errors
     <<','<<s.cartesian_reflex<<','<<s.joint_reflex<<','<<s.communication_error
     <<','<<s.last_cartesian_reflex<<','<<s.last_joint_reflex<<','<<s.last_communication_error;
    for(const auto* a:{&s.q,&s.dq,&s.q_d,&s.dq_d,&s.ddq_d,&s.tau,&s.tau_d,&s.dtau,&s.tau_ext})for(double x:*a)f<<','<<x;
    for(const auto* a:{&s.o_wrench,&s.k_wrench})for(double x:*a)f<<','<<x;
    for(double x:s.joint_contact)f<<','<<x;
    for(double x:s.cart_contact)f<<','<<x;
    for(double x:s.joint_collision)f<<','<<x;
    for(double x:s.cart_collision)f<<','<<x;
    for(const auto* a:{&s.o_t_ee,&s.ee_t_k})for(double x:*a)f<<','<<x;
    f<<'\n';
  }
  void write(const std::filesystem::path& path) const {
    if(streaming_)throw std::logic_error("streaming trace must be drained, never overwritten");
    std::ofstream f(path);f<<std::setprecision(17);header(f);
    for(size_t i=0;i<used_;++i)row(f,i,frames_[i]);
    f.flush();
    if(!f)throw std::runtime_error("wrench trace write failed");
  }
  size_t drain(std::ostream& out,size_t maximum){
    if(!streaming_)throw std::logic_error("trace not streaming");
    size_t count=0;
    while(count<maximum){
      const size_t c=consumed_.load(std::memory_order_relaxed);
      if(c==produced_.load(std::memory_order_acquire))break;
      const Frame copy=frames_[c%frames_.size()];row(out,c,copy);
      if(!out)throw std::runtime_error("streaming wrench log failed");
      consumed_.store(c+1,std::memory_order_release);++count;
    }
    return count;
  }
 private:
  std::vector<Frame> frames_;size_t used_=0;bool overflow_=false;
  bool streaming_=false;
  alignas(64) std::atomic<size_t> produced_{0};
  alignas(64) std::atomic<size_t> consumed_{0};
};
inline void write_exception(const std::filesystem::path& out,const franka::ControlException& e) {
  // Called after control has unwound, never in a callback. Preserve SDK packets
  // that triggered a reflex before they could reach the application callback.
  std::ofstream csv(out/"sdk_exception_wrench.csv"),states(out/"sdk_exception_states.txt"),
                commands(out/"sdk_exception_commands.csv"),info(out/"sdk_exception.txt");
  csv<<std::setprecision(17);states<<std::setprecision(17);commands<<std::setprecision(17);
  Trace::header(csv);commands<<"index,state_time_s";
  for(int j=0;j<7;++j)commands<<",previous_limited_command_q_"<<j;
  commands<<'\n';
  info<<"sdk_records="<<e.log.size()<<"\ncommand_alignment=command(n) belongs to state(n+1); separate from raw application command\n"<<e.what()<<'\n';
  for(size_t i=0;i<e.log.size();++i) {
    const auto& rec=e.log[i];
    const double dt=i?rec.state.time.toSec()-e.log[i-1].state.time.toSec():0.;
    Trace::row(csv,i,frame(rec.state,dt));states<<"record_"<<i<<'='<<rec.state<<'\n';
    commands<<i<<','<<rec.state.time.toSec();for(double x:rec.command.joint_positions.q)commands<<','<<x;commands<<'\n';
  }
  csv.flush();states.flush();commands.flush();info.flush();
  if(!csv||!states||!commands||!info)throw std::runtime_error("SDK exception trace write failed");
}
} // namespace fr3_trace
