// Bounded empty-Hand teaching replay. No gripper, parameter, mode or recovery calls.
// --offline does not construct Robot. --read-only never calls control.
// --execute-reviewed performs ONE explicitly reviewed segment; never auto-chains.
#include <franka/robot.h>
#include <franka/control_types.h>
#include <franka/rate_limiting.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "fr3_wrench_trace.hpp"
using Q=std::array<double,7>;
struct Piece {double begin{},end{};std::array<std::array<double,6>,7> c{};};
struct Path {std::string phase;bool hold_j1=true;double duration{},mass{},j1{};std::array<double,16> frame{};
  std::array<double,3> com{};std::array<double,9> inertia{};std::vector<Piece> pieces;};
constexpr Q lower{-2.9007,-1.8361,-2.9007,-3.077,-2.8763,.4398,-3.0508};
constexpr Q upper{2.9007,1.8361,2.9007,-.1169,2.8763,4.6216,3.0508};
volatile std::sig_atomic_t interrupted=0;
void on_signal(int){interrupted=1;}
double number(std::istream& in){double x;if(!(in>>x)||!std::isfinite(x))throw std::runtime_error("invalid plan value");return x;}
template<size_t N>void read_array(std::istream& in,std::array<double,N>& a){for(auto& x:a)x=number(in);}
template<size_t N>void close_array(const std::array<double,N>& a,const std::array<double,N>& b,double tol,
                                 const char* field="array"){
  for(size_t j=0;j<N;++j)if(!std::isfinite(a[j])||!std::isfinite(b[j])||std::abs(a[j]-b[j])>tol){
    std::ostringstream message;message<<std::setprecision(17)<<field<<"["<<j<<"] actual="<<a[j]
      <<" expected="<<b[j]<<" delta="<<(a[j]-b[j])<<" tolerance="<<tol;
    throw std::runtime_error(message.str());
  }
}
Q evaluate(const Path& p,double t,int derivative=0){
  t=std::clamp(t,0.,p.duration);
  auto it=std::upper_bound(p.pieces.begin(),p.pieces.end(),t,
                          [](double value,const Piece& x){return value<x.begin;});
  if(it!=p.pieces.begin())--it;
  const double x=t-it->begin;Q q{};
  for(size_t j=0;j<7;++j){
    double result=0;
    for(int degree=5;degree>=derivative;--degree){
      double c=it->c[j][static_cast<size_t>(5-degree)];
      for(int n=0;n<derivative;++n)c*=degree-n;
      result=result*x+c;
    }
    q[j]=result;
  }
  if(p.hold_j1)q[0]=derivative==0?p.j1:0.;
  return q;
}
Path read_path(const std::string& name){
  std::ifstream in(name);std::string magic;size_t count{};Path p;
  if(!(in>>magic>>p.phase>>count)||(magic!="FR3_TAUGHT_FIXED_J1_V1"&&magic!="FR3_TAUGHT_ALL_JOINTS_V2")||
     (p.phase!="return"&&p.phase!="replay")||count==0||count>10000)throw std::runtime_error("invalid plan header");
  p.hold_j1=magic=="FR3_TAUGHT_FIXED_J1_V1";
  p.duration=number(in);p.mass=number(in);p.j1=number(in);
  if(p.duration<4||p.duration>90||p.mass<0||p.mass>10||p.j1<lower[0]||p.j1>upper[0])throw std::runtime_error("invalid plan bounds");
  read_array(in,p.frame);read_array(in,p.com);read_array(in,p.inertia);
  p.pieces.resize(count);double previous=0;
  for(auto& piece:p.pieces){
    piece.begin=number(in);piece.end=number(in);
    if(std::abs(piece.begin-previous)>1e-9||piece.end<=piece.begin||piece.end>p.duration+1e-9)
      throw std::runtime_error("invalid polynomial intervals");
    for(auto& c:piece.c)read_array(in,c);
    if(p.hold_j1){
      if(std::abs(piece.c[0][5]-p.j1)>1e-9)throw std::runtime_error("J1 must be constant");
      for(size_t n=0;n<5;++n)if(std::abs(piece.c[0][n])>1e-10)throw std::runtime_error("J1 must not move");
    }
    previous=piece.end;
  }
  if(std::abs(previous-p.duration)>1e-9)throw std::runtime_error("incomplete polynomial path");
  std::string extra;if(in>>extra)throw std::runtime_error("trailing plan data");
  for(double t:{0.,p.duration})for(int d:{1,2})close_array(evaluate(p,t,d),Q{},1e-6);
  for(size_t i=1;i<p.pieces.size();++i)for(int d:{0,1,2}){
    // Compare both sides near each knot; tolerate the finite 2 ns separation.
    const double t=p.pieces[i].begin;
    close_array(evaluate(p,t-1e-9,d),evaluate(p,t+1e-9,d),d==2?1e-3:1e-6);
  }
  for(double t=0;t<p.duration+.0005;t+=.001){
    const auto q=evaluate(p,t),v=evaluate(p,t,1),a=evaluate(p,t,2),jerk=evaluate(p,t,3);
    for(size_t j=0;j<7;++j)if(!std::isfinite(q[j])||q[j]<lower[j]||q[j]>upper[j]||
      !std::isfinite(v[j])||std::abs(v[j])>.75||!std::isfinite(a[j])||std::abs(a[j])>9.5||
      !std::isfinite(jerk[j])||std::abs(jerk[j])>4500)throw std::runtime_error("sampled path bounds failed");
  }
  return p;
}
void state_ok(const franka::RobotState& s,const Path& p){
  if(s.current_errors||s.last_motion_errors)throw std::runtime_error("robot errors; no recovery");
  if(!std::isfinite(s.m_total)||std::abs(s.m_total-p.mass)>1e-7)throw std::runtime_error("load changed");
  close_array(s.F_T_EE,p.frame,1e-7,"F_T_EE");close_array(s.F_x_Ctotal,p.com,1e-7,"F_x_Ctotal");close_array(s.I_total,p.inertia,1e-7,"I_total");
  for(auto x:s.joint_contact)if(x!=0)throw std::runtime_error("joint contact");
  for(auto x:s.cartesian_contact)if(x!=0)throw std::runtime_error("Cartesian contact");
  for(auto x:s.joint_collision)if(x!=0)throw std::runtime_error("joint collision");
  for(auto x:s.cartesian_collision)if(x!=0)throw std::runtime_error("Cartesian collision");
}
void start_ok(const franka::RobotState& s,const Path& p){
  close_array(s.q,evaluate(p,0),.003,"start.q_vs_plan");
  close_array(s.q_d,evaluate(p,0),.001,"start.q_d_vs_plan");
  close_array(s.q_d,s.q,.001,"start.q_d_vs_measured");
  close_array(s.dq,Q{},.01,"start.dq");close_array(s.dq_d,Q{},.001,"start.dq_d");
  close_array(s.ddq_d,Q{},.001,"start.ddq_d");
}
Q callback_start(const franka::RobotState& s,const franka::RobotState& initial,const Path& p){
  state_ok(s,p);start_ok(s,p);
  close_array(s.q,initial.q,.001,"first.q_vs_preflight_q");
  // q_d is a control-session reference, not a cross-session invariant. Re-anchor
  // only AFTER checking measured drift, plan alignment, velocity and config.
  // No offset jump: command(p, start, 0) equals this first callback q_d exactly.
  return s.q_d;
}
Q command(const Path& p,const Q& start,double t){
  Q q=evaluate(p,t);const auto reference=evaluate(p,0);
  double u=std::clamp(t,0.,1.);const double fade=1-u*u*u*(10+u*(-15+6*u));
  for(size_t j=p.hold_j1?1:0;j<7;++j)q[j]+=(start[j]-reference[j])*fade;
  if(p.hold_j1)q[0]=start[0];
  return q;
}
void preview(franka::Robot& robot,const Path& p,const Q& start){
  Q last=start,v{},a{};double worst=0,held=0;
  for(double t=0;t<p.duration+.251;t+=.001){
    const auto raw=command(p,start,t);auto filtered=raw;
    for(size_t j=0;j<7;++j)filtered[j]=franka::lowpassFilter(.001,raw[j],last[j],franka::kDefaultCutoffFrequency);
    const auto lo=robot.getLowerJointVelocityLimits(last),hi=robot.getUpperJointVelocityLimits(last);
    for(size_t j=0;j<7;++j)if(lo[j]>0||hi[j]<0)throw std::runtime_error("zero velocity excluded along path");
    const auto next=franka::limitRate(hi,lo,franka::kMaxJointAcceleration,franka::kMaxJointJerk,filtered,last,v,a);
    Q nv{},na{};
    for(size_t j=0;j<7;++j){
      nv[j]=(next[j]-last[j])/.001;na[j]=(nv[j]-v[j])/.001;
      if(!std::isfinite(next[j])||next[j]<lower[j]||next[j]>upper[j]||std::abs(nv[j])>.8)
        throw std::runtime_error("limiter output bounds");
      worst=std::max(worst,std::abs(next[j]-raw[j]));
    }
    held=std::max(held,std::abs(next[0]-start[0]));last=next;v=nv;a=na;
  }
  if(worst>.003||(p.hold_j1&&held>1e-7))throw std::runtime_error("limiter distorts path or moves held J1");
  close_array(last,command(p,start,p.duration),.0005);
  std::cout<<"Read-only limiter preview passed; max raw/limited difference "<<worst<<" rad\n";
}
struct Sample{double t,dt;Q q,dq,raw,desired;};
struct Snapshots {
  bool have_initial=false,have_first=false,have_last=false;
  franka::RobotState initial{},first{},last{};
};
void write_snapshots(const std::filesystem::path& out,const Snapshots& snapshots,const char* stage){
  std::ofstream f(out/"diagnostic_states.txt");f<<std::setprecision(17)<<"stage="<<stage<<'\n';
  for(const auto& item:std::array<std::pair<const char*,const franka::RobotState*>,3>{{
      {"preflight",snapshots.have_initial?&snapshots.initial:nullptr},
      {"first_callback",snapshots.have_first?&snapshots.first:nullptr},
      {"last_callback",snapshots.have_last?&snapshots.last:nullptr}}}){
    f<<item.first<<"=";if(item.second)f<<*item.second;else f<<"not_observed";f<<'\n';
  }
  if(!f)throw std::runtime_error("diagnostic write failed");
}
int main(int argc,char** argv){
  const bool offline=argc==4&&std::string(argv[1])=="--offline";
  const bool readonly=argc==5&&std::string(argv[1])=="--read-only";
  const bool pathonly=argc==5&&std::string(argv[1])=="--read-only-path";
  const bool execute=argc==5&&std::string(argv[1])=="--execute-reviewed";
  if(!offline&&!readonly&&!pathonly&&!execute){std::cerr<<"Usage: --offline PLAN NEW_DIR | --read-only|--read-only-path|--execute-reviewed IP PLAN NEW_DIR\n";return 2;}
  const std::filesystem::path out(argv[offline?3:4]);bool created=false;int code=1;std::string status="not_started";
  std::vector<Sample> log(92000);log.clear();
  fr3_trace::Trace wrench_trace(execute?92000:0); // client buffer only; no robot settings
  bool sdk_exception_write_failed=false;
  Snapshots snapshots;const char* stage="plan_validation";
  std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
  try{
    const auto p=read_path(argv[offline?2:3]);
    if(!std::filesystem::create_directory(out))throw std::runtime_error("output exists");
    created=true;
    if(offline){status="offline_checks_passed_no_robot_connection";code=0;}
    else{
      stage="connect_read";franka::Robot robot(argv[2]);auto initial=robot.readOnce();
      snapshots.initial=initial;snapshots.have_initial=true;stage="preflight";state_ok(initial,p);
      if(execute&&initial.robot_mode!=franka::RobotMode::kIdle)throw std::runtime_error("execution requires Idle; no mode change attempted");
      if(!pathonly){
        start_ok(initial,p);
      }
      stage="limiter_preview";
      preview(robot,p,pathonly?evaluate(p,0):initial.q_d);
      if(readonly||pathonly){status=pathonly?"read_only_candidate_preview_no_start_match_no_motion":"read_only_preview_passed_no_motion";code=0;}
      else{
        double elapsed=0;bool first=true;Q start=initial.q_d;status="control_started";stage="control_entry";
        robot.control([&](const franka::RobotState& s,franka::Duration period)->franka::JointPositions{
          // Fixed-size memory copies BEFORE any guard; file I/O happens after control exits.
          snapshots.last=s;snapshots.have_last=true;
          if(first){snapshots.first=s;snapshots.have_first=true;}
          stage=first?"first_callback":"tracking";
          if(!wrench_trace.capture(s,period.toSec()))throw std::runtime_error("wrench trace capacity");
          if(interrupted)throw std::runtime_error("operator interrupt");
          state_ok(s,p);const double dt=period.toSec();
          if(!std::isfinite(dt)||dt<0||dt>.003)throw std::runtime_error("control period exceeded");
          if(first){
            if(dt!=0)throw std::runtime_error("first callback period must be zero");
            start=callback_start(s,initial,p);first=false;
          }
          else{if(dt<=0)throw std::runtime_error("nonpositive period");elapsed+=dt;}
          const auto raw=command(p,start,elapsed);
          // These SDK queries calculate limits locally; they do not command the arm.
          const auto lo=robot.getLowerJointVelocityLimits(s.q_d),hi=robot.getUpperJointVelocityLimits(s.q_d);
          for(size_t j=0;j<7;++j)if(!std::isfinite(lo[j])||!std::isfinite(hi[j])||lo[j]>0||hi[j]<0)
            throw std::runtime_error("callback zero velocity excluded");
          for(size_t j=0;j<7;++j)if(!std::isfinite(s.q[j])||!std::isfinite(s.dq[j])||
             std::abs(s.q[j]-raw[j])>.03||std::abs(s.dq[j])>.8)throw std::runtime_error("tracking/speed guard");
          if(p.hold_j1&&std::abs(s.q[0]-initial.q[0])>.001)throw std::runtime_error("held J1 drift");
          if(log.size()==log.capacity())throw std::runtime_error("log capacity");
          log.push_back({elapsed,dt,s.q,s.dq,raw,s.q_d});franka::JointPositions result(raw);
          if(elapsed>=p.duration+.25)return franka::MotionFinished(result);
          return result;
        },franka::ControllerMode::kJointImpedance,true);
        stage="final_check";const auto final=robot.readOnce();state_ok(final,p);
        close_array(final.q,command(p,start,p.duration),.003,"final.q_vs_goal");status="completed_one_segment";code=0;
      }
    }
  }catch(const franka::ControlException& e){
    status="failed at "+std::string(stage)+": "+e.what();std::cerr<<status<<'\n';
    if(created)try{fr3_trace::write_exception(out,e);}catch(const std::exception& writing){
      sdk_exception_write_failed=true;std::cerr<<"Original control failure retained; SDK trace write failed: "<<writing.what()<<'\n';
    }
  }catch(const std::exception& e){status="failed at "+std::string(stage)+": "+e.what();std::cerr<<status<<'\n';}
  if(created){
    try{write_snapshots(out,snapshots,stage);wrench_trace.write(out/"wrench.csv");}
    catch(const std::exception& e){std::cerr<<status<<"; diagnostic write failed: "<<e.what()<<'\n';return 3;}
    std::ofstream receipt(out/"status.txt");receipt<<status<<'\n';
    std::ofstream f(out/"motion.csv");f<<std::setprecision(17)<<"t,dt";
    for(auto prefix:{"q","dq","command","desired"})for(int j=1;j<=7;++j)f<<','<<prefix<<j;
    f<<'\n';for(const auto& s:log){f<<s.t<<','<<s.dt;for(const auto& a:{s.q,s.dq,s.raw,s.desired})for(auto x:a)f<<','<<x;f<<'\n';}
    if(!f||!receipt)return 3;
    std::ofstream trace_info(out/"wrench_trace_info.txt");
    trace_info<<"schema=FR3_WRENCH_TRACE_V1\nrows="<<wrench_trace.size()<<"\noverflow="<<wrench_trace.overflowed()
      <<"\nallocated_bytes="<<wrench_trace.allocated_bytes()<<"\nrow_order=one received callback state, captured before guards; not proof a command was sent\n"
      <<"timing=absolute robot_time_s; missing packets are not interpolated\n"
      <<"wrench=raw SDK O/K fields; no bias subtraction, clipping or reference-point conversion\n"
      <<"transforms=column-major O_T_EE and EE_T_K; units=N,Nm,rad,s\n"
      <<"mode=0 Other,1 Idle,2 Move,3 Guiding,4 Reflex,5 UserStopped,6 AutomaticErrorRecovery\n"
      <<"SDK_tail_capacity=unchanged SDK constructor default (50 records)\n"
      <<"sdk_exception_write_failed="<<sdk_exception_write_failed<<'\n';
    if(!trace_info||sdk_exception_write_failed)return 3;
  }
  std::cout<<status<<'\n';return code;
}
