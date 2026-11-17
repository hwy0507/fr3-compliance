// Synthetic states and files only; no Robot or control loop is constructed.
#include "fr3_wrench_trace.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <type_traits>
int main(int argc,char** argv) {
  assert(argc==2);const std::filesystem::path dir(argv[1]);
  assert(std::filesystem::create_directory(dir));
  static_assert(std::is_trivially_copyable<fr3_trace::Frame>::value,"RT copies must be fixed-size");
  fr3_trace::Trace trace(2);franka::RobotState s;
  s.time=franka::Duration(1000);s.K_F_ext_hat_K[0]=21.5;s.O_F_ext_hat_K[4]=8.2;
  s.tau_J[3]=4.8;s.EE_T_K[14]=.1;
  assert(trace.capture(s,0));
  s.time=franka::Duration(1003);s.K_F_ext_hat_K[0]=std::numeric_limits<double>::quiet_NaN();
  std::array<bool,41> bits{};bool found=false;
  for(size_t i=0;i<bits.size();++i){bits.fill(false);bits[i]=true;franka::Errors e(bits);
    if(e.cartesian_reflex){s.current_errors=e;found=true;break;}}
  assert(found);s.robot_mode=franka::RobotMode::kReflex;
  assert(trace.capture(s,.003));assert(trace.size()==2&&trace.at(0).k_wrench[0]==21.5);
  assert(trace.at(0).ee_t_k[14]==.1&&trace.at(1).cartesian_reflex&&trace.at(1).mode==4);
  assert(std::isnan(trace.at(1).k_wrench[0]));
  assert(!trace.capture(s,.001)&&trace.overflowed()&&trace.size()==2);
  trace.write(dir/"wrench.csv");
  std::ifstream in(dir/"wrench.csv");std::string header,line;std::getline(in,header);std::getline(in,line);
  auto columns=[](const std::string& x){return std::count(x.begin(),x.end(),',')+1;};
  assert(columns(header)==columns(line));assert(header.find("EE_T_K_15")!=std::string::npos);
  franka::Record before{},fault{};before.state.time=franka::Duration(1002);fault.state=s;
  before.command.joint_positions.q.fill(.25);fault.command.joint_positions.q.fill(.5);
  franka::ControlException error("synthetic cartesian_reflex\nno robot connected",{before,fault});
  fr3_trace::write_exception(dir,error);
  std::ifstream tail(dir/"sdk_exception_wrench.csv");std::getline(tail,header);
  size_t rows=0;while(std::getline(tail,line)){assert(columns(header)==columns(line));++rows;}assert(rows==2);
  std::ifstream states(dir/"sdk_exception_states.txt");std::stringstream text;text<<states.rdbuf();
  assert(text.str().find("cartesian_reflex")!=std::string::npos);
  fr3_trace::Trace empty(0);assert(!empty.capture(s,0));empty.write(dir/"empty.csv");
  bool write_rejected=false;
  try{trace.write(dir/"missing"/"wrench.csv");}catch(const std::runtime_error&){write_rejected=true;}
  assert(write_rejected);
  std::cout<<"PASS: per-callback fields, raw NaN, fault state, bounded overflow, CSV schema and SDK-only reflex tail; no hardware.\n";
}
