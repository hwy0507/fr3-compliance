#include "native_hold.hpp"
#include <fstream>
#include <iostream>
#include <limits>
using namespace fr3_native;
// Force linking the complete real SDK integration, without ever invoking it.
decltype(&run_reviewed_native_hold) native_session_link_check=&run_reviewed_native_hold;
namespace {
int checks=0;
void need(bool x,const char* message){++checks;if(!x)throw std::runtime_error(message);}
Sample good(){Sample s;s.q<<0,-.5,0,-1.5,0,1.5,0;s.position<<.3,0,.4;
  s.jacobian.leftCols<6>()=.5*fr3_contact::M6::Identity();return s;}
Baseline baseline(){std::vector<Sample> samples(1001,good());for(size_t i=0;i<samples.size();++i)samples[i].time=i*.001;
  return qualify_static_baseline(samples,std::vector<franka::RobotMode>(samples.size(),franka::RobotMode::kIdle));}
Guard guard(const Sample& s){return Guard(s,baseline(),Eigen::Matrix3d::Identity());}
Result tick(Guard& g,Sample& s,double dt=.001){s.time+=dt;return g.step(s,dt);}
void ready(Guard& g,Sample& s){need(g.step(s,0).valid,"start");for(int i=0;i<3100;++i)need(tick(g,s).valid,"settle");}
template<class F>void fault(F mutate,Fault expected){auto s=good();auto g=guard(s);ready(g,s);mutate(s);
  auto r=tick(g,s);need(!r.valid&&r.fault==expected,"wrong native fault");s=good();s.time=20;
  need(!tick(g,s).valid,"fault must latch");}
Review reviewed(){Review r;r.motion_authorized=r.stiffness_write_authorized=r.restore_authorized=
  r.previous_stiffness_verified=r.static_baseline_verified=r.tool_and_frames_verified=
  r.existing_collision_protection_verified=r.realtime_and_stopping_verified=r.onsite_ready=true;
  r.previous_stiffness={1000,1000,1000,100,100,100};return r;}
Review retained(){Review r;r.end_policy=EndPolicy::RetainTrial;
  r.motion_authorized=r.stiffness_write_authorized=r.static_baseline_conditions_reviewed=r.onsite_ready=
    r.retain_trial_authorized=r.collision_settings_preserved=r.collision_values_unverified=
    r.tool_configuration_and_static_reviewed=r.host_realtime_and_stop_reviewed=true;return r;}
void tests(){
  {auto a=good();franka::RobotState state;Eigen::Map<V7>(state.q_d.data())=a.q;state.q_d[0]+=.0004;
    FixedCommand fixed(a);need(fixed.get(state)==state.q_d,"first native reference exactly continuous");
    const auto first=fixed.get(state);state.q_d[0]+=.01;need(fixed.get(state)==first,"reference stays fixed during yielding");
    FixedCommand bad(a);bool rejected=false;try{bad.get(state);}catch(const Abort& e){rejected=e.fault==Fault::Startup;}
    need(rejected,"large first reference jump rejected");}
  {Review r;bool rejected=false;try{r.validate();}catch(const std::invalid_argument&){rejected=true;}need(rejected,"default review refuses motion");}
  {auto r=reviewed();r.previous_stiffness.fill(0);bool rejected=false;try{r.validate();}catch(const std::invalid_argument&){rejected=true;}need(rejected,"unknown prior stiffness cannot be restored");}
  {Proposal p;p.force_stop=7;bool rejected=false;try{p.validate();}catch(const std::invalid_argument&){rejected=true;}need(rejected,"cannot silently increase trial force");}
  {auto s=good();auto g=guard(s);need(g.step(s,0).valid,"start");bool finished=false;
    Eigen::internal::set_is_malloc_allowed(false);
    for(int i=0;i<15002;++i){auto r=tick(g,s);need(r.valid,"static native guard");if(r.finished){finished=true;break;}}
    Eigen::internal::set_is_malloc_allowed(true);need(finished,"bounded normal finish");}
  {auto s=good();auto g=guard(s);ready(g,s);s.raw_wrench[0]=s.wrench[0]=2;s.contact=true;s.position[0]+=.005;
    for(int i=0;i<100;++i){auto r=tick(g,s);need(r.valid&&r.phase==Phase::Contact,"finite contact must continue");}
    s.contact=false;s.raw_wrench.setZero();s.wrench.setZero();
    for(int i=0;i<300;++i)need(tick(g,s).valid,"release debounce/recovery");
    s.position[0]-=.005;need(tick(g,s).phase==Phase::Ready,"release returns ready");}
  fault([](Sample& s){s.collision=true;},Fault::Robot);
  fault([](Sample& s){s.errors=true;},Fault::Robot);
  fault([](Sample& s){s.configuration_ok=false;},Fault::Configuration);
  fault([](Sample& s){s.stop_requested=true;},Fault::OperatorStop);
  fault([](Sample& s){s.raw_wrench[0]=6.1;},Fault::Wrench);
  fault([](Sample& s){s.raw_wrench[3]=1.6;},Fault::Wrench);
  fault([](Sample& s){s.wrench[3]=1.6;},Fault::Wrench);
  fault([](Sample& s){s.raw_wrench[2]=1.01;},Fault::Wrench);
  fault([](Sample& s){s.raw_wrench[3]=.26;},Fault::Wrench);
  fault([](Sample& s){s.position[0]+=.0151;},Fault::Bounds);
  fault([](Sample& s){s.position[2]+=.0051;},Fault::Bounds);
  fault([](Sample& s){s.q[0]+=.171;},Fault::Bounds);
  fault([](Sample& s){s.dq[0]=.151;},Fault::Speed);
  fault([](Sample& s){s.dq[0]=.08;s.dq[1]=.08;},Fault::Speed);
  fault([](Sample& s){s.success_rate=.98;},Fault::Timing);
  fault([](Sample& s){s.jacobian.row(4).setZero();},Fault::Singularity);
  fault([](Sample& s){s.raw_wrench[0]=std::numeric_limits<double>::quiet_NaN();},Fault::Nonfinite);
  {auto s=good();auto g=guard(s);need(g.step(s,0).valid,"start");need(tick(g,s,.003).fault==Fault::Timing,"late callback");}
  {auto s=good();auto g=guard(s);need(g.step(s,0).valid,"start");s.raw_wrench[0]=s.wrench[0]=2;
    need(tick(g,s).fault==Fault::Startup,"no push during settle");}
  {auto s=good();auto g=guard(s);ready(g,s);s.raw_wrench[0]=s.wrench[0]=2;Result r;
    for(int i=0;i<2002;++i)r=tick(g,s);
    need(r.fault==Fault::ContactTimeout,"blocked contact bounded");}
  {auto s=good();Proposal p;p.min_success=.95;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p);
    ready(g,s);s.success_rate=.949999988079071;need(tick(g,s).valid,"float32 representation of 95 percent accepted");
    s.success_rate=.94;need(tick(g,s).fault==Fault::Timing,"below authorized 95 percent still stops");}
  {auto s=good();Proposal p;p.min_success=.95;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p);
    ready(g,s);need(tick(g,s,.003).fault==Fault::Timing,"95 percent never disables period guard");}
  {auto s=good();Proposal p;p.min_success=.90;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p);
    ready(g,s);s.success_rate=.8999999761581421;need(tick(g,s).valid,"float32 90 percent accepted");
    s.success_rate=.89;need(tick(g,s).fault==Fault::Timing,"below authorized 90 percent still stops");}
  {auto s=good();Proposal p;p.min_success=.90;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p);
    ready(g,s);s.errors=true;need(tick(g,s).fault==Fault::Robot,"90 percent preserves every SDK error stop");}
  {auto s=good();Proposal p;p.min_success=.90;p.max_period=.005;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p);
    ready(g,s);need(tick(g,s,.003).valid,"reviewed native profile tolerates 3ms receive gap");
    need(tick(g,s,.006).fault==Fault::Timing,"reviewed native profile still rejects gaps above 5ms");}
  {std::vector<Sample> samples(1001,good());for(size_t i=0;i<samples.size();++i)samples[i].time=i*.001;
    samples[100].contact=true;bool rejected=false;
    const std::vector<franka::RobotMode> modes(samples.size(),franka::RobotMode::kIdle);
    try{qualify_static_baseline(samples,modes);}catch(const std::invalid_argument&){rejected=true;}need(rejected,"contact cannot become baseline");
    samples[100].contact=false;samples[100].raw_wrench[0]=.5;rejected=false;
    try{qualify_static_baseline(samples,modes);}catch(const std::invalid_argument&){rejected=true;}need(rejected,"unstable baseline refused");
    samples[100].raw_wrench.setZero();rejected=false;auto wrong_mode=modes;wrong_mode[100]=franka::RobotMode::kMove;
    try{qualify_static_baseline(samples,wrong_mode);}catch(const std::invalid_argument&){rejected=true;}need(rejected,"baseline must be Idle");}
  {auto s=good();Baseline b=baseline();b.raw[0]=b.ee[0]=1.5;s.raw_wrench=b.raw;s.wrench=b.ee;
    Guard g(s,b,Eigen::Matrix3d::Identity());ready(g,s);s.raw_wrench[0]=s.wrench[0]=6.1;
    need(tick(g,s).fault==Fault::Wrench,"baseline cannot hide absolute overload");}
  // Transaction tests use fake setters only; no franka::Robot is ever created.
  {Session s;int calls=0;bool body=false;detail::stiffness_transaction([&](auto k){++calls;
      need(k==(calls==1?Proposal{}.stiffness:reviewed().previous_stiffness),"setter order");},
      [&]{body=true;},reviewed(),Proposal{},s);
    need(body&&calls==2&&s.restore_succeeded,"normal restoration");}
  {Session s;int calls=0;bool preserved=false;
    try{detail::stiffness_transaction([&](auto){++calls;},[]{throw franka::ControlException("original fault");},reviewed(),Proposal{},s);}
    catch(const franka::ControlException& e){preserved=std::string(e.what())=="original fault";}
    need(calls==2&&s.restore_succeeded&&preserved,"original SDK failure restored/rethrown");}
  {Session s;int calls=0;bool body=false;
    try{detail::stiffness_transaction([&](auto){if(++calls==1)throw std::runtime_error("lost acknowledgement");},[&]{body=true;},reviewed(),Proposal{},s);}
    catch(const std::runtime_error&){}
    need(!body&&calls==2&&s.restore_attempted,"uncertain setter acknowledged by restoration attempt");}
  {Session s;int calls=0;bool preserved=false;
    try{detail::stiffness_transaction([&](auto){if(++calls==2)throw std::runtime_error("restore failed");},
      []{throw Abort(Fault::OperatorStop);},reviewed(),Proposal{},s);}
    catch(const Abort& e){preserved=e.fault==Fault::OperatorStop;}
    need(preserved&&s.restoration_failure&&!s.restore_succeeded,"both failures preserved");}
  {Session s;int calls=0;bool rejected=false;
    try{detail::stiffness_transaction([&](auto){++calls;},[]{},Review{},Proposal{},s);}
    catch(const std::invalid_argument&){rejected=true;}need(rejected&&calls==0,"default review prevents even fake writes");}
  {auto r=retained();r.validate();need(!r.previous_stiffness_verified&&!r.existing_collision_protection_verified,
    "retain profile keeps unknown original stiffness and thresholds unverified");
    Session s;int calls=0;detail::stiffness_transaction([&](auto k){++calls;need(k==Proposal{}.stiffness,"only trial K written");},[]{},r,Proposal{},s);
    need(calls==1&&s.trial_stiffness_retained&&!s.restore_attempted,"retain trial: one setter, no original restore");}
  {Session s;int calls=0;bool preserved=false;
    try{detail::stiffness_transaction([&](auto){++calls;},[]{throw franka::ControlException("original stop");},retained(),Proposal{},s);}
    catch(const franka::ControlException& e){preserved=std::string(e.what())=="original stop";}
    need(preserved&&calls==1&&s.trial_stiffness_retained&&!s.restore_attempted,"retain after SDK failure; no reset/retry");}
  {Session s;int calls=0;bool body=false;
    try{detail::stiffness_transaction([&](auto){++calls;throw std::runtime_error("ack unknown");},[&]{body=true;},retained(),Proposal{},s);}
    catch(const std::runtime_error&){}
    need(calls==1&&!body&&!s.trial_stiffness_retained&&!s.restore_attempted,"retain lost ack remains unknown; no second write");}
  {auto r=retained();r.retain_trial_authorized=false;Session s;int calls=0;
    try{detail::stiffness_transaction([&](auto){++calls;},[]{},r,Proposal{},s);}catch(const std::invalid_argument&){}
    need(calls==0,"retain needs separate explicit authorization");}
  std::cout<<"PASS "<<checks<<" native guard/transaction assertions; synthetic only; no Robot, control or parameter writes.\n";
}
} // namespace
int main(int argc,char** argv){
  try{
    if(argc==2&&std::string(argv[1])=="--proposal"){
      std::cout<<"K axes: [300,300,800] N/m; [50,50,50] Nm/rad\n"
        <<"Fixed reference, command speed 0; measured TCP 0.05m/s, angular 0.2rad/s, joint 0.15rad/s\n"
        <<"XY push suggestion 1-3N; delta stop 4N, absolute stop 6N / 1.5Nm; offset 15mm\n"
        <<"3s no-contact settling, max 2s contact, max 15s session; review defaults FALSE\n";return 0;
    }
    if(argc!=2||std::string(argv[1])!="--offline")throw std::invalid_argument("offline/proposal only; no IP, execution or setter option accepted");
    tests();return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
