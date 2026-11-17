// Actual reference shaper + actual guard, with synthetic states only. No Robot.
#include "start_protocol.hpp"
#include <iostream>
using namespace fr3_native;
namespace {
int checks=0;
void need(bool x,const char* message){++checks;if(!x)throw std::runtime_error(message);}
Sample good(){Sample s;s.q<<0,-.5,0,-1.5,0,1.5,0;s.position<<.3,0,.4;
  s.jacobian.leftCols<6>()=.5*fr3_contact::M6::Identity();return s;}
Baseline baseline(){auto s=good();std::vector<Sample> v(1001,s);for(size_t i=0;i<v.size();++i)v[i].time=i*.001;
  return qualify_static_baseline(v,std::vector<franka::RobotMode>(v.size(),franka::RobotMode::kIdle));}
void tests(){
  for(bool ee_only:{false,true}){
    auto s=good();Proposal p;p.operator_ended=true;p.delta_moment_stop=1;
    Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p,true);need(g.step(s,0).valid,"1Nm candidate start");
    for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"1Nm candidate settle");}
    s.wrench[3]=1;if(!ee_only)s.raw_wrench[3]=1;s.time+=.001;
    need(g.step(s,.001).valid,"1Nm equality accepted at each point");
    s.wrench[3]=1.0001;if(!ee_only)s.raw_wrench[3]=1.0001;s.time+=.001;
    const auto r=g.step(s,.001);need(r.fault==Fault::Wrench&&std::string(r.reason)==
      (ee_only?"relative_EE_moment":"relative_K_moment"),"1Nm exceeded stops at the correct point");}
  {auto s=good();Proposal p;p.operator_ended=true;p.delta_moment_stop=1;auto b=baseline();
    b.raw[3]=b.ee[3]=.9;s.raw_wrench=b.raw;s.wrench=b.ee;
    Guard g(s,b,Eigen::Matrix3d::Identity(),p,true);need(g.step(s,0).valid,"1Nm biased baseline start");
    for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"1Nm biased settle");}
    s.raw_wrench[3]=s.wrench[3]=1.501;s.time+=.001;
    const auto r=g.step(s,.001);need(r.fault==Fault::Wrench&&std::string(r.reason)=="absolute_K_moment",
      "absolute 1.5Nm stops even when relative change below 1Nm");}
  {auto s=good();Proposal p;p.operator_ended=true;p.delta_moment_stop=.5;
    Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p,true);need(g.step(s,0).valid,"candidate start");
    for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"candidate settle");}
    s.raw_wrench[0]=s.wrench[0]=1.03;s.raw_wrench[3]=s.wrench[3]=.2513;s.time+=.001;
    need(g.step(s,.001).valid,"candidate permits modeled 0.2513Nm contact without changing old defaults");
    s.raw_wrench[3]=s.wrench[3]=.5;s.time+=.001;need(g.step(s,.001).valid,"inclusive 0.5Nm limit");
    s.raw_wrench[3]=s.wrench[3]=.5001;s.time+=.001;auto r=g.step(s,.001);
    need(r.fault==Fault::Wrench&&std::string(r.reason)=="relative_K_moment","candidate still stops above 0.5 with named reason");
    s.raw_wrench.setZero();s.wrench.setZero();s.time+=.001;
    need(std::string(g.step(s,.001).reason)=="relative_K_moment","exact first fault reason latched");}
  {auto s=good();Proposal p;p.operator_ended=true;p.delta_moment_stop=.5;
    auto b=baseline();b.raw[3]=b.ee[3]=.9;s.raw_wrench=b.raw;s.wrench=b.ee;
    Guard g(s,b,Eigen::Matrix3d::Identity(),p,true);need(g.step(s,0).valid,"biased moment baseline start");
    for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"biased settle");}
    s.raw_wrench[3]=s.wrench[3]=1.501;s.time+=.001;
    auto r=g.step(s,.001);need(r.fault==Fault::Wrench&&std::string(r.reason)=="absolute_K_moment","absolute 1.5Nm remains independent of relative candidate");}
  {auto s=good();Proposal p;p.operator_ended=true;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p,true);
    need(g.step(s,0).valid,"operator-ended start");
    for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"operator-ended settling");}
    s.position[0]+=.005;s.raw_wrench[0]=s.wrench[0]=2;
    for(int i=0;i<70000;++i){s.time+=.001;const auto r=g.step(s,.001);
      need(r.valid&&!r.finished&&r.phase==Phase::Contact,"gentle guiding has no 2s/15s/60s countdown");}
    s.stop_requested=true;s.time+=.001;need(g.step(s,.001).fault==Fault::OperatorStop,"manual end still stops immediately");}
  {auto s=good();Proposal p;p.operator_ended=true;Guard g(s,baseline(),Eigen::Matrix3d::Identity(),p,true);
    need(g.step(s,0).valid,"operator guard start");for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"settle");}
    s.raw_wrench[0]=s.wrench[0]=6.01;s.time+=.001;
    need(g.step(s,.001).fault==Fault::Wrench,"operator-ended never disables force guard");}
  {LinkPoses initial;for(auto& pose:initial)pose=Eigen::Matrix4d::Identity();auto actual=initial;
    need(guiding_links_within_bounds(initial,actual),"initial whole-arm envelope");
    actual[2](0,3)=.02001;need(!guiding_links_within_bounds(initial,actual),"elbow excursion rejected even if TCP fixed");
    actual=initial;actual[4].topLeftCorner<3,3>()=Eigen::AngleAxisd(.09,V3::UnitZ()).toRotationMatrix();
    need(!guiding_links_within_bounds(initial,actual),"whole-link rotation bound");}
  const std::array<double,7> lo{-1,-1,-1,-1,-1,-1,-1},hi{1,1,1,1,1,1,1};
  {auto s=good();auto b=baseline();GuidedCommand follow(s);franka::RobotState state;
    Eigen::Map<V7>(state.q_d.data())=s.q;const auto initial=state.q_d;
    need(follow.get(state,s,b,false,lo,hi).q==initial,"first reference exactly continuous");
    s.q[0]+=.005;s.raw_wrench[0]=s.wrench[0]=2;s.position[0]+=.0025;
    for(int i=0;i<50;++i)need(follow.get(state,s,b,false,lo,hi).q==initial,"no following before DRAG authorization");
    auto tick=[&](bool permitted){const auto previous=state.q_d,velocity=state.dq_d,acceleration=state.ddq_d;
      const auto out=follow.get(state,s,b,permitted,lo,hi);
      for(size_t j=0;j<7;++j){const double v=(out.q[j]-previous[j])/.001,a=(v-velocity[j])/.001;
        need(std::abs(v)<=.015000001,"reference velocity bound");need(std::abs(a)<=.200001,"reference acceleration bound");
        need(std::abs((a-acceleration[j])/.001)<=1.001,"reference jerk bound");
        state.q_d[j]=out.q[j];state.dq_d[j]=v;state.ddq_d[j]=a;}
      return out;};
    for(int i=0;i<1000;++i)tick(true);
    need(state.q_d[0]>.004,"reference follows actual displaced posture within 1s");
    s.raw_wrench.setZero();s.wrench.setZero();need(tick(true).released,"release captured once");
    const double latched=s.q[0];s.q[0]-=.0005;
    for(int i=0;i<2000;++i)need(!tick(true).released,"released target not reanchored to jitter");
    if(std::abs(state.q_d[0]-latched)>=1e-6)std::cerr<<"reference="<<state.q_d[0]<<" target="<<latched<<" velocity="<<state.dq_d[0]<<" acceleration="<<state.ddq_d[0]<<'\n';
    need(std::abs(state.q_d[0]-latched)<1e-6,"hold the released posture instead of original anchor");
    need(std::abs(state.dq_d[0])<1e-5&&std::abs(state.ddq_d[0])<1e-3,"released reference must settle without oscillation");}
  {auto s=good();auto b=baseline();Guard g(s,b,Eigen::Matrix3d::Identity(),Proposal{},true);
    need(g.step(s,0).valid,"start");
    for(int i=0;i<3100;++i){s.time+=.001;need(g.step(s,.001).valid,"settle");}
    s.position[0]+=.005;s.raw_wrench[0]=s.wrench[0]=2;s.time+=.001;
    need(g.step(s,.001).phase==Phase::Contact,"guiding contact");
    s.raw_wrench.setZero();s.wrench.setZero();g.guiding_release(s);
    for(int i=0;i<300;++i){s.time+=.001;need(g.step(s,.001).valid,"settle at NEW target");}
    s.time+=.001;need(g.step(s,.001).phase==Phase::Ready,"new target qualifies without return to origin");
    s.position[0]+=.016;s.time+=.001;need(g.step(s,.001).fault==Fault::Bounds,"global workspace NOT reset after new latch");}
  {auto s=good();Guard fixed(s,baseline(),Eigen::Matrix3d::Identity());bool rejected=false;
    try{fixed.guiding_release(s);}catch(const Abort&){rejected=true;}need(rejected,"fixed-point behavior cannot change silently");}
  {Session session;std::atomic<bool> stop{false};OperatorGate gate(Trial::FreeGuiding,session,stop);
    session.active.store(true);session.phase.store(int(Phase::Ready));
    need(gate.status(true).find("READY_TO_DRAG")==0,"guiding is a separate Ready mode");
    need(gate.permit_contact(true),"DRAG enable");need(gate.status(true).find("READY_DRAG")==0,"guiding cue");
    gate.stop();need(!gate.permit_contact(true),"guiding STOP cannot reset");}
  {Session session;std::atomic<bool> stop{false};OperatorGate gate(Trial::FreeGuiding,session,stop,true);
    session.active.store(true);session.phase.store(int(Phase::Ready));need(gate.permit_contact(true),"operator cue arm");
    need(gate.status(true).find("within 2s")==std::string::npos&&gate.status(true).find("operator-ended")!=std::string::npos,
      "operator-ended prompt has no obsolete 2-second instruction");}
  std::cout<<"PASS "<<checks<<" synthetic guiding/reference assertions; no Robot or setters\n";
}
}
int main(){try{tests();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
