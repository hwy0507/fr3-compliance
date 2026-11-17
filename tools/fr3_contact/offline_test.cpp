#include "reviewed_session.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

using namespace fr3_contact;
namespace {
int checks=0;
void require(bool yes,const char* message) {++checks;if(!yes)throw std::runtime_error(message);}
Sample good() {
  Sample s;s.q<<0,-.5,0,-1.5,0,1.5,0;s.position<<.3,0,.4;
  s.jacobian.leftCols<6>()=.5*M6::Identity();return s;
}
Reference ref(const Sample& s) {Reference r;r.position=s.position;r.rotation=s.rotation;return r;}
ContactCore core(const Sample& s,Limits limits=Limits()) {return ContactCore(ref(s),s.q,Eigen::Matrix3d::Identity(),limits);}
Result tick(ContactCore& c,Sample& s,double dt=.001) {s.time+=dt;return c.step(s,dt);}
franka::RobotState fake_state() {
  franka::RobotState s;
  for(auto* a:{&s.O_T_EE,&s.F_T_EE,&s.F_T_NE,&s.NE_T_EE,&s.EE_T_K})
    Eigen::Map<Eigen::Matrix4d>(a->data()).setIdentity();
  s.robot_mode=franka::RobotMode::kIdle;return s;
}
Sample converted(const franka::RobotState& s,const franka::RobotState& expected) {
  return convert_state(s,expected,good().jacobian,V7::Zero(),V7::Constant(-1),V7::Constant(1));
}
template<class F> void fault_test(F mutate,Fault expected) {
  auto s=good();auto c=core(s);require(c.step(s,0).valid,"startup failed");mutate(s);
  const auto r=tick(c,s);require(!r.valid&&r.fault==expected,"wrong fault");
  s=good();s.time=1;require(!tick(c,s).valid,"fault must latch, no automatic restart");
}
void tests() {
  {
    const Eigen::Matrix3d rounded=Eigen::AngleAxisd(.7,V3(1,2,3).normalized())
      .toRotationMatrix().cast<float>().cast<double>();
    require(rotation_ok(rounded),"float-precision rotation accepted without normalization");
    auto shear=rounded;shear(0,1)+=1e-4;
    require(!rotation_ok(shear),"nonrigid shear still rejected");
  }
  {
    SessionReview review;bool threw=false;
    try{validate_session_review(review);}catch(const std::invalid_argument&){threw=true;}
    require(threw,"default review cannot allow control");
    review.motion_authorized=true;threw=false;
    try{validate_session_review(review);}catch(const std::invalid_argument&){threw=true;}
    require(threw,"motion phrase alone cannot substitute for physical preflight");
    SessionTrace trace;trace.prepare(1);require(trace.frames.size()==1008&&trace.used==0&&!trace.control_entered,"preallocate trace before control");
  }
  {
    auto s=good();auto c=core(s);s.tau_previous.setConstant(.25);
    const auto r=c.step(s,0);require(r.valid&&(r.torque-s.tau_previous).norm()==0,"first command continuity");
    s.tau_previous.setZero();
    Eigen::internal::set_is_malloc_allowed(false);
    for(int i=0;i<1000;++i) {
      const auto x=tick(c,s);require(x.valid&&x.torque.norm()<1e-12,"zero equilibrium must remain zero");
    }
    Eigen::internal::set_is_malloc_allowed(true);
  }
  fault_test([](Sample& s){s.q[0]=std::numeric_limits<double>::quiet_NaN();},Fault::Nonfinite);
  fault_test([](Sample& s){s.wrench[0]=std::numeric_limits<double>::infinity();},Fault::Nonfinite);
  fault_test([](Sample& s){s.coriolis[0]=std::numeric_limits<double>::quiet_NaN();},Fault::Nonfinite);
  fault_test([](Sample& s){s.errors=true;},Fault::Robot);
  fault_test([](Sample& s){s.collision=true;},Fault::Robot);
  fault_test([](Sample& s){s.mode_ok=false;},Fault::Robot);
  fault_test([](Sample& s){s.configuration_ok=false;},Fault::Configuration);
  fault_test([](Sample& s){s.stop_requested=true;},Fault::OperatorStop);
  fault_test([](Sample& s){s.dq[0]=.121;},Fault::Speed);
  fault_test([](Sample& s){s.dq[0]=.1;s.dq[1]=.1;},Fault::Speed); // TCP norm, not axiswise
  fault_test([](Sample& s){s.jacobian(3,3)=3;s.dq[3]=.1;},Fault::Speed);
  fault_test([](Sample& s){s.position[0]+=.041;},Fault::Bounds);
  fault_test([](Sample& s){s.rotation=Eigen::AngleAxisd(.151,V3::UnitZ()).toRotationMatrix();},Fault::Bounds);
  fault_test([](Sample& s){s.q[1]+=.251;},Fault::Bounds);
  fault_test([](Sample& s){s.velocity_lower[0]=.001;},Fault::Bounds);
  fault_test([](Sample& s){s.wrench[0]=12.01;},Fault::Wrench);
  fault_test([](Sample& s){s.raw_wrench[0]=12.01;},Fault::Wrench); // clipping/correction cannot hide force
  fault_test([](Sample& s){s.wrench[3]=1.01;},Fault::Wrench);
  fault_test([](Sample& s){s.external_joint[6]=2.01;},Fault::Wrench);
  fault_test([](Sample& s){s.tau_measured[6]=9.01;},Fault::Torque);
  fault_test([](Sample& s){s.tau_previous[0]=8.01;},Fault::Torque);
  fault_test([](Sample& s){s.jacobian.row(4).setZero();},Fault::Singularity);
  fault_test([](Sample& s){s.success_rate=.98;},Fault::Timing);
  {
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");
    require(c.step(s,.001).fault==Fault::Timing,"stale timestamp");
  }
  {
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");
    require(tick(c,s,.004).fault==Fault::Timing,"long callback gap");
  }
  {
    auto s=good();s.contact=true;auto c=core(s);
    require(c.step(s,0).fault==Fault::Startup,"cannot start under contact");
  }
  {
    auto s=good();auto c=core(s);require(c.step(s,.001).fault==Fault::Startup,"first dt must be zero");
  }
  {
    auto s=good();s.q[5]=.5;auto c=core(s);
    require(c.step(s,0).fault==Fault::Bounds,"joint limit margin");
  }
  {
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");
    s.contact=true;s.wrench[0]=3;s.raw_wrench=s.wrench;
    for(int i=0;i<100;++i){const auto r=tick(c,s);require(r.valid&&r.phase==Phase::Contact,"bounded contact must continue");}
    s.contact=false;s.wrench.setZero();s.raw_wrench.setZero();
    for(int i=0;i<249;++i)require(tick(c,s).phase==Phase::Contact,"release debounce");
    require(tick(c,s).phase==Phase::Free,"release transition");
  }
  {
    auto s=good();Limits l;l.contact_timeout=.1;auto c=core(s,l);require(c.step(s,0).valid,"start");
    s.contact=true;Result r;for(int i=0;i<102;++i)r=tick(c,s);
    require(r.fault==Fault::Timeout,"persistent blocked contact must stop");
  }
  {
    auto s=good();Limits l;l.session_timeout=.1;auto c=core(s,l);require(c.step(s,0).valid,"start");
    Result r;for(int i=0;i<102;++i)r=tick(c,s);require(r.fault==Fault::Timeout,"bounded session");
  }
  {
    bool threw=false;Limits l;l.force_stop=1;
    try{auto c=core(good(),l);(void)c;}catch(const std::invalid_argument&){threw=true;}
    require(threw,"reject inconsistent limits");
  }
  {
    bool threw=false;Limits l;l.command_torque_limit[6]=13;
    try{auto c=core(good(),l);(void)c;}catch(const std::invalid_argument&){threw=true;}
    require(threw,"reject envelope beyond nominal motor torque reference");
  }
  {
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");
    s.position[0]+=.01;s.wrench[0]=5;s.raw_wrench=s.wrench;
    V7 previous=V7::Zero();
    for(int i=0;i<2300;++i) {
      if(i==300){s.wrench.setZero();s.raw_wrench.setZero();}
      const auto r=tick(c,s);require(r.valid,"synthetic push/release rejected");
      const V6 twist=s.jacobian*r.qdot;
      require(twist.head<3>().norm()<=.03+1e-10&&twist.tail<3>().norm()<=.15+1e-10,"command TCP limits");
      require(r.qdot.cwiseAbs().maxCoeff()<=.08+1e-10&&(r.qdot-previous).cwiseAbs().maxCoeff()<=.00025+1e-10,"command speed/slew");
      require((r.torque-s.tau_previous).cwiseAbs().maxCoeff()<=.05+1e-10,"torque slew");
      s.tau_previous=r.torque;previous=r.qdot;
      if(i==2299)require(twist[0]<0,"after release, command must point back to fixed reference");
    }
  }
  {
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");
    Reference r=ref(s);r.direction=V3::UnitX();r.feedforward[0]=.02;
    s.wrench[0]=-4;s.raw_wrench=s.wrench;double max_memory=0;bool saw_limit=false;
    for(int i=0;i<1400;++i) {
      r.position[0]+=.00002;s.time+=.001;
      const auto out=c.step(s,.001,r);require(out.valid,"moving-intent synthetic active chain");
      max_memory=std::max(max_memory,out.recovery_memory);saw_limit|=out.limited;
      require((s.jacobian*out.qdot).head<3>().norm()<=.03+1e-10,"active recovery speed bounded");
      s.tau_previous=out.torque;
    }
    require(max_memory>.5&&saw_limit,"exercise active recovery AND independent limiter");
  }
  {
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");
    Reference r=ref(s);r.position[0]+=.01;s.time+=.001;
    require(c.step(s,.001,r).fault==Fault::Reference,"reference jump rejected");
  }
  {
    // Capping desired qdot must not excuse a previous command that becomes unsafe under a changed J.
    auto s=good();auto c=core(s);require(c.step(s,0).valid,"start");s.position[0]-=.02;
    for(int i=0;i<300;++i){auto r=tick(c,s);require(r.valid,"ramp");s.tau_previous=r.torque;}
    s.jacobian*=10;require(tick(c,s).fault==Fault::Speed,"changing-J conflicting constraints reject");
  }
  {
    // Pure RobotState conversion; no Robot is constructed.
    auto state=fake_state();
    state.EE_T_K[14]=.1;state.O_F_ext_hat_K[0]=state.K_F_ext_hat_K[0]=-2;
    state.O_F_ext_hat_K[4]=-.2;
    const auto expected=state;
    auto s=convert_state(state,expected,good().jacobian,V7::Zero(),V7::Constant(-1),V7::Constant(1));
    require(std::abs(s.wrench[0]-2)<1e-12&&std::abs(s.wrench[4]-.2)<1e-12,"force sign and K-to-EE moment translation");
    require(s.configuration_ok,"equal configuration");
    require(std::abs(s.raw_wrench[4])<1e-12,"raw K moment retained separately");
    state.m_load=.01;s=convert_state(state,expected,good().jacobian,V7::Zero(),V7::Constant(-1),V7::Constant(1));
    require(!s.configuration_ok,"load mutation detected");
    state=expected;state.EE_T_K[12]=.01;
    s=convert_state(state,expected,good().jacobian,V7::Zero(),V7::Constant(-1),V7::Constant(1));
    require(!s.configuration_ok,"stiffness frame mutation detected");
    state=expected;state.joint_contact[0]=std::numeric_limits<double>::quiet_NaN();
    s=convert_state(state,expected,good().jacobian,V7::Zero(),V7::Constant(-1),V7::Constant(1));require(s.errors,"NaN flag reject");
  }
  {
    // Nonidentity EE/K rotations, offset, arbitrary force and free moment.
    auto state=fake_state();
    Eigen::Map<Eigen::Matrix4d> t(state.O_T_EE.data()),k(state.EE_T_K.data());
    t.topLeftCorner<3,3>()=Eigen::AngleAxisd(.7,V3(1,2,3).normalized()).toRotationMatrix();
    k.topLeftCorner<3,3>()=Eigen::AngleAxisd(-.4,V3(3,-1,2).normalized()).toRotationMatrix();
    t.topRightCorner<3,1>()=V3(.3,-.2,.5);k.topRightCorner<3,1>()=V3(.04,-.02,.1);
    V6 environment_k;environment_k<<2,-3,1,.2,-.3,.1;
    Eigen::Map<V6>(state.K_F_ext_hat_K.data())=-environment_k;
    const Eigen::Matrix3d r=t.topLeftCorner<3,3>()*k.topLeftCorner<3,3>();
    const V3 arm=t.topLeftCorner<3,3>()*k.topRightCorner<3,1>();
    const V3 f=r*environment_k.head<3>(),m=r*environment_k.tail<3>();
    V6 origin;origin.head<3>()=f;origin.tail<3>()=m+(t.topRightCorner<3,1>()+arm).cross(f);
    Eigen::Map<V6>(state.O_F_ext_hat_K.data())=-origin;
    const auto s=converted(state,state);
    require(s.configuration_ok,"rotated K accepted");
    require((s.wrench.head<3>()-f).norm()<1e-12&&
            (s.wrench.tail<3>()-(origin.tail<3>()-t.topRightCorner<3,1>().cross(f))).norm()<1e-12,
            "K route equals independent base-origin to EE route");
    require((s.raw_wrench.tail<3>()-m).norm()<1e-12,"raw K is rotation only");
    const V3 v(.1,-.2,.3),omega(-.4,.2,.1);
    require(std::abs(s.wrench.head<3>().dot(v)+s.wrench.tail<3>().dot(omega)-
      (environment_k.head<3>().dot(r.transpose()*(v+omega.cross(arm)))+
       environment_k.tail<3>().dot(r.transpose()*omega)))<1e-12,"power invariant under point shift");
    // Moving the base origin must not change wrench at EE.
    t.topRightCorner<3,1>()+=V3(2,-1,3);
    Eigen::Map<V6>(state.O_F_ext_hat_K.data()).tail<3>()=-m-(t.topRightCorner<3,1>()+arm).cross(f);
    require((converted(state,state).wrench-s.wrench).norm()<1e-12,"EE wrench origin invariance");
    auto bad=state;bad.K_F_ext_hat_K[0]=std::numeric_limits<double>::quiet_NaN();
    require(!converted(bad,state).configuration_ok,"NaN K cannot fall back to O");
    bad=state;bad.O_F_ext_hat_K[4]=std::numeric_limits<double>::quiet_NaN();
    require(!converted(bad,state).configuration_ok,"NaN diagnostic O rejected");
    bad=state;bad.O_F_ext_hat_K[0]+=.01;
    require(!converted(bad,state).configuration_ok,"inconsistent force rejected");
    bad=state;bad.EE_T_K[0]=2;
    require(!converted(bad,bad).configuration_ok,"nonrigid K rejected");
  }
  {
    // Gravity is not an input to FCI torque synthesis; with zero servo error only coriolis remains.
    auto s=good();s.coriolis.setConstant(.02);auto c=core(s);require(c.step(s,0).valid,"start");
    require((tick(c,s).torque-s.coriolis).norm()<1e-12,"FCI torque convention");
  }
  {
    ActiveVmc a;Obs obs=Obs::Zero();obs[0]=std::numeric_limits<double>::quiet_NaN();bool threw=false;
    try{a.step(obs);}catch(const std::invalid_argument&){threw=true;}require(threw,"actor nonfinite reject");
  }
  {
    // Force channels affect action, but unrelated joint position channels cannot enter the actor.
    ActiveVmc a,b;Obs x=Obs::Zero(),y=Obs::Zero();x[39]=y[39]=2;y.head<14>().setConstant(100);
    for(int i=0;i<80;++i)require((a.step(x)-b.step(y)).norm()==0,"actor input boundary");
  }
  std::cout<<"PASS: "<<checks<<" assertions; bounded contact, faults, limits, active chain, frame/torque conventions.\n";
}
} // namespace
int main(int argc,char** argv) {
  try {
    if(argc==3&&std::string(argv[1])=="--wrench-fixture") {
      // Saved numbers only. No Robot/Model instance or network operation.
      std::ifstream in(argv[2]);if(!in)throw std::runtime_error("fixture unavailable");
      std::cout<<std::setprecision(17);std::string line;size_t rows=0;
      while(std::getline(in,line)) {
        std::istringstream row(line);auto state=fake_state();
        for(auto* a:{&state.O_T_EE,&state.EE_T_K})
          for(double& x:*a)if(!(row>>x)||!std::isfinite(x))throw std::runtime_error("invalid pose fixture");
        for(auto* a:{&state.O_F_ext_hat_K,&state.K_F_ext_hat_K})
          for(double& x:*a)if(!(row>>x)||!std::isfinite(x))throw std::runtime_error("invalid wrench fixture");
        std::string extra;if(row>>extra)throw std::runtime_error("extra fixture field");
        const auto s=converted(state,state);
        for(const V6* w:{&s.wrench,&s.raw_wrench})for(double x:*w)std::cout<<x<<' ';
        // Isolate the existing wrench guards; this is NOT a full hardware preflight.
        auto probe=good();probe.wrench=s.wrench;probe.raw_wrench=s.raw_wrench;
        auto c=core(probe);const auto result=c.step(probe,0);
        std::cout<<s.configuration_ok<<' '<<result.valid<<' '<<int(result.fault)<<'\n';++rows;
      }
      if(rows==0)throw std::runtime_error("empty fixture");
      return 0;
    }
    if(argc==3&&std::string(argv[1])=="--actor-fixture") {
      std::ifstream in(argv[2]);if(!in)throw std::runtime_error("fixture unavailable");ActiveVmc actor;
      std::cout<<std::setprecision(17);Obs obs;
      while(in>>obs[0]) {
        for(int i=1;i<48;++i)if(!(in>>obs[i]))throw std::runtime_error("truncated fixture");
        const auto action=actor.step(obs);
        for(double x:action)std::cout<<x<<' ';
        for(const V6* v:{&actor.offset,&actor.velocity,&actor.filtered})for(double x:*v)std::cout<<x<<' ';
        std::cout<<actor.recovery_memory<<'\n';
      }
      return 0;
    }
    if(argc!=1)throw std::runtime_error("offline tests only; no robot address or execute option accepted");
    tests();return 0;
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
