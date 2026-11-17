#define EIGEN_RUNTIME_NO_MALLOC
#include "franka_session.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <chrono>
#include <limits>
using namespace fr3_esn;
size_t checks=0;
void require(bool ok,const char* what){++checks;if(!ok)throw std::runtime_error(what);}
template<class F> void rejects(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}require(caught,"expected rejection");}
EsnSample valid_sample() {
  EsnSample s;s.q<<0,-.7,0,-1.9,0,1.3,0;
  s.jacobian.block<6,6>(0,0)=M6::Identity();s.time=1;return s;
}
int main(int argc,char** argv) {
 try {
  if(argc!=4)throw std::runtime_error("offline_test ACTOR_FIXTURE OBSERVER_FIXTURE SUMMARY");
  Eigen::internal::set_is_malloc_allowed(false);
  Esn64 actor;ActionFilter filter;double actor_error=0,state_error=0,filter_error=0,max_us=0;
  std::ifstream input(argv[1]);size_t n=0;input>>n;require(input.good()&&n>1000,"actor fixture");
  for(size_t k=0;k<n;++k){int reset=0;input>>reset;Obs obs;V7 expected,ef;Eigen::Matrix<double,64,1> es;
    for(double& x:obs)input>>x;for(double& x:expected)input>>x;for(double& x:es)input>>x;for(double& x:ef)input>>x;
    require(input.good(),"truncated actor fixture");if(reset){actor.reset();filter=ActionFilter();}
    const auto begin=std::chrono::steady_clock::now();const V7 output=actor.step(obs);
    const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-begin).count();max_us=std::max(max_us,us);
    filter.update(output);V7 filtered;filtered<<filter.scale,filter.yield;
    actor_error=std::max(actor_error,(output-expected).cwiseAbs().maxCoeff());
    state_error=std::max(state_error,(actor.state-es).cwiseAbs().maxCoeff());
    filter_error=std::max(filter_error,(filtered-ef).cwiseAbs().maxCoeff());
    require(actor_error<1e-10&&state_error<1e-10&&filter_error<1e-10,"Python parity");
  }
  actor.reset();Obs context=Obs::Constant(.3);context.segment<6>(39).setZero();
  for(int i=0;i<100;++i)require(actor.step(context).norm()==0,"context-only neutral equilibrium");
  Obs bad=Obs::Zero();bad[2]=std::numeric_limits<double>::quiet_NaN();rejects([&]{actor.step(bad);});
  rejects([&]{actor.step(context,.001);});
  std::ifstream observer_input(argv[2]);size_t count=0;observer_input>>count;
  NativeObserver observer(V7::Zero(),V7::Constant(.1));double observer_error=0,wrench_error=0;
  for(size_t k=0;k<count;++k) {
    int reset=0;double dt;V7 velocity,bias,passive,motor,expected;M7 m;J67 j;V6 expected_wrench;
    observer_input>>reset>>dt;for(double& x:velocity)observer_input>>x;
    for(int r=0;r<7;++r)for(int c=0;c<7;++c)observer_input>>m(r,c);
    for(double& x:bias)observer_input>>x;for(double& x:passive)observer_input>>x;for(double& x:motor)observer_input>>x;
    for(double& x:expected)observer_input>>x;for(int r=0;r<6;++r)for(int c=0;c<7;++c)observer_input>>j(r,c);
    for(double& x:expected_wrench)observer_input>>x;require(observer_input.good(),"observer fixture");
    if(reset)observer=NativeObserver(V7::Zero(),V7::Constant(.1));
    const auto estimate=observer.update(velocity,m,bias,passive,motor,dt);
    observer_error=std::max(observer_error,(estimate-expected).cwiseAbs().maxCoeff());
    const V6 wrench=equivalent_wrench(j,estimate);
    wrench_error=std::max(wrench_error,(wrench-expected_wrench).cwiseAbs().maxCoeff());
    require(observer_error<1e-9&&wrench_error<1e-8,"observer/wrench parity");
  }
  rejects([&]{observer.update(V7::Zero(),-M7::Identity(),V7::Zero(),V7::Zero(),V7::Zero(),.001);});
  auto s=valid_sample();Reference hold;
  EsnCore core(hold,s.q,Eigen::Matrix3d::Identity());require(core.step(s,0).valid,"core start");
  for(int i=1;i<=4000;++i){s.time=1+i*.001;s.policy_load[0]=.1;s.policy_wrench[0]=.1;
    auto result=core.step(s,.001);require(result.valid,"bounded synthetic session");
    require(result.qdot.cwiseAbs().maxCoeff()<=.08+1e-10,"joint speed");
    require((s.jacobian*result.qdot).head<3>().norm()<=.03+1e-10,"tcp speed");
    require((result.torque-s.tau_previous).cwiseAbs().maxCoeff()<=.05+1e-10,"torque slew");
    s.tau_previous=result.torque;
  }
  require(core.policy_updates()==100,"exact 25Hz cadence");
  require(std::abs(core.observation()[32]-.1)<1e-12&&std::abs(core.observation()[39]-.1)<1e-12,"policy channels separate");
  // Each independent fault must latch even after the next sample returns to normal.
  for(int kind=0;kind<13;++kind){auto x=valid_sample();EsnCore c(hold,x.q,Eigen::Matrix3d::Identity());require(c.step(x,0).valid,"fault start");
    x.time+=.001;double dt=.001;
    switch(kind){case 0:x.collision=true;break;case 1:x.errors=true;break;case 2:x.configuration_ok=false;break;
      case 3:x.stop_requested=true;break;case 4:x.wrench[0]=13;break;case 5:x.raw_wrench[4]=1.1;break;
      case 6:x.dq[0]=.13;break;case 7:x.velocity_lower[0]=.001;break;case 8:x.policy_wrench[0]=NAN;break;
      case 9:x.time+=.01;break;case 10:x.jacobian.setZero();break;case 11:x.tau_measured[6]=10;break;
      case 12:x.position[0]=.05;break;}
    require(!c.step(x,dt).valid,"fault rejected");x=valid_sample();x.time+=.002;require(!c.step(x,.001).valid,"latched fault");
  }
  auto x=valid_sample();x.raw_wrench[0]=2;EsnCore biased(hold,x.q,Eigen::Matrix3d::Identity());
  require(!biased.step(x,0).valid,"SDK bias not hidden by zero policy observer");
  // Bounded contact continues; collision never becomes an allowed contact.
  { auto x=valid_sample();EsnCore c(hold,x.q,Eigen::Matrix3d::Identity());require(c.step(x,0).valid,"contact start");
    for(int i=1;i<=1000;++i){x.time=1+i*.001;x.contact=i<=200;x.wrench[0]=i<=200?3:0;
      auto r=c.step(x,.001);require(r.valid,"bounded contact allowed");
      if(i==200)require(r.phase==Phase::Contact,"contact entered");
      if(i==1000)require(r.phase==Phase::Free,"release dwell completed");x.tau_previous=r.torque;}
  }
  { auto x=valid_sample();Limits limits;limits.contact_timeout=.1;EsnCore c(hold,x.q,Eigen::Matrix3d::Identity(),limits);
    require(c.step(x,0).valid,"timeout start");Result r;
    for(int i=1;i<=101;++i){x.time=1+i*.001;x.contact=true;r=c.step(x,.001);x.tau_previous=r.torque;}
    require(!r.valid&&r.fault==Fault::Timeout,"contact deadline");
  }
  { auto x=valid_sample();EsnCore c(hold,x.q,Eigen::Matrix3d::Identity());require(c.step(x,0).valid,"reference start");
    Reference next=hold;next.position[0]=.01;x.time+=.001;require(c.step(x,.001,next).fault==Fault::Reference,"reference jump");}
  { auto x=valid_sample();EsnCore c(hold,x.q,Eigen::Matrix3d::Identity());require(c.step(x,0).valid,"moving reference start");
    for(int i=1;i<=1000;++i){Reference next=hold;next.position[0]=.001*i*.001;next.feedforward[0]=.001;next.direction[0]=1;
      x.time=1+i*.001;auto r=c.step(x,.001,next);require(r.valid,"bounded reference increment");x.tau_previous=r.torque;}
    require(c.observation()[45]==1,"path intent channel");}
  EsnReview review;rejects([&]{review.validate();});ObserverCalibration calibration;rejects([&]{calibration.validate();});
  Eigen::internal::set_is_malloc_allowed(true);
  std::ofstream out(argv[3]);out<<std::setprecision(17)<<"{\"checks\":"<<checks<<",\"actor_frames\":"<<n
    <<",\"actor_max_abs\":"<<actor_error<<",\"reservoir_max_abs\":"<<state_error<<",\"filter_max_abs\":"<<filter_error
    <<",\"observer_frames\":"<<count<<",\"observer_max_abs\":"<<observer_error<<",\"wrench_max_abs\":"<<wrench_error
    <<",\"actor_observed_max_us\":"<<max_us<<",\"no_eigen_heap_allocation\":true,\"hardware_executed\":false}\n";
  std::cout<<"Offline ESN checks passed: "<<checks<<"\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
