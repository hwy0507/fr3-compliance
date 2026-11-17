// Synthetic operator/start protocol checks. Never instantiates Robot.
#include "start_protocol.hpp"
#include <iostream>
#include <sstream>
using namespace fr3_native;
namespace {
int checks=0;
void need(bool x,const char* message){++checks;if(!x)throw std::runtime_error(message);}
template<class F>void rejected(F action){bool failed=false;try{action();}catch(const std::exception&){failed=true;}need(failed,"must fail closed");}
std::string manifest(){return "FR3_NATIVE_START_V1\n192.0.2.1\n0\n2\n0\n1000\n1300\n"+std::string(32,'a')+
 "\n1 1 1 1 1 1 1 1 1\n1000 1000 1000 100 100 100\n"+std::string(64,'b')+"\n"+std::string(64,'c')+"\n";}
void tests(){
  {const auto text="FR3_NATIVE_OPERATOR_MOMENT10_V1\n192.0.2.1\n2\n21\n1\n1000\n1300\n"+std::string(32,'a')+
     "\n1 1 1 1 1 1 1 1 1\n"+std::string(64,'b')+"\n"+std::string(64,'c')+"\n";
    std::istringstream in(text);auto m=read_manifest(in,1100);
    need(m.relative_moment_limit==1&&m.review.guiding_moment_10_reviewed,"separate 1Nm candidate manifest");
    m.review.guiding_moment_10_reviewed=false;m.review.guiding_moment_05_reviewed=true;
    rejected([&]{m.validate(1100);});m.review.guiding_moment_10_reviewed=true;
    m.relative_moment_limit=2;rejected([&]{m.validate(1100);});}
  {const auto text="FR3_NATIVE_OPERATOR_MOMENT05_V1\n192.0.2.1\n2\n21\n1\n1000\n1300\n"+std::string(32,'a')+
     "\n1 1 1 1 1 1 1 1 1\n"+std::string(64,'b')+"\n"+std::string(64,'c')+"\n";
    std::istringstream in(text);auto m=read_manifest(in,1100);
    need(m.relative_moment_limit==.5&&m.review.guiding_moment_05_reviewed,"separate candidate manifest");
    m.review.guiding_moment_05_reviewed=false;rejected([&]{m.validate(1100);});
    m.review.guiding_moment_05_reviewed=true;m.trial=Trial::LightPush;rejected([&]{m.validate(1100);});}
  {const auto text="FR3_NATIVE_OPERATOR_V1\n192.0.2.1\n2\n21\n1\n1000\n1300\n"+std::string(32,'a')+
     "\n1 1 1 1 1 1 1 1 1\n"+std::string(64,'b')+"\n"+std::string(64,'c')+"\n";
    std::istringstream in(text);auto m=read_manifest(in,1100);
    need(m.operator_ended&&m.review.operator_ended_authorized,"explicit operator-ended manifest");
    m.trial=Trial::HoldOnly;rejected([&]{m.validate(1100);});m.trial=Trial::FreeGuiding;
    m.review.operator_ended_authorized=false;rejected([&]{m.validate(1100);});}
  {const auto text="FR3_NATIVE_RETAIN_V1\n192.0.2.1\n0\n2\n0\n1000\n1300\n"+std::string(32,'a')+
     "\n1 1 1 1 1 1 1 1 1\n"+std::string(64,'b')+"\n"+std::string(64,'c')+"\n";
    std::istringstream in(text);auto m=read_manifest(in,1100);
    need(m.review.end_policy==EndPolicy::RetainTrial,"explicit retain schema");
    need(!m.review.previous_stiffness_verified&&!m.review.existing_collision_protection_verified,"unknown values never relabelled");
    m.review.collision_settings_preserved=false;rejected([&]{m.validate(1100);});}
  {auto text="FR3_NATIVE_RETAIN_V1\n192.0.2.1\n2\n2\n1\n1000\n1300\n"+std::string(32,'a')+
     "\n1 1 1 1 1 1 1 1 1\n"+std::string(64,'b')+"\n"+std::string(64,'c')+"\n";
    std::istringstream in(text);auto m=read_manifest(in,1100);
    need(m.trial==Trial::FreeGuiding&&m.review.free_guiding_authorized,"separate free-guiding trial mode");
    m.contact_authorized=false;rejected([&]{m.validate(1100);});}
  {std::istringstream input(manifest());const auto m=read_manifest(input,1100);need(m.trial==Trial::HoldOnly,"hold default");
    for(long long time:{999,1280,1300})rejected([&]{m.validate(time);});}
  for(const auto& bad:{std::string{},std::string("FR3_NATIVE_START_V1"),manifest()+"extra"})
    rejected([&]{std::istringstream in(bad);read_manifest(in,1100);});
  {auto text=manifest();text.replace(text.find("1 1 1 1"),1,"0");rejected([&]{std::istringstream in(text);read_manifest(in,1100);});}
  {std::istringstream in(manifest());auto m=read_manifest(in,1100);m.trial=Trial::LightPush;
    rejected([&]{m.validate(1100);});m.contact_authorized=true;m.validate(1100);need(true,"separate contact authority");}
  {Session session;std::atomic<bool> cancel{false};OperatorGate gate(Trial::HoldOnly,session,cancel);
    need(!gate.permit_contact(true),"inactive hold cannot arm");session.active.store(true);session.phase.store(int(Phase::Ready));
    need(gate.status(true).find("READY_HOLD")==0,"hold ready has no push permission");
    need(!gate.permit_contact(true)&&!session.contact_enabled.load(),"hold can never arm contact");
    rejected([&]{enforce_operator_contact({true,false,Fault::None,Phase::Contact,4},session);});
    need(session.phase.load()==int(Phase::Stopped),"unarmed contact withdraws Ready");}
  {Session session;std::atomic<bool> cancel{false};OperatorGate gate(Trial::LightPush,session,cancel);
    session.active.store(true);session.phase.store(int(Phase::Settling));
    need(!gate.permit_contact(true),"settling cannot arm");
    session.phase.store(int(Phase::Ready));need(gate.status(true).find("READY_TO_ARM")==0,"Ready alone not armed");
    need(gate.status(false).find("NOT_READY")==0,"stale Ready withdrawn");
    need(!gate.permit_contact(false),"stale Ready cannot arm contact");
    need(gate.permit_contact(true)&&session.contact_enabled.load(),"fresh Ready plus explicit PUSH");
    enforce_operator_contact({true,false,Fault::None,Phase::Contact,4},session);need(true,"armed contact accepted");
    session.phase.store(int(Phase::Contact));need(!gate.permit_contact(true),"cannot rearm while in contact");
    gate.stop();need(cancel.load()&&!session.contact_enabled.load(),"stop latch revokes contact");
    session.phase.store(int(Phase::Ready));need(!gate.permit_contact(true),"stop cannot reset/restart");
    need(gate.status(true).find("STOP_REQUESTED")==0,"stop dominates Ready");}
  std::cout<<"PASS "<<checks<<" synthetic start/operator checks; no Robot or setters\n";
}
}
int main(){try{tests();return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
