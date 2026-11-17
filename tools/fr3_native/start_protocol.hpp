#pragma once
#include "operator_gate.hpp"
#include <arpa/inet.h>
#include <sched.h>
#include <cctype>
#include <ctime>
#include <istream>

namespace fr3_native {
struct StartManifest {
  std::string ip,session,stiffness_digest,collision_digest;
  std::string preflight_digest,authorization_digest;
  Trial trial=Trial::HoldOnly;int cpu=-1;bool contact_authorized=false;long long issued=0,expires=0;Review review;
  double minimum_success=.99;
  double maximum_period=.002;
  double session_timeout=15;
  bool operator_ended=false;
  double relative_moment_limit=.25;
  static bool hex(const std::string& s,size_t n){
    if(s.size()!=n)return false;
    for(unsigned char c:s)if(!std::isxdigit(c))return false;
    return true;
  }
  void validate(long long now) const {
    review.validate();in_addr addr{};
    if((relative_moment_limit!=.25&&
        (!((relative_moment_limit==.5&&review.guiding_moment_05_reviewed)||
           (relative_moment_limit==1&&review.guiding_moment_10_reviewed))||!operator_ended||trial!=Trial::FreeGuiding))||
       (operator_ended&&(!review.operator_ended_authorized||trial!=Trial::FreeGuiding||review.end_policy!=EndPolicy::RetainTrial))||
       (session_timeout!=15&&session_timeout!=60)||
       (session_timeout>15&&(!review.extended_session_reviewed||review.end_policy!=EndPolicy::RetainTrial))||
       (maximum_period!=.002&&maximum_period!=.005)||
       (maximum_period>.002&&(!review.native_gap_5ms_reviewed||review.end_policy!=EndPolicy::RetainTrial))||
       (minimum_success!=.99&&minimum_success!=.95&&minimum_success!=.90)||
       (minimum_success==.95&&(!review.minimum_success_95_authorized||review.end_policy!=EndPolicy::RetainTrial))||
       (minimum_success==.90&&(!review.minimum_success_90_authorized||review.end_policy!=EndPolicy::RetainTrial))||
       (trial!=Trial::HoldOnly&&!contact_authorized)||
       (trial==Trial::FreeGuiding&&(!review.free_guiding_authorized||review.end_policy!=EndPolicy::RetainTrial))||
       inet_pton(AF_INET,ip.c_str(),&addr)!=1||cpu<0||cpu>=CPU_SETSIZE||
       issued<=0||issued>now||expires<=now||expires-issued>600||expires-now<session_timeout+15||
       !hex(session,32)||
       (review.end_policy==EndPolicy::RestoreOriginal?(!hex(stiffness_digest,64)||!hex(collision_digest,64)):
        (!hex(preflight_digest,64)||!hex(authorization_digest,64))))
      throw std::invalid_argument("invalid/expired manifest; no robot connection allowed");
  }
};
inline StartManifest read_manifest(std::istream& in,long long now) {
  StartManifest m;std::string schema;int trial=-1,contact=-1;
  in>>schema>>m.ip>>trial>>m.cpu>>contact>>m.issued>>m.expires>>m.session;
  const bool moment05=schema=="FR3_NATIVE_OPERATOR_MOMENT05_V1";
  const bool moment10=schema=="FR3_NATIVE_OPERATOR_MOMENT10_V1";
  const bool operator_ended=schema=="FR3_NATIVE_OPERATOR_V1"||moment05||moment10;
  const bool extended=schema=="FR3_NATIVE_RETAIN_90_GAP5_LONG_V1";
  const bool gap5=schema=="FR3_NATIVE_RETAIN_90_GAP5_V1"||extended||operator_ended;
  if((schema!="FR3_NATIVE_START_V1"&&schema!="FR3_NATIVE_RETAIN_V1"&&schema!="FR3_NATIVE_RETAIN_95_V1"&&schema!="FR3_NATIVE_RETAIN_90_V1"&&!gap5)||(trial<0||trial>2)||(contact!=0&&contact!=1))throw std::invalid_argument("unknown start schema/mode");
  m.trial=static_cast<Trial>(trial);
  m.contact_authorized=contact==1;
  if(schema=="FR3_NATIVE_RETAIN_V1"||schema=="FR3_NATIVE_RETAIN_95_V1"||schema=="FR3_NATIVE_RETAIN_90_V1"||gap5){
    m.review.end_policy=EndPolicy::RetainTrial;
    m.minimum_success=(schema=="FR3_NATIVE_RETAIN_90_V1"||gap5)?.90:schema=="FR3_NATIVE_RETAIN_95_V1"?.95:.99;
    m.review.minimum_success_95_authorized=schema=="FR3_NATIVE_RETAIN_95_V1";
    m.review.minimum_success_90_authorized=schema=="FR3_NATIVE_RETAIN_90_V1"||gap5;
    m.maximum_period=gap5?.005:.002;m.review.native_gap_5ms_reviewed=gap5;
    m.session_timeout=extended?60:15;m.review.extended_session_reviewed=extended;
    m.operator_ended=operator_ended;m.review.operator_ended_authorized=operator_ended;
    m.relative_moment_limit=moment10?1:moment05?.5:.25;m.review.guiding_moment_05_reviewed=moment05;
    m.review.guiding_moment_10_reviewed=moment10;
    m.review.free_guiding_authorized=m.trial==Trial::FreeGuiding&&m.contact_authorized;
    for(bool* flag:{&m.review.motion_authorized,&m.review.stiffness_write_authorized,&m.review.static_baseline_conditions_reviewed,
        &m.review.onsite_ready,&m.review.retain_trial_authorized,&m.review.collision_settings_preserved,
        &m.review.collision_values_unverified,&m.review.tool_configuration_and_static_reviewed,&m.review.host_realtime_and_stop_reviewed}){
      int bit=-1;in>>bit;if(bit!=1)throw std::invalid_argument("retain review incomplete");*flag=true;
    }
  }else{
   for(bool* flag:{&m.review.motion_authorized,&m.review.stiffness_write_authorized,&m.review.restore_authorized,
      &m.review.previous_stiffness_verified,&m.review.static_baseline_verified,&m.review.tool_and_frames_verified,
      &m.review.existing_collision_protection_verified,&m.review.realtime_and_stopping_verified,&m.review.onsite_ready}) {
    int bit=-1;in>>bit;if(bit!=1)throw std::invalid_argument("review incomplete; no robot connection allowed");*flag=true;
  }
  for(double& k:m.review.previous_stiffness)in>>k;
  }
  if(m.review.end_policy==EndPolicy::RetainTrial)in>>m.preflight_digest>>m.authorization_digest;
  else in>>m.stiffness_digest>>m.collision_digest;
  if(!in)throw std::invalid_argument("truncated manifest");
  std::string extra;if(in>>extra)throw std::invalid_argument("unexpected manifest field");
  m.validate(now);return m;
}
} // namespace fr3_native
