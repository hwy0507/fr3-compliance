#pragma once
// Bounded append-only recording. No robot APIs, locks, allocation or file I/O in capture().
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <time.h>
#include <vector>
namespace esn_sync {
inline int64_t monotonic_ns() noexcept {timespec t{};if(clock_gettime(CLOCK_MONOTONIC,&t))return 0;return int64_t(t.tv_sec)*1000000000+t.tv_nsec;}
struct Sample {int64_t callback_ns=0,ready_ns=0;double robot_s=0,elapsed_s=0;std::array<double,48> obs{};std::array<double,128> state{};std::array<double,7> action{};};
class Recorder {
 std::filesystem::path path_;std::vector<Sample> rows_;size_t used_=0;bool overflow_=false;
public:
 explicit Recorder(const std::filesystem::path& p,size_t capacity=3000,bool synthetic=false):path_(p),rows_(capacity){
  std::string boot;{std::ifstream f("/proc/sys/kernel/random/boot_id");f>>boot;}
  if(boot.empty())throw std::runtime_error("Missing boot ID");
  if(std::filesystem::exists(p/"policy_clock.json")||std::filesystem::exists(p/"policy_sync.csv"))throw std::runtime_error("Sync output exists");
  std::ofstream f(p/"policy_clock.json");f<<"{\"schema\":\"FR3_POLICY_MONOTONIC_V1\",\"clock\":\"CLOCK_MONOTONIC\",\"boot_id\":\""<<boot<<"\",\"direct_reservoir_states\":true,\"synthetic_test_data\":"<<(synthetic?"true":"false")<<",\"timestamp_meaning\":\"host callback entry; not robot sensor exposure\"}\n";
  f.flush();if(!f)throw std::runtime_error("Cannot write policy clock metadata");
 }
 template<class O,class H,class A> void capture(int64_t entry,double robot,double elapsed,const O& obs,const H& state,const A& action) noexcept {
  if(used_==rows_.size()){overflow_=true;return;} // Diagnostic overflow never alters robot commands.
  auto& r=rows_[used_++];r.callback_ns=entry;r.ready_ns=monotonic_ns();r.robot_s=robot;r.elapsed_s=elapsed;
  for(int i=0;i<48;++i)r.obs[i]=obs[i];for(int i=0;i<128;++i)r.state[i]=state[i];for(int i=0;i<7;++i)r.action[i]=action[i];
 }
 void write() const {
  std::ofstream f(path_/"policy_sync.csv");f<<std::setprecision(17)<<"sample,callback_monotonic_ns,state_ready_monotonic_ns,robot_time_s,elapsed_s";
  for(int i=0;i<48;++i)f<<",obs"<<i;for(int i=0;i<128;++i)f<<",h"<<i;for(int i=0;i<7;++i)f<<",a"<<i;f<<'\n';
  for(size_t i=0;i<used_;++i){const auto& r=rows_[i];f<<i<<','<<r.callback_ns<<','<<r.ready_ns<<','<<r.robot_s<<','<<r.elapsed_s;for(double v:r.obs)f<<','<<v;for(double v:r.state)f<<','<<v;for(double v:r.action)f<<','<<v;f<<'\n';}
  f.flush();if(!f)throw std::runtime_error("Policy sync CSV write failed");
  std::ofstream s(path_/"policy_sync_status.json");s<<"{\"rows\":"<<used_<<",\"overflow\":"<<(overflow_?"true":"false")<<"}\n";
 }
 ~Recorder() noexcept {try{write();}catch(const std::exception& e){std::cerr<<"SYNC_LOG_FAILED: "<<e.what()<<'\n';}}
};
}
