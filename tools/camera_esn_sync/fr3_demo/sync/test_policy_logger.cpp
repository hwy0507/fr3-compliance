// Model/clock/buffer regression only. No libfranka or robot connection.
#include "policy_sync.hpp"
#include "esn_actor.hpp"
#include <algorithm>
#include <sstream>
#include <chrono>
int main(int argc,char** argv){
 if(argc!=3)return 2;
 std::filesystem::create_directory(argv[2]);
 esn_sync::Recorder recorder(argv[2],3000,true);
 fr3_esn128::Esn128 policy;
 std::ifstream f(argv[1]);std::string line;std::getline(f,line);
 size_t count=0;double worst=0;int64_t slowest=0;
 while(std::getline(f,line)){
  std::replace(line.begin(),line.end(),',',' ');std::istringstream row(line);
  double t;fr3_esn128::Obs obs;fr3_esn128::V7 logged;row>>t;for(auto& v:obs)row>>v;for(auto& v:logged)row>>v;if(!row)return 3;
  const auto start=esn_sync::monotonic_ns();const auto action=policy.step(obs,.04);
  const auto a=esn_sync::monotonic_ns();recorder.capture(start,t,t,obs,policy.state,action);const auto b=esn_sync::monotonic_ns();
  slowest=std::max(slowest,b-a);worst=std::max(worst,(action-logged).cwiseAbs().maxCoeff());++count;
 }
 std::cout<<std::setprecision(17)<<"{\"rows\":"<<count<<",\"max_action_error\":"<<worst<<",\"max_capture_ns_offline_only\":"<<slowest<<",\"robot_connected\":false,\"synthetic_clock_experiment\":true}\n";
 return count&&worst<.002?0:1;
}
