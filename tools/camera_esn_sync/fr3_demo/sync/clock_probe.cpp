// Synthetic 25 Hz timing probe, with no robot/library/network dependency.
#include "policy_sync.hpp"
#include <thread>
#include <chrono>
int main(int argc,char** argv){
 if(argc!=2)return 2;
 if(!std::filesystem::create_directory(argv[1]))return 3;
 esn_sync::Recorder r(argv[1],3000,true);
 const auto start=std::chrono::steady_clock::now();
 std::array<double,48> obs{};std::array<double,128> h{};std::array<double,7> a{};
 for(int i=0;i<150;++i){
  std::this_thread::sleep_until(start+std::chrono::milliseconds(i*40));
  const int64_t entry=esn_sync::monotonic_ns();
  r.capture(entry,0,i*.04,obs,h,a);
 }
 std::cout<<"SYNTHETIC CLOCK PROBE ONLY: 150 samples; no robot connected\n";
}
