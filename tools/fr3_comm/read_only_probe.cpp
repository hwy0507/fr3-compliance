// Bounded state reception/model-cost probe. No control/write/recovery/gripper API.
#include "../fr3_native/native_hold.hpp"
#include <sys/mman.h>
#include <sched.h>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {
volatile std::sig_atomic_t cancelled=0;
void stop(int){cancelled=1;}
using Clock=std::chrono::steady_clock;
struct Row {double robot=0,host=0,cost_us=0;int cpu=0;};
double seconds(Clock::time_point x){return std::chrono::duration<double>(x.time_since_epoch()).count();}
}
int main(int argc,char** argv) {
  if(argc!=5||std::string(argv[1])!="--read-only"){
    std::cerr<<"Usage: --read-only IP NEW_DIRECTORY CPU(-1=unbound)\n";return 2;
  }
  std::signal(SIGINT,stop);std::signal(SIGTERM,stop);
  const std::filesystem::path out(argv[3]);
  bool created=false;std::vector<Row> rows(5000);size_t count=0;
  std::string status="not_started";int rc=1;bool locked=false;int policy=-1,priority=-1;
  try {
    const int cpu=std::stoi(argv[4]);
    if(cpu < -1||cpu>=CPU_SETSIZE)throw std::invalid_argument("invalid CPU");
    if(!std::filesystem::create_directory(out))throw std::runtime_error("output exists");
    created=true;
    if(cpu>=0){cpu_set_t set;CPU_ZERO(&set);CPU_SET(cpu,&set);
      if(sched_setaffinity(0,sizeof(set),&set))throw std::runtime_error("CPU affinity failed");}
    // Only current process memory. Automatically released on process exit.
    locked=mlockall(MCL_CURRENT)==0;
    if(!locked)throw std::runtime_error("memory locking failed");
    franka::Robot robot(argv[2],franka::RealtimeConfig::kEnforce);
    auto model=robot.loadModel();const auto expected=robot.readOnce();
    if(expected.robot_mode!=franka::RobotMode::kIdle)throw std::runtime_error("requires Idle");
    // Warm local model/guard conversion before the timed reception loop.
    for(int i=0;i<100;++i)(void)fr3_native::convert(expected,expected,robot,model);
    const auto begin=Clock::now();bool have_first=false;double first_robot=0;
    robot.read([&](const franka::RobotState& s){
      const auto entry=Clock::now();
      if(cancelled)throw std::runtime_error("operator stopped read probe");
      if(s.robot_mode!=franka::RobotMode::kIdle||bool(s.current_errors))
        throw std::runtime_error("state not stationary Idle / error observed; no recovery");
      for(double v:s.dq)if(!std::isfinite(v)||std::abs(v)>.02)
        throw std::runtime_error("motion observed; no commands sent");
      if(entry-begin>std::chrono::seconds(10))throw std::runtime_error("read deadline exceeded");
      if(!have_first){first_robot=s.time.toSec();have_first=true;}
      if(s.time.toSec()-first_robot>8)throw std::runtime_error("robot timestamp deadline exceeded");
      // The same model + wrench/frame/velocity conversion used by native_hold.
      // No control guard/session is armed; actual Idle state is not modified.
      const auto sample=fr3_native::convert(s,expected,robot,model);
      if(!sample.configuration_ok)throw std::runtime_error("tool/frames changed");
      const auto finished=Clock::now();
      rows[count++]={s.time.toSec(),seconds(entry),
        std::chrono::duration<double,std::micro>(finished-entry).count(),sched_getcpu()};
      return count<rows.size();
    });
    policy=sched_getscheduler(0);sched_param sp{};
    if(sched_getparam(0,&sp))throw std::runtime_error("priority query failed");
    priority=sp.sched_priority;status="read_only_completed";rc=0;
  }catch(const std::exception& e){status=e.what();std::cerr<<status<<'\n';}
  if(created){
    std::ofstream csv(out/"reception.csv"),info(out/"status.txt");csv<<std::setprecision(17);
    csv<<"robot_time_s,host_monotonic_s,conversion_us,cpu\n";
    for(size_t i=0;i<count;++i)csv<<rows[i].robot<<','<<rows[i].host<<','<<rows[i].cost_us<<','<<rows[i].cpu<<'\n';
    info<<status<<"\nread_only=true\nmotion_commands=false\nparameter_writes=false\n"
      <<"mlock_current="<<locked<<"\npolicy="<<policy<<"\npriority="<<priority<<"\nrows="<<count<<'\n';
    if(!csv||!info)return 3;
  }
  return rc;
}
