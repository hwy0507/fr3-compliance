// Full real-SDK entry, with a default OFF build interlock. No execution this turn.
#include "start_protocol.hpp"
#include <sys/mman.h>
#include <sys/resource.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

#ifndef FR3_NATIVE_EXECUTION_BUILD
#define FR3_NATIVE_EXECUTION_BUILD 0
#endif
using namespace fr3_native;
namespace {
std::atomic<bool> cancel{false};
static_assert(std::atomic<bool>::is_always_lock_free,"signal stop flag must be lock free");
void signal_stop(int){cancel.store(true,std::memory_order_relaxed);}
using Clock=std::chrono::steady_clock;
void claim_run(const std::filesystem::path& out) {
  if(!std::filesystem::is_directory(out)||std::filesystem::exists(out/"STOP"))
    throw std::runtime_error("invalid run directory / preexisting STOP");
  const auto file=out/"LIVE_STARTED";
  const int fd=open(file.c_str(),O_WRONLY|O_CREAT|O_EXCL|O_NOFOLLOW,0600);
  if(fd<0)throw std::runtime_error("run already used / cannot claim run");
  close(fd);
}
void terminal_confirm(const StartManifest& m) {
  if(!isatty(STDIN_FILENO)||!isatty(STDOUT_FILENO))throw std::runtime_error("attended terminal required");
  const std::string expected="START "+std::string(m.trial==Trial::HoldOnly?"HOLD ":m.trial==Trial::FreeGuiding?"DRAG ":"PUSH ")+m.session;
  std::cout<<"This will WRITE K=[300,300,800,50,50,50] and enter native control.\n"
    <<(m.review.end_policy==EndPolicy::RetainTrial?"End policy: RETAIN the trial stiffness; original and numeric collision values are unverified. Collision settings are NOT changed.\n":
       "End policy: attempt restoration of verified original stiffness.\n")
    <<"Software success-rate floor: "<<m.minimum_success<<"; max receive period "<<m.maximum_period*1000<<"ms; SDK errors remain enabled.\n"
    <<"Relative moment limit: "<<m.relative_moment_limit<<" Nm; absolute force/moment limits unchanged.\n"
    <<(m.operator_ended?"Operator-ended guiding: no countdown; STOP/terminal loss or a guard exits control.\n":
       "Session window: "+std::to_string(m.session_timeout)+" seconds; contact/recovery limited to 2 seconds.\n")
    <<"Fixed target does not mean no physical motion. STOP+Enter, Ctrl-C or the run STOP latch request exit.\n"
    <<"Keep clear. Confirm within 30 seconds by typing exactly:\n"<<expected<<std::endl;
  const auto deadline=Clock::now()+std::chrono::seconds(30);std::string answer;
  // Read exactly one line without stdio buffering: a pasted following STOP must
  // remain on the file descriptor for the supervisor, not hidden in cin's buffer.
  for(;;){
    if(cancel.load()||Clock::now()>=deadline)throw std::runtime_error("confirmation cancelled/timed out");
    pollfd input{STDIN_FILENO,POLLIN,0};const int ready=poll(&input,1,100);
    if(ready<0||input.revents&(POLLHUP|POLLERR|POLLNVAL))throw std::runtime_error("confirmation terminal lost");
    if(!(input.revents&POLLIN))continue;
    char c=0;if(read(STDIN_FILENO,&c,1)!=1)throw std::runtime_error("confirmation input ended");
    if(c=='\n')break;
    if(answer.size()>=100)throw std::runtime_error("confirmation too long");
    answer+=c;
  }
  if(answer!=expected)throw std::runtime_error("confirmation mismatch");
}
void run_live(const StartManifest& m,const std::filesystem::path& out) {
  m.validate(std::time(nullptr));terminal_confirm(m);m.validate(std::time(nullptr));claim_run(out);
  std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
  Session session;OperatorGate gate(m.trial,session,cancel,m.operator_ended);
  Proposal proposal;proposal.min_success=m.minimum_success;proposal.max_period=m.maximum_period;proposal.session_timeout=m.session_timeout;
  proposal.operator_ended=m.operator_ended;
  proposal.delta_moment_stop=m.relative_moment_limit;
  session.free_guiding_requested=m.trial==Trial::FreeGuiding;
  std::ofstream continuous_trace;
  if(m.operator_ended){
    session.trace.enable_streaming();continuous_trace.open(out/"wrench.csv");
    continuous_trace<<std::setprecision(17);fr3_trace::Trace::header(continuous_trace);continuous_trace.flush();
    if(!continuous_trace)throw std::runtime_error("cannot open continuous wrench log; control not started");
  }
  std::vector<Sample> baseline_samples(1001);
  std::vector<franka::RobotMode> modes(1001,franka::RobotMode::kOther);
  franka::RobotState expected{};size_t used=0;std::exception_ptr failure;
  std::atomic<bool> done{false};rusage before{},after{};bool fault_counts_valid=false;
  std::thread worker([&]{
    try {
      cpu_set_t set;CPU_ZERO(&set);CPU_SET(m.cpu,&set);
      if(sched_setaffinity(0,sizeof(set),&set))throw std::runtime_error("worker CPU binding failed");
      if(cancel.load())throw Abort(Fault::OperatorStop);
      franka::Robot robot(m.ip,franka::RealtimeConfig::kEnforce);auto model=robot.loadModel();
      expected=robot.readOnce();
      if(expected.robot_mode!=franka::RobotMode::kIdle||bool(expected.current_errors))throw Abort(Fault::Startup);
      for(int i=0;i<100;++i)(void)convert(expected,expected,robot,model);
      const auto begin=Clock::now();
      robot.read([&](const franka::RobotState& state){
        if(cancel.load())throw Abort(Fault::OperatorStop);
        if(Clock::now()-begin>std::chrono::seconds(3))throw std::runtime_error("baseline read deadline exceeded");
        if(state.robot_mode!=franka::RobotMode::kIdle)throw Abort(Fault::Startup);
        modes[used]=state.robot_mode;baseline_samples[used++]=convert(state,expected,robot,model);
        return used<baseline_samples.size();
      });
      const auto baseline=qualify_static_baseline(baseline_samples,modes,proposal);
      if(mlockall(MCL_CURRENT))throw std::runtime_error("memory locking failed");
      fault_counts_valid=getrusage(RUSAGE_THREAD,&before)==0;m.validate(std::time(nullptr));
      if(cancel.load())throw Abort(Fault::OperatorStop);
      run_reviewed_native_hold(robot,model,expected,baseline,m.review,proposal,cancel,session);
      fault_counts_valid=getrusage(RUSAGE_THREAD,&after)==0&&fault_counts_valid;
    }catch(...){fault_counts_valid=getrusage(RUSAGE_THREAD,&after)==0&&fault_counts_valid;failure=std::current_exception();}
    session.active.store(false);session.contact_enabled.store(false);done.store(true);
  });
  // This thread never constructs Robot and never calls its stop()/control().
  // Only it reads terminal/file commands and prints phase messages.
  auto last_update=Clock::now();auto heartbeat=session.heartbeat.load();
  std::string previous,input;const auto start=Clock::now();
  std::exception_ptr supervisor_failure;
  try{while(!done.load()) {
    const auto now=Clock::now();const auto next=session.heartbeat.load();
    if(next!=heartbeat){heartbeat=next;last_update=now;}
    const bool fresh=now-last_update<std::chrono::milliseconds(100);
    std::error_code ec;const bool stop_file=std::filesystem::exists(out/"STOP",ec);
    if(cancel.load()||stop_file||ec||(session.active.load()&&!fresh)||
       (!m.operator_ended&&std::chrono::duration<double>(now-start).count()>m.session_timeout+10))gate.stop();
    // Single non-realtime consumer; a full queue or failed log stops control.
    // No file IO, allocation or mutex is introduced in the SDK callback.
    if(m.operator_ended)session.trace.drain(continuous_trace,128);
    const auto status=gate.status(fresh);
    if(status!=previous){std::cout<<status<<std::endl;previous=status;}
    pollfd terminal{STDIN_FILENO,POLLIN,0};const int polled=poll(&terminal,1,20);
    if(polled<0){if(!cancel.load())gate.stop();continue;}
    if(terminal.revents&(POLLHUP|POLLERR|POLLNVAL)){gate.stop();continue;}
    if(terminal.revents&POLLIN) {
      char buffer[128];const auto n=read(STDIN_FILENO,buffer,sizeof(buffer));
      if(n<=0){gate.stop();continue;}
      for(ssize_t i=0;i<n;++i) {
        if(buffer[i]=='\n') {
          if(input=="STATUS")std::cout<<gate.status(fresh)<<std::endl;
          else if(((input=="PUSH"&&m.trial==Trial::LightPush)||(input=="DRAG"&&m.trial==Trial::FreeGuiding))&&!cancel.load()){
            if(!gate.permit_contact(fresh))std::cout<<"PUSH REJECTED: not Ready / hold-only / stopped"<<std::endl;
          }else gate.stop(); // STOP, q, empty Enter or unknown command all stop
          input.clear();
        }else if(input.size()<32)input+=buffer[i];else gate.stop();
      }
    }
  }}catch(...){supervisor_failure=std::current_exception();gate.stop();}
  worker.join(); // No forced process kill, automatic recovery or retry.
  if(!failure)failure=supervisor_failure;
  gate.stop(); // Withdraw all contact permission even on a normal finish.
  std::cout<<"CONTROL_RETURNED: contact forbidden; checking restoration and saving diagnostics"<<std::endl;
  bool sdk_failure=false;std::exception_ptr log_failure;
  if(failure)try{std::rethrow_exception(failure);}catch(const franka::ControlException& error){
    sdk_failure=true;try{fr3_trace::write_exception(out,error);}catch(...){log_failure=std::current_exception();}
  }catch(const std::exception&){}
  try{
    if(m.operator_ended){session.trace.drain(continuous_trace,session.trace.pending());continuous_trace.flush();
      if(!continuous_trace)throw std::runtime_error("continuous wrench log flush failed");}
    else session.trace.write(out/"wrench.csv");
  }catch(...){if(!log_failure)log_failure=std::current_exception();}
  std::ofstream receipt(out/"result.txt"),states(out/"preflight_state.txt"),base(out/"baseline.csv");
  receipt<<"control_entered="<<session.control_entered<<"\nstiffness_write_attempted="<<session.stiffness_write_attempted
    <<"\nstiffness_applied="<<session.stiffness_applied<<"\nrestore_attempted="<<session.restore_attempted
    <<"\nrestore_succeeded="<<session.restore_succeeded<<"\nsdk_control_exception="<<sdk_failure
    <<"\ntrial_stiffness_retained="<<session.trial_stiffness_retained
    <<"\nfree_guiding_requested="<<session.free_guiding_requested
    <<"\nminimum_success_rate="<<m.minimum_success
    <<"\nmaximum_callback_period_s="<<m.maximum_period
    <<"\nrelative_moment_limit_nm="<<m.relative_moment_limit
    <<"\nsession_timeout_s="<<m.session_timeout
    <<"\noperator_ended="<<m.operator_ended<<"\ntrace_rows="<<session.trace.size()
    <<"\ntrace_overflow="<<session.trace.overflowed()<<"\ntrace_pending="<<session.trace.pending()
    <<"\ncallback_total_count="<<session.callback_total_count
    <<"\ncallback_total_us="<<session.callback_total_us<<"\ncallback_max_us="<<session.callback_max_us
    <<"\ncallback_timing_rows="<<session.callback_time_count
    <<"\ncollision_settings_written=false\noriginal_stiffness_verified="<<m.review.previous_stiffness_verified
    <<"\ncollision_values_verified="<<m.review.existing_collision_protection_verified
    <<"\nfault="<<int(session.fault)<<"\nfault_reason="<<session.fault_reason<<"\nphase="<<session.phase.load()<<"\n"
    <<"worker_fault_counts_valid="<<fault_counts_valid
    <<"\nworker_minor_fault_delta="<<after.ru_minflt-before.ru_minflt<<"\nworker_major_fault_delta="<<after.ru_majflt-before.ru_majflt<<'\n';
  states<<std::setprecision(17)<<expected<<'\n';
  base<<std::setprecision(17)<<"time,raw_Fx,raw_Fy,raw_Fz,raw_Mx,raw_My,raw_Mz\n";
  for(size_t i=0;i<used;++i){base<<baseline_samples[i].time;for(double x:baseline_samples[i].raw_wrench)base<<','<<x;base<<'\n';}
  if(session.restoration_failure)try{std::rethrow_exception(session.restoration_failure);}catch(const std::exception& e){receipt<<"restoration_error="<<e.what()<<'\n';}
  if(failure)try{std::rethrow_exception(failure);}catch(const std::exception& e){receipt<<"primary_error="<<e.what()<<'\n';}
  if(supervisor_failure)try{std::rethrow_exception(supervisor_failure);}catch(const std::exception& e){receipt<<"supervisor_error="<<e.what()<<'\n';}
  if(log_failure)try{std::rethrow_exception(log_failure);}catch(const std::exception& e){receipt<<"log_error="<<e.what()<<'\n';}
  receipt.flush();states.flush();base.flush();
  if((!receipt||!states||!base)&&!log_failure)
    log_failure=std::make_exception_ptr(std::runtime_error("result write failed; inspect run and robot before reuse"));
  std::ofstream timing(out/"callback_timing_us.csv");timing<<std::setprecision(17)<<"index,callback_us\n";
  for(size_t i=0;i<session.callback_time_count;++i)timing<<i<<','<<session.callback_us[i]<<'\n';
  timing.flush();if(!timing&&!log_failure)log_failure=std::make_exception_ptr(std::runtime_error("callback timing log failed"));
  if(log_failure)std::cerr<<"Diagnostic output incomplete; inspect retained primary failure and robot before reuse"<<std::endl;
  if(failure)std::rethrow_exception(failure);
  if(log_failure)std::rethrow_exception(log_failure);
  if(m.review.end_policy==EndPolicy::RetainTrial){
    if(!session.trial_stiffness_retained)throw std::runtime_error("trial stiffness acknowledgement not confirmed");
    std::cout<<"CONTROL_EXITED: trial stiffness retained; collision settings unchanged; no automatic retry/return"<<std::endl;
  }else{
    if(!session.restore_succeeded)throw std::runtime_error("restoration not confirmed");
    std::cout<<"CONTROL_EXITED: restoration acknowledged; no automatic retry/return"<<std::endl;
  }
}
} // namespace
int main(int argc,char** argv) {
  try {
    if(argc==2&&std::string(argv[1])=="--help"){
      std::cout<<"--validate-manifest FILE (local only)\n--execute-reviewed FILE RUN_DIR (build default OFF, attended terminal required)\n";return 0;
    }
    if(argc==3&&std::string(argv[1])=="--validate-manifest") {
      std::ifstream in(argv[2]);read_manifest(in,std::time(nullptr));
      std::cout<<"MANIFEST_VALID; no robot connection or parameter writes"<<std::endl;return 0;
    }
    if(argc!=4||std::string(argv[1])!="--execute-reviewed")throw std::invalid_argument("explicit reviewed entry required");
    if(!FR3_NATIVE_EXECUTION_BUILD)throw std::runtime_error("execution build interlock OFF; no Robot constructed");
    std::ifstream in(argv[2]);const auto manifest=read_manifest(in,std::time(nullptr));
    run_live(manifest,argv[3]);return 0;
  }catch(const std::exception& e){std::cerr<<"BLOCKED/STOPPED: "<<e.what()<<std::endl;return 1;}
}
