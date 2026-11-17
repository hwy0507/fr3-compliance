// Bounded SPSC logging regression; synthetic RobotState only, no Robot.
#include "../fr3_wrench_trace.hpp"
#include <sstream>
#include <thread>
#include <iostream>
void need(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){try{
  franka::RobotState s;
  {fr3_trace::Trace old(2);need(old.capture(s,0)&&old.capture(s,.001),"linear capture");
    need(!old.capture(s,.001)&&old.size()==2&&old.overflowed(),"linear overflow remains latched");}
  {fr3_trace::Trace t(2);t.enable_streaming();std::ostringstream out;
    for(int i=0;i<10;++i){need(t.capture(s,.001),"ring wrap");need(t.drain(out,1)==1,"drain one");}
    need(t.size()==10&&t.pending()==0&&!t.overflowed(),"wrap loses no rows");
    need(t.capture(s,.001)&&t.capture(s,.001)&&!t.capture(s,.001)&&t.overflowed(),"full queue fails closed");}
  {fr3_trace::Trace t(2);t.enable_streaming();need(t.capture(s,.001),"IO failure capture");
    std::ostringstream broken;broken.setstate(std::ios::badbit);bool failed=false;
    try{t.drain(broken,1);}catch(const std::runtime_error&){failed=true;}
    need(failed&&t.pending()==1,"failed write does not discard unpublished row");}
  {constexpr size_t count=20000;fr3_trace::Trace t(count);t.enable_streaming();std::atomic<bool> done{false};
    std::thread producer([&]{franka::RobotState state;for(size_t i=0;i<count;++i){state.q[0]=double(i);
      if(!t.capture(state,.001))std::terminate();}done.store(true);});
    std::ostringstream out;
    while(!done.load()||t.pending()){t.drain(out,128);std::this_thread::yield();}producer.join();
    std::istringstream rows(out.str());std::string row;size_t seen=0;
    while(std::getline(rows,row)){need(row.rfind(std::to_string(seen)+",",0)==0,"SPSC row order");++seen;}
    need(seen==count&&t.size()==count&&!t.overflowed(),"SPSC all rows retained");}
  std::cout<<"PASS linear/wrap/overflow/IO-failure/concurrent 20000-row streaming trace; no Robot\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
