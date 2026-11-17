// Pure mathematical regression; no Robot object is constructed.
#define main motion_entry_not_called
#include "fr3_taught_path.cpp"
#undef main
#include <cassert>
#include <limits>
template<class F>void rejects(F fn,const std::string& field){
  bool rejected=false;try{fn();}catch(const std::exception& e){
    rejected=true;assert(std::string(e.what()).find(field)!=std::string::npos);
  }assert(rejected);
}
int main(){
  Path p;p.duration=12;p.j1=-2.86;p.pieces.resize(1);
  auto& s=p.pieces[0];s.begin=0;s.end=12;
  Q start{-2.86,.3,2.5,-2.5,0,2,-2.2};
  for(size_t j=0;j<7;++j)s.c[j][5]=start[j];
  const double delta=.6;
  s.c[1][0]=6*delta/std::pow(12.,5);s.c[1][1]=-15*delta/std::pow(12.,4);s.c[1][2]=10*delta/std::pow(12.,3);
  close_array(evaluate(p,0),start,1e-12);
  auto goal=start;goal[1]+=.6;close_array(evaluate(p,12),goal,1e-12);
  assert(std::abs(evaluate(p,6)[1]-.6)<1e-12);
  assert(std::abs(evaluate(p,6,1)[1]-1.875*.6/12)<1e-12);
  for(int d:{1,2})for(double t:{0.,12.})close_array(evaluate(p,t,d),Q{},1e-12);
  auto corrected=start;corrected[0]+=.00003;corrected[1]+=.0007;
  close_array(command(p,corrected,0),corrected,1e-12);
  auto final=command(p,corrected,12);goal[0]=corrected[0];close_array(final,goal,1e-12);
  assert(command(p,corrected,.5)[0]==corrected[0]);
  bool rejected=false;try{read_path("/does-not-exist/offline-test");}catch(const std::exception&){rejected=true;}
  assert(rejected);
  franka::RobotState initial{};initial.q=start;initial.q_d=start;
  p.mass=initial.m_total;p.frame=initial.F_T_EE;p.com=initial.F_x_Ctotal;p.inertia=initial.I_total;
  auto first=initial;
  // Reproduce the old guard's false rejection if activation refreshes q_d,
  // while measured q, configuration and plan alignment all remain acceptable.
  initial.q_d[1]+=.0004;
  rejects([&]{close_array(first.q_d,initial.q_d,.0001);},"delta=");
  auto anchor=callback_start(first,initial,p);
  close_array(command(p,anchor,0),first.q_d,1e-12);
  auto bad=first;bad.q[1]+=.0011;
  rejects([&]{callback_start(bad,initial,p);},"start.q_d_vs_measured");
  bad=first;bad.q[1]+=.002;bad.q_d[1]+=.002;
  rejects([&]{callback_start(bad,initial,p);},"start.q_d_vs_plan");
  bad=first;bad.q[1]+=.0011;bad.q_d[1]+=.0005;
  rejects([&]{callback_start(bad,initial,p);},"first.q_vs_preflight_q");
  bad=first;bad.dq[1]=.011;
  rejects([&]{callback_start(bad,initial,p);},"start.dq");
  bad=first;bad.F_T_EE[12]+=.01;
  rejects([&]{callback_start(bad,initial,p);},"F_T_EE[12]");
  bad=first;bad.q_d[1]=std::numeric_limits<double>::quiet_NaN();
  rejects([&]{callback_start(bad,initial,p);},"start.q_d_vs_plan");
  Snapshots snapshots;snapshots.first=bad;snapshots.have_first=true;
  assert(std::isnan(snapshots.first.q_d[1])); // Rejected first frame remains available.
  p.hold_j1=false;
  p.pieces[0].c[0][0]=6*.1/std::pow(12.,5);
  p.pieces[0].c[0][1]=-15*.1/std::pow(12.,4);
  p.pieces[0].c[0][2]=10*.1/std::pow(12.,3);
  assert(std::abs(evaluate(p,12)[0]-start[0]-.1)<1e-12);
  close_array(command(p,corrected,0),corrected,1e-12);
  assert(std::abs(command(p,corrected,12)[0]-start[0]-.1)<1e-12);
  std::cout<<"Offline math, fixed/all-joint, stale-reference reproduction, startup guards and diagnostics tests passed.\n";
}
