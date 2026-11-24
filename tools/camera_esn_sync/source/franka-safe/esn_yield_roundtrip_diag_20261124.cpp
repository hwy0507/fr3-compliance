// esn_yield.cpp — ESN 完整接管模式：减速 + 六维让位
//
// 说明: 在减速基础上，把 a[1:7] 的让位通道真正施加到末端：
//   平移最多 ±0.32 m、转动最多 ±0.6 rad，通过雅可比伪逆转成关节偏移。
//   1kHz 低通 + 关节偏移限幅（每关节 ≤ 0.25 rad）。
//
// 原 esn_slowdown 模式:
//   机械臂按名义轨迹走，但"推进速度"由策略的 a[0] 调制。
//   a0=0 → 全速；a0=1 → 20% 速度。让位通道只打印，不施加。
//
// 用法:
//   ./esn_shadow <IP> <起点姿态> <终点姿态> [峰值速度rad/s]
// 例:
//   ./esn_shadow 172.16.0.2 home pos2 0.12
//
// 行为:
//   1. 关节空间余弦轨迹从起点到终点，但推进用一个"虚拟时间" tau
//   2. 每 40 ms 调用一次 Esn128，得到 a0
//   3. tau 的推进速率 = dt * (1 - 0.8*a0) → 被推时自动变慢
//   4. 让位通道（a1..a6）只打印，不施加（后续版本再启用）
//
// 观测组装（canonical48，下标从 0 开始）:
//    0-6   q                 7-13  dq
//   14-16  名义末端线速度     17-19 名义角速度
//   20-22  位置误差(名义-实测) 23-25 旋转误差
//   26-28  名义-实测线速度    29-31 名义-实测角速度
//   32-38  tau_ext_hat_filtered
//   39-44  O_F_ext_hat_K
//   45-47  路径方向单位向量 (XY)
//
// ⚠️ 坐标系/符号/TCP 需与训练一致，本程序按如下约定实现，需在影子模式里核对:
//   - 位姿来自 Frame::kEndEffector（含 F_T_EE 的 0.255 m 偏移）
//   - 位置误差 = 名义 - 实测；速度误差 = 名义 - 实测
//   - 路径方向 = 名义末端位置在 XY 平面上的运动方向

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include <optional>
#include "fr3_wrench_trace.hpp"
#include <csignal>
#include <franka/rate_limiting.h>
#include "esn_roundtrip_reference.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "../FR3_ESN128_main/fr3_esn128/esn_actor.hpp"

namespace {

struct RawRow {double elapsed=0,tau=0,dt=0,compute_us=0;std::array<double,7> command{},reference{},offset{},offset_velocity{},target{};};
struct Diagnostics {
 std::filesystem::path out;fr3_trace::Trace trace;std::vector<RawRow> rows;size_t used=0;
 explicit Diagnostics(const std::filesystem::path& p):out(p),trace(92000),rows(92000){}
 void raw(double t,double tau,double dt,const std::array<double,7>& cmd,const std::array<double,7>& ref,const Eigen::Matrix<double,7,1>& offset,const Eigen::Matrix<double,7,1>& velocity,const Eigen::Matrix<double,7,1>& target,double us){
  if(used==rows.size())throw std::runtime_error("diagnostic raw capacity");
  auto& x=rows[used++];x.elapsed=t;x.tau=tau;x.dt=dt;x.command=cmd;x.reference=ref;x.compute_us=us;
  for(int j=0;j<7;++j){x.offset[j]=offset[j];x.offset_velocity[j]=velocity[j];x.target[j]=target[j];}
 }
 ~Diagnostics() noexcept{
  try{
   trace.write(out/"callback_states.csv");
   std::ofstream f(out/"raw_commands.csv");f<<std::setprecision(17)<<"index,elapsed_s,virtual_time_s,dt,compute_us";
   for(auto group:{"raw_command","nominal_reference","offset","offset_velocity","target"})for(int j=0;j<7;++j)f<<','<<group<<'_'<<j;f<<'\n';
   for(size_t i=0;i<used;++i){const auto& x=rows[i];f<<i<<','<<x.elapsed<<','<<x.tau<<','<<x.dt<<','<<x.compute_us;for(const auto* a:{&x.command,&x.reference,&x.offset,&x.offset_velocity,&x.target})for(auto v:*a)f<<','<<v;f<<'\n';}
   f.flush();if(!f)throw std::runtime_error("diagnostic raw write");
  }catch(const std::exception& e){std::cerr<<"Diagnostic write failed: "<<e.what()<<'\n';}
 }
};
volatile std::sig_atomic_t stop_requested=0;
void stop_signal(int){stop_requested=1;}
constexpr double kPolicyDt = 0.04;   // 25 Hz
constexpr double kHoldSeconds = 1.0;

std::string posePath(const std::string& name) {
  const char* h = std::getenv("HOME");
  return std::string(h ? h : ".") + "/franka-poses/" + name + ".json";
}

bool parseQ(const std::string& text, std::array<double, 7>* q) {
  size_t pos = text.find("\"q\"");
  if (pos == std::string::npos) return false;
  pos = text.find('[', pos);
  if (pos == std::string::npos) return false;
  size_t end = text.find(']', pos);
  if (end == std::string::npos) return false;
  std::string body = text.substr(pos + 1, end - pos - 1);
  for (char& c : body) {
    if (c == ',') c = ' ';
  }
  std::istringstream iss(body);
  for (size_t i = 0; i < 7; ++i) {
    if (!(iss >> (*q)[i])) return false;
  }
  return true;
}

bool loadQ(const std::string& name, std::array<double, 7>* q) {
  std::ifstream in(posePath(name));
  if (!in) return false;
  std::stringstream buf;
  buf << in.rdbuf();
  return parseQ(buf.str(), q);
}

// 4x4 列主序 -> 位置与旋转
struct Pose {
  Eigen::Vector3d p;
  Eigen::Matrix3d R;
};

Pose toPose(const std::array<double, 16>& T) {
  Pose out;
  out.p << T[12], T[13], T[14];
  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 3; ++r) {
      out.R(r, c) = T[c * 4 + r];
    }
  }
  return out;
}

// 旋转矩阵差 -> 角速度向量 (rad/s)
Eigen::Vector3d angularVelocity(const Eigen::Matrix3d& R_prev,
                                const Eigen::Matrix3d& R_now, double dt) {
  Eigen::Matrix3d dR = R_now * R_prev.transpose();
  Eigen::AngleAxisd aa(dR);
  return aa.axis() * aa.angle() / dt;
}

// 旋转误差 (名义 vs 实测) -> 轴角向量
Eigen::Vector3d rotationError(const Eigen::Matrix3d& R_nom,
                              const Eigen::Matrix3d& R_act) {
  Eigen::Matrix3d dR = R_nom * R_act.transpose();
  Eigen::AngleAxisd aa(dR);
  return aa.axis() * aa.angle();
}

void setCollisionHigh(franka::Robot& r) {
  // 抗干扰：collision 放开，推撞不触发 reflex（仅影子测试用）
  r.setCollisionBehavior({{20, 20, 18, 18, 15, 12, 10}},
                         {{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
                         {{20, 20, 18, 18, 15, 12, 10}},
                         {{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
                         {{15, 15, 15, 20, 20, 20}},
                         {{1000, 1000, 1000, 1000, 1000, 1000}},
                         {{15, 15, 15, 20, 20, 20}},
                         {{1000, 1000, 1000, 1000, 1000, 1000}});
}

void setCollisionNormal(franka::Robot& r) {
  r.setCollisionBehavior({{15, 15, 12, 12, 10, 8, 6}},
                         {{40, 40, 36, 36, 32, 28, 24}},
                         {{15, 15, 12, 12, 10, 8, 6}},
                         {{40, 40, 36, 36, 32, 28, 24}},
                         {{10, 10, 10, 15, 15, 15}},
                         {{40, 40, 40, 60, 60, 60}},
                         {{10, 10, 10, 15, 15, 15}},
                         {{40, 40, 40, 60, 60, 60}});
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cout << "用法: " << argv[0]
              << " <IP> <起点姿态> <终点姿态> [峰值速度rad/s]\n"
              << "例:   " << argv[0] << " 172.16.0.2 home pos2 0.12\n"
              << "      " << argv[0] << " 172.16.0.2 home pos2 0.12 high  (可推撞)\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string from_name = argv[2];
  const std::string to_name = argv[3];
  double speed = (argc > 4) ? std::atof(argv[4]) : 0.12;
  if (speed <= 0.0 || speed > 1.0) speed = 0.12;
  bool allow_push = (argc > 5) && (std::string(argv[5]) == "high");

  std::array<double, 7> q_from{}, q_to{};
  if (!loadQ(from_name, &q_from)) {
    std::cerr << "找不到姿态: " << posePath(from_name) << "\n";
    return 1;
  }
  if (!loadQ(to_name, &q_to)) {
    std::cerr << "找不到姿态: " << posePath(to_name) << "\n";
    return 1;
  }

  if(allow_push){std::cerr<<"Continuous trial requires normal collision profile\n";return 2;}
  const bool preview_only=argc>6&&std::string(argv[6])=="--read-only-cycle";
  std::signal(SIGINT,stop_signal);std::signal(SIGTERM,stop_signal);
  const char* diagnostic_env=std::getenv("FR3_ESN_DIAG_DIR");
  if(!diagnostic_env||!std::filesystem::is_directory(diagnostic_env)){std::cerr<<"Precreated diagnostic directory required\n";return 2;}
  Diagnostics diagnostic(diagnostic_env);
  try {
    franka::Robot robot(ip);
    franka::Model model = robot.loadModel();
    // Refuse implicit approach; start must already be near original home.
    const auto pre=robot.readOnce();
    if(pre.robot_mode!=franka::RobotMode::kIdle||pre.current_errors||pre.last_motion_errors)throw std::runtime_error("cycle requires Idle and no errors");
    for(int j=0;j<7;++j)if(std::abs(pre.q[j]-q_from[j])>.02||std::abs(pre.dq[j])>.01)throw std::runtime_error("cycle start not home/stationary");
    for(auto v:pre.joint_contact)if(v)throw std::runtime_error("joint contact at start");
    for(auto v:pre.cartesian_contact)if(v)throw std::runtime_error("Cartesian contact at start");
    for(auto v:pre.joint_collision)if(v)throw std::runtime_error("joint collision at start");
    for(auto v:pre.cartesian_collision)if(v)throw std::runtime_error("Cartesian collision at start");
    const esn_cycle::Reference preview(q_from,q_to,pre.q_d,speed);
    double max_limit_error=0,max_tcp_speed=0;
    auto last=pre.q_d;esn_cycle::Q velocity{},acceleration{};
    for(double tt=0;tt<preview.duration()+.251;tt+=.001){
      const auto raw=preview.at(tt);auto filtered=raw;
      const auto lo=robot.getLowerJointVelocityLimits(last),hi=robot.getUpperJointVelocityLimits(last);
      for(int j=0;j<7;++j){
        if(lo[j]>0||hi[j]<0)throw std::runtime_error("cycle excludes zero velocity");
        filtered[j]=franka::lowpassFilter(.001,raw[j],last[j],franka::kDefaultCutoffFrequency);
      }
      const auto next=franka::limitRate(hi,lo,franka::kMaxJointAcceleration,franka::kMaxJointJerk,filtered,last,velocity,acceleration);
      esn_cycle::Q nv{},na{};
      for(int j=0;j<7;++j){nv[j]=(next[j]-last[j])/.001;na[j]=(nv[j]-velocity[j])/.001;max_limit_error=std::max(max_limit_error,std::abs(next[j]-raw[j]));}
      last=next;velocity=nv;acceleration=na;
      if(static_cast<int>(tt*1000)%10==0){
        auto local=pre;local.q=raw;const auto jac=model.zeroJacobian(franka::Frame::kEndEffector,local);const auto v=preview.at(tt,1);
        double squared=0;for(int axis=0;axis<3;++axis){double speed_axis=0;for(int j=0;j<7;++j)speed_axis+=jac[6*j+axis]*v[j];squared+=speed_axis*speed_axis;}
        max_tcp_speed=std::max(max_tcp_speed,std::sqrt(squared));
      }
    }
    if(max_limit_error>.003)throw std::runtime_error("cycle limiter distortion");
    std::cout<<"Continuous reference preview passed: nominal seconds="<<preview.duration()<<", limiter difference rad="<<max_limit_error<<", nominal peak TCP m/s="<<max_tcp_speed<<std::endl;
    if(preview_only){std::cout<<"READ_ONLY_COMPLETE_NO_SETTERS_NO_CONTROL"<<std::endl;return 0;}
    // 低刚度阻抗: 才能"被推动"而不是硬顶
    {
      std::array<double, 7> K;
      K.fill(300.0);
      robot.setJointImpedance(K);
      std::cout << "关节刚度 = 300 Nm/rad（柔顺，可被推动）\n";
    }

    if (allow_push) {
      setCollisionHigh(robot);
      std::cout << "已连接 " << ip << "  （减速+让位接管 + 抗干扰阈值）\n";
    } else {
      setCollisionNormal(robot);
      std::cout << "已连接 " << ip << "  （减速接管模式）\n";
    }

    // No automatic approach in continuous trial; preflight must match home.

    // ---- 外力零偏校准：静止读 200 帧（约 200ms），取均值 ----
    Eigen::Matrix<double, 6, 1> f_bias = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix3d R_cal = Eigen::Matrix3d::Identity();
    {
      // 先静置 0.5s 让机械臂稳定, 再取 400 个样本的【中位数】(抗离群)
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      constexpr int kN = 400;
      std::array<std::array<double, 6>, kN> buf{};
      int n = 0;
      franka::RobotState st_cal{};
      robot.read([&](const franka::RobotState& rs) {
        for (size_t i = 0; i < 6; ++i) buf[n][i] = rs.O_F_ext_hat_K[i];
        st_cal = rs;
        return ++n < kN;
      });
      for (size_t i = 0; i < 6; ++i) {
        std::array<double, kN> col{};
        for (int k = 0; k < n; ++k) col[k] = buf[k][i];
        std::sort(col.begin(), col.begin() + n);
        f_bias[i] = col[n / 2];   // 中位数
      }
      R_cal = toPose(model.pose(franka::Frame::kEndEffector, st_cal)).R;
      std::printf("外力零偏校准(基线): [%.2f %.2f %.2f | %.2f %.2f %.2f] N/Nm\n",
                  f_bias[0], f_bias[1], f_bias[2], f_bias[3], f_bias[4],
                  f_bias[5]);
    }
    // 把基线偏置换算到末端坐标系 -> 之后按当前姿态旋转回来,
    // 近似补偿"未建模负载随姿态变化的重力投影"
    const Eigen::Vector3d c_force = R_cal.transpose() * f_bias.head<3>();
    const Eigen::Vector3d c_moment = R_cal.transpose() * f_bias.tail<3>();
    std::printf("末端系固定偏置: F=[%.2f %.2f %.2f] N  M=[%.2f %.2f %.2f] Nm\n",
                c_force[0], c_force[1], c_force[2], c_moment[0], c_moment[1],
                c_moment[2]);

    // ---- CSV 日志 ----
    FILE* csv = std::fopen("/tmp/esn_shadow_obs.csv", "w");
    if (csv) {
      std::fprintf(csv, "t");
      for (int i = 0; i < 48; ++i) std::fprintf(csv, ",obs%d", i);
      for (int i = 0; i < 7; ++i) std::fprintf(csv, ",a%d", i);
      for (int i = 0; i < 6; ++i) std::fprintf(csv, ",dy%d", i);
      for (int i = 0; i < 3; ++i) std::fprintf(csv, ",pa%d", i);
      for (int i = 0; i < 3; ++i) std::fprintf(csv, ",pr%d", i);
      std::fprintf(csv, ",a0_applied,y_applied");
      std::fprintf(csv, "\n");
      std::fflush(csv);
    }

    // ---- 主循环：走 nomin 轨迹 + 影子推理 ----
    fr3_esn128::Esn128 policy;
    fr3_esn128::ActionFilter filter;
    policy.reset();

    bool initialized = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_ref{};
    std::optional<esn_cycle::Reference> cycle;
    bool midpoint_reported=false;
    double T = 0.0, t = 0.0, hold = 0.0, next_tick = 0.0;
    double tau = 0.0;        // 虚拟时间：按 a0 缩放后推进
    double a0_now = 0.0;     // 策略最新输出
    double a0_filt = 0.0;    // 低通后的减速量（真正用于调速）
    Eigen::Matrix<double, 6, 1> y_filt = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq_prev = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq_target = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq_vel = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 6, 1> dy_cart = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Vector3d p_ref_log = Eigen::Vector3d::Zero();
    Eigen::Vector3d p_act_log = Eigen::Vector3d::Zero();
    double max_yield = 0.0;
    double max_dq_step = 0.0;
    bool finishing = false;      // 已到终点, 正在收回让位
    bool in_contact = false;     // 接触门控状态
    double contact_timer = 0.0;  // 退出接触的确认计时
    bool have_prev = false;
    Pose prev_nom{}, prev_act{};

    // 固定任务方向: 起点 -> 终点的 XY 单位向量 (与训练一致的做法)
    Eigen::Vector3d path_dir = Eigen::Vector3d::Zero();
    {
      franka::RobotState st_tmp = robot.readOnce();
      for (size_t i = 0; i < 7; ++i) st_tmp.q[i] = q_from[i];
      const Eigen::Vector3d p_from =
          toPose(model.pose(franka::Frame::kEndEffector, st_tmp)).p;
      for (size_t i = 0; i < 7; ++i) st_tmp.q[i] = q_to[i];
      const Eigen::Vector3d p_to =
          toPose(model.pose(franka::Frame::kEndEffector, st_tmp)).p;
      Eigen::Vector3d d = p_to - p_from;
      d.z() = 0.0;
      if (d.norm() > 1e-6) path_dir = d.normalized();
      std::printf("任务方向(XY): [%.3f %.3f %.3f]  (起点->终点)\n", path_dir[0],
                  path_dir[1], path_dir[2]);
    }

    double max_force = 0.0, max_a0 = 0.0;
    int ticks = 0;

    std::cout << "\n开始：减速 + 让位都会真的作用到机械臂。\n";
    std::printf(" %6s | %7s | %8s | %7s | %8s | %s\n", "t(s)", "a0", "speed%",
                "|F|N", "yield(mm)", "接触");
    std::printf("--------+---------+----------+---------+---------\n");

    robot.control(
        [&](const franka::RobotState& rs,
            franka::Duration period) -> franka::JointPositions {
          const auto callback_begin=std::chrono::steady_clock::now();
          if(!diagnostic.trace.capture(rs,period.toSec()))throw std::runtime_error("diagnostic state capacity");
          if (!initialized) {
            if(stop_requested||rs.current_errors||rs.last_motion_errors)throw std::runtime_error("cycle stopped before start");
            q_start = rs.q_d;
            q_ref = q_start;
            double md = 0.0;
            for (size_t i = 0; i < 7; ++i) {
              md = std::max(md, std::abs(q_to[i] - q_start[i]));
            }
            cycle.emplace(q_from,q_to,q_start,speed);
            T = cycle->duration();
            initialized = true;
            return franka::JointPositions(q_ref);
          }

          if(stop_requested)throw std::runtime_error("operator stop");
          const double dt = period.toSec();
          if(!std::isfinite(dt)||dt<=0||dt>.005)throw std::runtime_error("cycle callback timing");
          t += dt;

          // a0 低通：时间常数 0.2s，避免速度阶跃导致 acceleration_discontinuity
          const double alpha = 1.0 - std::exp(-dt / 0.2);
          a0_filt += (a0_now - a0_filt) * alpha;

          // 虚拟时间推进：a0=1 时速度降到 20%
          const double scale = 1.0 - 0.8 * a0_filt;
          tau += dt * scale;

          // One cosine period; same peak nominal speed, no midpoint dwell/reset.
          q_ref = cycle->at(tau);
          if(!midpoint_reported&&tau>=cycle->leg){
            midpoint_reported=true;
            std::cout<<"MIDPOINT_POS2_RETURNING_SAME_CONTROL_SESSION"<<std::endl;
          }

          // ---- 让位通道 (1kHz): 只做速率限制逼近目标 dq ----
          // 雅可比只在 25Hz 策略时刻计算 (见下方 next_tick 分支)，避免撑爆 1ms 周期
          static std::array<double, 7> q_cmd_last{};
          static bool q_last_init = false;
          std::array<double, 7> q_cmd = q_ref;
          // 调试: DQ_TEST=<rad> 时用恒定让位量, 绕过策略
          if (const char* dqt = std::getenv("DQ_TEST")) {
            dq_target.setZero();
            dq_target[6] = std::atof(dqt);
          }
          {
            // 二阶平滑: 速度斜坡 (加速度 ≤ 2 rad/s²) + 速度上限 0.08 rad/s
            static const double vel_lim =
                std::getenv("VEL_LIM") ? std::atof(std::getenv("VEL_LIM")) : 0.15;
            static const double acc_lim =
                std::getenv("ACC_LIM") ? std::atof(std::getenv("ACC_LIM")) : 1.0;
            for (size_t i = 0; i < 7; ++i) {
              const double err = dq_target[i] - dq_prev[i];
              double v_des = err / 0.05;                    // 期望速度
              v_des = std::max(-vel_lim, std::min(vel_lim, v_des));
              const double dv = v_des - dq_vel[i];
              const double dv_lim = acc_lim * dt;
              dq_vel[i] += std::max(-dv_lim, std::min(dv_lim, dv));
              dq_prev[i] += dq_vel[i] * dt;
              max_dq_step = std::max(max_dq_step, std::abs(dq_vel[i]));
              q_cmd[i] = q_ref[i] + dq_prev[i];
            }
            max_yield = std::max(max_yield, y_filt.head<3>().norm());
          }

          // 诊断: 检测下发改动速率异常
          if (q_last_init && ticks < 300 && dt > 0) {
            double mv = 0.0;
            for (size_t i = 0; i < 7; ++i) {
              mv = std::max(mv, std::abs(q_cmd[i] - q_cmd_last[i]) / dt);
            }
            if (mv > 1.0) {
              std::fprintf(stderr,
                           "[超速] t=%.4f dt=%.4f vel=%.2f rad/s  dq=[",
                           t, dt, mv);
              for (size_t i = 0; i < 7; ++i) {
                std::fprintf(stderr, "%.4f ", q_cmd[i] - q_cmd_last[i]);
              }
              std::fprintf(stderr, "]\n");
            }
          }
          if (std::getenv("DQ_DEBUG") && ticks < 20) {
            std::fprintf(stderr,
                         "cyc t=%.4f dt=%.5f dqprev7=%.5f qcmd7=%.5f qref7=%.5f\n",
                         t, dt, dq_prev[6], q_cmd[6], q_ref[6]);
          }
          q_cmd_last = q_cmd;
          q_last_init = true;

          if (t >= next_tick && !finishing) {
            next_tick += kPolicyDt;

            // 名义位姿用"实际下发的指令"(含让位)，避免让位 -> 位置误差 -> 更大让位 的正反馈
            franka::RobotState st_nom = rs;
            for (size_t i = 0; i < 7; ++i) st_nom.q[i] = q_cmd[i];
            const Pose nom = toPose(model.pose(franka::Frame::kEndEffector, st_nom));
            const Pose act = toPose(model.pose(franka::Frame::kEndEffector, rs));
            {
              franka::RobotState st_r = rs;
              for (size_t i = 0; i < 7; ++i) st_r.q[i] = q_ref[i];
              p_ref_log = toPose(model.pose(franka::Frame::kEndEffector, st_r)).p;
              p_act_log = act.p;
            }

            fr3_esn128::Obs obs;
            obs.setZero();
            for (size_t i = 0; i < 7; ++i) {
              obs[i] = rs.q[i];
              obs[7 + i] = rs.dq[i];
              obs[32 + i] = rs.tau_ext_hat_filtered[i];
            }
            // 姿态相关的偏置补偿: 把校准时的末端系偏置按当前姿态旋转
            const Eigen::Vector3d bias_f = act.R * c_force;
            const Eigen::Vector3d bias_m = act.R * c_moment;
            for (size_t i = 0; i < 3; ++i) {
              obs[39 + i] = rs.O_F_ext_hat_K[i] - bias_f[i];
              obs[42 + i] = rs.O_F_ext_hat_K[3 + i] - bias_m[i];
            }
            // 死区: 残余偏置(1.5 N / 0.15 Nm 以内)不算接触, 直接清零
            {
              const double f_norm =
                  std::sqrt(obs[39] * obs[39] + obs[40] * obs[40] +
                            obs[41] * obs[41]);
              const double m_norm =
                  std::sqrt(obs[42] * obs[42] + obs[43] * obs[43] +
                            obs[44] * obs[44]);
              static const double kFDead =
                  std::getenv("F_DEAD") ? std::atof(std::getenv("F_DEAD")) : 1.5;
              static const double kMDead =
                  std::getenv("M_DEAD") ? std::atof(std::getenv("M_DEAD")) : 0.15;
              if (f_norm < kFDead) {
                for (size_t i = 0; i < 3; ++i) obs[39 + i] = 0.0;
              }
              if (m_norm < kMDead) {
                for (size_t i = 0; i < 3; ++i) obs[42 + i] = 0.0;
              }
            }

            Eigen::Vector3d v_nom = Eigen::Vector3d::Zero(), w_nom = Eigen::Vector3d::Zero();
            Eigen::Vector3d v_act = Eigen::Vector3d::Zero(), w_act = Eigen::Vector3d::Zero();
            if (have_prev) {
              v_nom = (nom.p - prev_nom.p) / kPolicyDt;
              w_nom = angularVelocity(prev_nom.R, nom.R, kPolicyDt);
              v_act = (act.p - prev_act.p) / kPolicyDt;
              w_act = angularVelocity(prev_act.R, act.R, kPolicyDt);
            }
            prev_nom = nom;
            prev_act = act;
            have_prev = true;

            for (size_t i = 0; i < 3; ++i) {
              obs[14 + i] = v_nom[i];           // 名义线速度
              obs[17 + i] = w_nom[i];           // 名义角速度
              obs[20 + i] = nom.p[i] - act.p[i];// 位置误差
              obs[26 + i] = v_nom[i] - v_act[i];// 线速度误差
              obs[29 + i] = w_nom[i] - w_act[i];// 角速度误差
            }
            const Eigen::Vector3d rerr = rotationError(nom.R, act.R);
            for (size_t i = 0; i < 3; ++i) obs[23 + i] = rerr[i];

            const double direction=tau<cycle->leg?1.:-1.;
            for (size_t i = 0; i < 3; ++i) obs[45 + i] = direction*path_dir[i];

            const fr3_esn128::V7 action = policy.step(obs, kPolicyDt);
            filter.update(action);

            // ---- 接触门控（带滞回）----
            // 无接触时完全旁路策略 -> 指令等于纯轨迹, 与 safe_goto 一致
            const double f_norm = std::sqrt(obs[39] * obs[39] +
                                            obs[40] * obs[40] +
                                            obs[41] * obs[41]);
            static const double kEnter = 12.0;   // 进入接触阈值 N
            static const double kExit = 9.0;    // 退出接触阈值 N
            static const double kHold = 0.3;    // 退出确认时间 s
            if (!in_contact) {
              if (f_norm > kEnter) {
                in_contact = true;
                contact_timer = 0.0;
              }
            } else {
              if (f_norm < kExit) {
                contact_timer += kPolicyDt;
                if (contact_timer > kHold) in_contact = false;
              } else {
                contact_timer = 0.0;
              }
            }
            a0_now = in_contact ? action[0] : 0.0;
            // 对照实验: ESN_OFF=1 时强制策略输出为零(仅保留控制层)
            static const bool esn_off = std::getenv("ESN_OFF") != nullptr;
            if (esn_off) {
              a0_now = 0.0;
              dq_target.setZero();
              y_filt.setZero();
              in_contact = false;
            }

            // ---- 让位: 25Hz 计算一次目标关节偏移 ----
            if (!in_contact) {
              dq_target.setZero();   // 旁路: 不叠加任何让位
              for (size_t i = 0; i < 6; ++i) y_filt[i] = 0.0;
            } else {
              for (size_t i = 0; i < 6; ++i) {
                const double vmax = (i < 3) ? 0.12 : 0.5;
                const double step = filter.yield[i] - y_filt[i];
                const double l = vmax * kPolicyDt;
                y_filt[i] += std::max(-l, std::min(l, step));
              }
              Eigen::Matrix<double, 6, 1> y = y_filt;
              static const double kYmax =
                  std::getenv("Y_MAX") ? std::atof(std::getenv("Y_MAX")) : 0.18;
              static const double kRmax =
                  std::getenv("R_MAX") ? std::atof(std::getenv("R_MAX")) : 0.35;
              y.head<3>() = y.head<3>().cwiseMax(-kYmax).cwiseMin(kYmax);
              y.tail<3>() = y.tail<3>().cwiseMax(-kRmax).cwiseMin(kRmax);

              if (y.norm() > 1e-9) {
                franka::RobotState st_nom = rs;
                for (size_t i = 0; i < 7; ++i) st_nom.q[i] = q_ref[i];
                const std::array<double, 42> Jarr =
                    model.zeroJacobian(franka::Frame::kEndEffector, st_nom);
                Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(Jarr.data());
                const double lambda2 = 0.05 * 0.05;
                Eigen::Matrix<double, 6, 6> A =
                    J * J.transpose() +
                    lambda2 * Eigen::Matrix<double, 6, 6>::Identity();
                // 注意: ESN 的让位通道符号与我们的施加方向相反(实测 cos=-0.70),
                // 这里取反, 使末端沿外力方向让开
                Eigen::Matrix<double, 7, 1> dq =
                    -(J.transpose() * A.ldlt().solve(y));
                dy_cart = J * dq;   // 让位产生的笛卡尔位移(米/弧度)
                for (size_t i = 0; i < 7; ++i) {
                  dq_target[i] = std::max(-0.25, std::min(0.25, dq[i]));
                }
              } else {
                dq_target.setZero();
              }
            }

            const Eigen::Vector3d f_cal =
                Eigen::Vector3d(obs[39], obs[40], obs[41]);
            const double f_ext = f_cal.norm();
            if (csv) {
              std::fprintf(csv, "%.4f", t);
              for (int i = 0; i < 48; ++i) std::fprintf(csv, ",%.6f", obs[i]);
              for (int i = 0; i < 7; ++i) std::fprintf(csv, ",%.6f", action[i]);
              for (int i = 0; i < 6; ++i) std::fprintf(csv, ",%.6f", dy_cart[i]);
              for (int i = 0; i < 3; ++i) std::fprintf(csv, ",%.6f", p_act_log[i]);
              for (int i = 0; i < 3; ++i) std::fprintf(csv, ",%.6f", p_ref_log[i]);
              std::fprintf(csv, ",%.6f,%.6f", a0_filt,
                           y_filt.head<3>().norm());
              std::fprintf(csv, "\n");
            }
            max_force = std::max(max_force, f_ext);
            max_a0 = std::max(max_a0, action[0]);
            ++ticks;

            if ((ticks % 5) == 0) {  // 每 200ms 打印一行，避免刷屏
              std::printf(" %6.2f | %7.3f | %7.0f%% | %7.2f | %8.1f | %s\n", t,
                          a0_filt, (1.0 - 0.8 * a0_filt) * 100.0, f_ext,
                          y_filt.head<3>().norm() * 1000.0,
                          in_contact ? "YES" : "-");
            }
          }

          diagnostic.raw(t,tau,dt,q_cmd,q_ref,dq_prev,dq_vel,dq_target,
            std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-callback_begin).count());
          if (t > 90.0) {
            std::fprintf(stderr, "[安全] 运行超过 90s, 强制结束\n");
            return franka::MotionFinished(franka::JointPositions(q_cmd));
          }

          if (tau >= T) {
            if (!finishing) {
              finishing = true;
              dq_target.setZero();   // 让位目标归零 -> 平滑收回
            }
            // 等让位速度收敛后再结束
            double vmax = 0.0, dmax = 0.0;
            for (size_t i = 0; i < 7; ++i) {
              vmax = std::max(vmax, std::abs(dq_vel[i]));
              dmax = std::max(dmax, std::abs(dq_prev[i]));
            }
            if (vmax < 0.002 && dmax < 0.002) {
              hold += dt;
              if (hold >= kHoldSeconds) {
                return franka::MotionFinished(franka::JointPositions(q_cmd));
              }
            } else {
              hold = 0.0;
            }
          }
          return franka::JointPositions(q_cmd);
        },
        franka::ControllerMode::kJointImpedance, true, 100.0);

    if (csv) {
      std::fclose(csv);
      std::cout << "观测/动作日志: /tmp/esn_shadow_obs.csv\n";
    }
    std::printf("\n运行结束: %d 次策略调用, max|F| = %.2f N, max a0 = %.3f, "
                "max yield = %.1f mm, max dq/dt = %.2f rad/s\n",
                ticks, max_force, max_a0, max_yield * 1000.0, max_dq_step);
    try {
      franka::Robot r(ip);
      r.setJointImpedance({3000, 3000, 3000, 2500, 2500, 2000, 2000});
    } catch (...) {}
    std::cout << "注意: 减速 + 让位通道均已下发。\n";
  } catch (const franka::ControlException& e) {
    std::cerr<<"[ControlException] "<<e.what()<<'\n';
    try{fr3_trace::write_exception(diagnostic.out,e);}catch(const std::exception& writing){std::cerr<<"SDK tail write failed: "<<writing.what()<<'\n';}
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    return 1;
  }
  return 0;
}
