// esn_velocity.cpp — 纯 P 速度伺服 + ESN（yield 解释为“速度残差”）
//
// 与 esn_yield 的区别：
//   1) 下发的不是位置 JointPositions，而是速度 JointVelocities
//   2) 底层跟踪不是 FR3 关节阻抗，而是 P 速度环：
//        dq_cmd = dq_ff + dq_res + KP * (dq_ff - dq_meas)
//      ——没有位置反馈，被推偏后不会自己回到轨迹
//   3) ESN 的 a1..a6 不再当作“位移偏移”，而是当作“速度残差”：
//        v_res = KV_LIN * a[1..3]   (m/s)
//        w_res = KV_ROT * a[4..6]   (rad/s)
//        dq_res = J^+ * [v_res; w_res]
//   4) a0 仍用于放慢名义轨迹（虚拟时间 tau）
//
// 环境变量：
//   KP=10        P 增益（速度误差 -> 速度修正）
//   VMAX=0.4     速度指令上限 rad/s
//   VACC=0.5     速度指令变化率上限 rad/s^2（必须小，否则会触发 discontinuity）
//   KV_LIN=0.32  ESN 平移通道 -> m/s（与原 ActionFilter 一致）
//   KV_ROT=0.60  ESN 转动通道 -> rad/s（与原 ActionFilter 一致）
//   GATE_ENTER=12 / GATE_EXIT=9   接触门控（N）
//   F_DEAD=1.5 / M_DEAD=0.15      力/力矩死区
//   ESN_OFF=1    关闭 ESN（纯 P 速度伺服对照）
//
// 用法：
//   ./esn_velocity <IP> <起点姿态> <终点姿态> [峰值速度rad/s] [high]

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "../FR3_ESN128_main/fr3_esn128/esn_actor.hpp"

namespace {

constexpr double kPolicyDt = 0.04;    // 25 Hz
constexpr double kHoldSeconds = 1.0;
constexpr double kMaxRuntime = 90.0;

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
  for (char& c : body) if (c == ',') c = ' ';
  std::istringstream iss(body);
  for (size_t i = 0; i < 7; ++i) if (!(iss >> (*q)[i])) return false;
  return true;
}

bool loadQ(const std::string& name, std::array<double, 7>* q) {
  std::ifstream in(posePath(name));
  if (!in) return false;
  std::stringstream buf; buf << in.rdbuf();
  return parseQ(buf.str(), q);
}

struct Pose { Eigen::Vector3d p; Eigen::Matrix3d R; };

Pose toPose(const std::array<double, 16>& T) {
  Pose o;
  o.p << T[12], T[13], T[14];
  for (int c = 0; c < 3; ++c) for (int r = 0; r < 3; ++r) o.R(r, c) = T[c * 4 + r];
  return o;
}

Eigen::Vector3d angVel(const Eigen::Matrix3d& Rp, const Eigen::Matrix3d& Rn, double dt) {
  Eigen::AngleAxisd aa(Eigen::Matrix3d(Rn * Rp.transpose()));
  return aa.axis() * aa.angle() / dt;
}

Eigen::Vector3d rotErr(const Eigen::Matrix3d& Rn, const Eigen::Matrix3d& Ra) {
  Eigen::AngleAxisd aa(Eigen::Matrix3d(Rn * Ra.transpose()));
  return aa.axis() * aa.angle();
}

void setCollisionHigh(franka::Robot& r) {
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
              << " <IP> <起点姿态> <终点姿态> [峰值速度rad/s] [high]\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string from_name = argv[2];
  const std::string to_name = argv[3];
  double speed = (argc > 4) ? std::atof(argv[4]) : 0.15;
  if (speed <= 0.0 || speed > 1.0) speed = 0.15;
  const bool allow_push = (argc > 5) && (std::string(argv[5]) == "high");

  std::array<double, 7> q_from{}, q_to{};
  if (!loadQ(from_name, &q_from)) { std::cerr << "找不到姿态: " << from_name << "\n"; return 1; }
  if (!loadQ(to_name, &q_to))     { std::cerr << "找不到姿态: " << to_name << "\n"; return 1; }

  // ---- 参数 ----
  const double KP    = std::getenv("KP")    ? std::atof(std::getenv("KP"))    : 10.0;
  const double VMAX  = std::getenv("VMAX")  ? std::atof(std::getenv("VMAX"))  : 0.4;
  const double VACC  = std::getenv("VACC")  ? std::atof(std::getenv("VACC"))  : 0.5;
  const double KVLIN = std::getenv("KV_LIN")? std::atof(std::getenv("KV_LIN")): 0.32;
  const double KVROT = std::getenv("KV_ROT")? std::atof(std::getenv("KV_ROT")): 0.60;
  const double GATE_IN  = std::getenv("GATE_ENTER") ? std::atof(std::getenv("GATE_ENTER")) : 12.0;
  const double GATE_OUT = std::getenv("GATE_EXIT")  ? std::atof(std::getenv("GATE_EXIT"))  : 9.0;
  const double F_DEAD = std::getenv("F_DEAD") ? std::atof(std::getenv("F_DEAD")) : 1.5;
  const double M_DEAD = std::getenv("M_DEAD") ? std::atof(std::getenv("M_DEAD")) : 0.15;
  const bool ESN_OFF = [&] {
    const char* e = std::getenv("ESN_OFF");
    return e != nullptr && *e != '\0' && std::strcmp(e, "0") != 0;
  }();

  try {
    franka::Robot robot(ip);
    franka::Model model = robot.loadModel();
    if (allow_push) setCollisionHigh(robot); else setCollisionNormal(robot);

    std::printf("模式: 纯 P 速度伺服   KP=%.1f VMAX=%.2f VACC=%.2f  "
                "KV_LIN=%.3f KV_ROT=%.2f  门控 %.1f/%.1f N  ESN=%s\n",
                KP, VMAX, VACC, KVLIN, KVROT, GATE_IN, GATE_OUT,
                ESN_OFF ? "关" : "开");

    // ---- 先（位置控制）走到起点 ----
    {
      franka::RobotState st = robot.readOnce();
      double md = 0.0;
      for (size_t i = 0; i < 7; ++i) md = std::max(md, std::abs(q_from[i] - st.q[i]));
      if (md > 0.02) {
        std::cout << "先移动到起点 " << from_name << "（最大差 " << md << " rad）...\n";
        bool init = false;
        std::array<double, 7> qs{}, qc{};
        double T = M_PI * md / (2.0 * 0.15), t = 0.0, hold = 0.0;
        robot.control([&](const franka::RobotState& rs, franka::Duration p) -> franka::JointPositions {
          if (!init) { qs = rs.q; qc = qs; init = true; return franka::JointPositions(qc); }
          const double dt = p.toSec(); t += dt;
          const double s = (t >= T) ? 1.0 : (1.0 - std::cos(M_PI * t / T)) / 2.0;
          for (size_t i = 0; i < 7; ++i) qc[i] = qs[i] + (q_from[i] - qs[i]) * s;
          if (t >= T && (hold += dt) >= kHoldSeconds)
            return franka::MotionFinished(franka::JointPositions(qc));
          return franka::JointPositions(qc);
        });
      }
    }

    // ---- 外力零偏校准（静置 0.5 s，400 帧中位数）----
    Eigen::Matrix<double, 6, 1> f_bias = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix3d R_cal = Eigen::Matrix3d::Identity();
    {
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
        f_bias[i] = col[n / 2];
      }
      R_cal = toPose(model.pose(franka::Frame::kEndEffector, st_cal)).R;
    }
    const Eigen::Vector3d c_force = R_cal.transpose() * f_bias.head<3>();
    const Eigen::Vector3d c_moment = R_cal.transpose() * f_bias.tail<3>();
    std::printf("零偏校准: |F0|=%.2f N |M0|=%.2f Nm\n", f_bias.head<3>().norm(),
                f_bias.tail<3>().norm());

    // ---- 任务方向 ----
    Eigen::Vector3d path_dir = Eigen::Vector3d::Zero();
    {
      franka::RobotState st = robot.readOnce();
      for (size_t i = 0; i < 7; ++i) st.q[i] = q_from[i];
      const Eigen::Vector3d p0 = toPose(model.pose(franka::Frame::kEndEffector, st)).p;
      for (size_t i = 0; i < 7; ++i) st.q[i] = q_to[i];
      const Eigen::Vector3d p1 = toPose(model.pose(franka::Frame::kEndEffector, st)).p;
      Eigen::Vector3d d = p1 - p0; d.z() = 0.0;
      if (d.norm() > 1e-6) path_dir = d.normalized();
    }

    FILE* csv = std::fopen("/tmp/esn_velocity.csv", "w");
    if (csv) std::fprintf(csv, "t,a0,vres_x,vres_y,vres_z,F\n");
    std::cout << "按回车开始 (Ctrl+C 取消)...\n";
    std::cin.get();

    fr3_esn128::Esn128 policy;
    fr3_esn128::ActionFilter filter;
    policy.reset();

    bool init = false;
    std::array<double, 7> q_start{};
    double T = 0.0, tau = 0.0, t = 0.0, next_tick = 0.0;
    double a0_now = 0.0, a0_filt = 0.0;
    Eigen::Matrix<double, 6, 1> v_res = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq_res_target = Eigen::Matrix<double, 7, 1>::Zero();
    std::array<double, 7> dq_cmd_prev{};
    bool contact = false;
    double contact_timer = 0.0;
    bool have_prev = false;
    Pose prev_nom{}, prev_act{};
    double maxF = 0.0, maxVres = 0.0;
    double hold = 0.0;
    bool finishing = false;
    bool qref_prev_init = false;
    std::array<double, 7> qref_prev{};

    robot.control(
        [&](const franka::RobotState& rs, franka::Duration period)
            -> franka::JointVelocities {
          if (!init) {
            q_start = rs.q;
            double md = 0.0;
            for (size_t i = 0; i < 7; ++i) md = std::max(md, std::abs(q_to[i] - q_start[i]));
            T = (md > 0) ? M_PI * md / (2.0 * speed) : 0.0;
            qref_prev = q_start;
            qref_prev_init = true;
            init = true;
            return franka::JointVelocities(std::array<double, 7>{});
          }

          const double dt = period.toSec();
          t += dt;

          // 虚拟时间与名义关节轨迹
          a0_filt += (a0_now - a0_filt) * (1.0 - std::exp(-dt / 0.2));
          const double scale = 1.0 - 0.8 * a0_filt;
          tau += dt * scale;
          const double s = (T <= 0.0 || tau >= T) ? 1.0
                          : (1.0 - std::cos(M_PI * tau / T)) / 2.0;
          std::array<double, 7> q_ref{};
          for (size_t i = 0; i < 7; ++i) q_ref[i] = q_start[i] + (q_to[i] - q_start[i]) * s;

          // 前馈速度：q_ref 的解析导数（含 a0 缩放）
          const double ds = (T <= 0.0 || tau >= T)
                                ? 0.0
                                : (M_PI / (2.0 * T)) * std::sin(M_PI * tau / T) * scale;
          std::array<double, 7> dq_ff{};
          for (size_t i = 0; i < 7; ++i) dq_ff[i] = (q_to[i] - q_start[i]) * ds;

          // ---- 25 Hz 策略 ----
          if (t >= next_tick && !finishing) {
            next_tick += kPolicyDt;

            franka::RobotState st_nom = rs;
            for (size_t i = 0; i < 7; ++i) st_nom.q[i] = q_ref[i];
            const Pose nom = toPose(model.pose(franka::Frame::kEndEffector, st_nom));
            const Pose act = toPose(model.pose(franka::Frame::kEndEffector, rs));

            fr3_esn128::Obs obs; obs.setZero();
            for (size_t i = 0; i < 7; ++i) {
              obs[i] = rs.q[i];
              obs[7 + i] = rs.dq[i];
              obs[32 + i] = rs.tau_ext_hat_filtered[i];
            }
            const Eigen::Vector3d bf = act.R * c_force;
            const Eigen::Vector3d bm = act.R * c_moment;
            for (size_t i = 0; i < 3; ++i) {
              obs[39 + i] = rs.O_F_ext_hat_K[i] - bf[i];
              obs[42 + i] = rs.O_F_ext_hat_K[3 + i] - bm[i];
            }
            {  // 死区
              const double fn = std::sqrt(obs[39]*obs[39] + obs[40]*obs[40] + obs[41]*obs[41]);
              const double mn = std::sqrt(obs[42]*obs[42] + obs[43]*obs[43] + obs[44]*obs[44]);
              if (fn < F_DEAD) for (size_t i = 0; i < 3; ++i) obs[39 + i] = 0.0;
              if (mn < M_DEAD) for (size_t i = 0; i < 3; ++i) obs[42 + i] = 0.0;
            }

            Eigen::Vector3d v_nom = Eigen::Vector3d::Zero(), w_nom = Eigen::Vector3d::Zero();
            Eigen::Vector3d v_act = Eigen::Vector3d::Zero(), w_act = Eigen::Vector3d::Zero();
            if (have_prev) {
              v_nom = (nom.p - prev_nom.p) / kPolicyDt;
              w_nom = angVel(prev_nom.R, nom.R, kPolicyDt);
              v_act = (act.p - prev_act.p) / kPolicyDt;
              w_act = angVel(prev_act.R, act.R, kPolicyDt);
            }
            prev_nom = nom; prev_act = act; have_prev = true;

            for (size_t i = 0; i < 3; ++i) {
              obs[14 + i] = v_nom[i];
              obs[17 + i] = w_nom[i];
              obs[20 + i] = nom.p[i] - act.p[i];
              obs[26 + i] = v_nom[i] - v_act[i];
              obs[29 + i] = w_nom[i] - w_act[i];
            }
            const Eigen::Vector3d re = rotErr(nom.R, act.R);
            for (size_t i = 0; i < 3; ++i) obs[23 + i] = re[i];
            for (size_t i = 0; i < 3; ++i) obs[45 + i] = path_dir[i];

            const fr3_esn128::V7 action = policy.step(obs, kPolicyDt);
            filter.update(action);

            // 接触门控（滞回）
            const double fn = std::sqrt(obs[39]*obs[39] + obs[40]*obs[40] + obs[41]*obs[41]);
            maxF = std::max(maxF, fn);
            if (!contact) {
              if (fn > GATE_IN) { contact = true; contact_timer = 0.0; }
            } else {
              if (fn < GATE_OUT) {
                contact_timer += kPolicyDt;
                if (contact_timer > 0.3) contact = false;
              } else {
                contact_timer = 0.0;
              }
            }
            a0_now = contact ? action[0] : 0.0;

            if (!ESN_OFF && contact) {
              // ESN yield -> 速度残差（不是位移）
              v_res << KVLIN * action[1], KVLIN * action[2], KVLIN * action[3],
                       KVROT * action[4], KVROT * action[5], KVROT * action[6];
              franka::RobotState st_n = rs;
              for (size_t i = 0; i < 7; ++i) st_n.q[i] = q_ref[i];
              const std::array<double, 42> Jarr =
                  model.zeroJacobian(franka::Frame::kEndEffector, st_n);
              Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(Jarr.data());
              Eigen::Matrix<double, 6, 6> A =
                  J * J.transpose() + 0.05 * 0.05 * Eigen::Matrix<double, 6, 6>::Identity();
              dq_res_target = J.transpose() * A.ldlt().solve(v_res);
            } else {
              v_res.setZero();
              dq_res_target.setZero();
            }
            maxVres = std::max(maxVres, v_res.head<3>().norm());

            if (csv) {
              std::fprintf(csv, "%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\n", t, a0_filt,
                           v_res[0], v_res[1], v_res[2], fn);
            }
          }

          // ---- 1 kHz：纯 P 速度伺服 ----
          std::array<double, 7> dq_cmd{};
          for (size_t i = 0; i < 7; ++i) {
            const double ff = dq_ff[i] + dq_res_target[i];
            double dq = ff + KP * (dq_ff[i] - rs.dq[i]);
            dq = std::max(-VMAX, std::min(VMAX, dq));
            const double lim = VACC * dt;
            double step = dq - dq_cmd_prev[i];
            step = std::max(-lim, std::min(lim, step));
            dq_cmd[i] = dq_cmd_prev[i] + step;
          }
          dq_cmd_prev = dq_cmd;
          qref_prev = q_ref;

          if (tau >= T) {
            if (!finishing) { finishing = true; dq_res_target.setZero(); v_res.setZero(); }
            bool stopped = true;
            for (size_t i = 0; i < 7; ++i) if (std::abs(dq_cmd[i]) > 0.002) stopped = false;
            hold = stopped ? hold + dt : 0.0;
            if (hold >= kHoldSeconds) {
              if (csv) std::fclose(csv);
              std::printf("\n运行结束: max|F|=%.2f N, max|v_res|=%.3f m/s\n", maxF, maxVres);
              return franka::MotionFinished(franka::JointVelocities(std::array<double, 7>{}));
            }
          }
          if (t > kMaxRuntime) {
            std::cerr << "\n[安全] 超时\n";
            if (csv) std::fclose(csv);
            return franka::MotionFinished(franka::JointVelocities(std::array<double, 7>{}));
          }
          return franka::JointVelocities(dq_cmd);
        },
        franka::ControllerMode::kJointImpedance);

    std::cout << "注意: 速度伺服模式已结束，机械臂停止运动。\n";
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    return 1;
  }
  return 0;
}
