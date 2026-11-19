// safe_compliant.cpp — 碰撞阈值预设 + 实时监测
//
// 用法:
//   ./safe_compliant <IP> [high|normal|off] [持续秒数]
//
// 模式:
//   high   contact 20 Nm/15 N，collision 1000 Nm/1000 N
//          → 一般推撞不会急停，但仍会显示"接触"标志（推荐做抗扰测试）
//   normal contact 15 Nm/10 N，collision 40 Nm/40 N
//          → 常规抓取/协作配置（恢复用）
//   off    contact 1000，collision 1000
//          → 两级都关掉，连 contact 标志也不会置位（最"钝"）
//
// 行为: 设好阈值后，机械臂原地保持当前位置 N 秒（默认 20s），
//       期间每 200ms 打印外部力矩/力峰值与 contact 标志。
//       程序结束前会自动把阈值恢复成 normal。
//
// 重要: libfranka 的阈值是持久化的（只有机器人重启才重置），
//       所以退出前必须恢复；本程序在正常结束/异常时都会尝试恢复。

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <cstdio>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

struct Preset {
  std::array<double, 7> torque_low;   // contact, acceleration
  std::array<double, 7> torque_high;  // collision, acceleration
  std::array<double, 6> force_low;    // contact
  std::array<double, 6> force_high;   // collision
};

Preset presetNormal() {
  return {{{15, 15, 12, 12, 10, 8, 6}},
          {{40, 40, 36, 36, 32, 28, 24}},
          {{10, 10, 10, 15, 15, 15}},
          {{40, 40, 40, 60, 60, 60}}};
}

Preset presetHigh() {
  return {{{20, 20, 18, 18, 15, 12, 10}},
          {{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
          {{15, 15, 15, 20, 20, 20}},
          {{1000, 1000, 1000, 1000, 1000, 1000}}};
}

Preset presetOff() {
  return {{{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
          {{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
          {{1000, 1000, 1000, 1000, 1000, 1000}},
          {{1000, 1000, 1000, 1000, 1000, 1000}}};
}

void apply(franka::Robot& robot, const Preset& p) {
  robot.setCollisionBehavior(p.torque_low, p.torque_high, p.torque_low,
                             p.torque_high, p.force_low, p.force_high,
                             p.force_low, p.force_high);
}

double norm3(const std::array<double, 6>& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
double norm4(const std::array<double, 6>& v) {
  return std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5]);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "用法: " << argv[0] << " <IP> [high|normal|off] [秒数]\n"
              << "  high   一般推撞不急停(默认), 显示 contact 标志\n"
              << "  normal 常规抓取配置(恢复用)\n"
              << "  off    连 contact 也不报\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string mode = (argc > 2) ? argv[2] : "high";
  const double seconds = (argc > 3) ? std::atof(argv[3]) : 20.0;
  double stiffness = (argc > 4) ? std::atof(argv[4]) : 300.0;
  if (stiffness < 50.0 || stiffness > 3000.0) stiffness = 300.0;

  Preset p;
  if (mode == "normal") {
    p = presetNormal();
  } else if (mode == "off") {
    p = presetOff();
  } else {
    p = presetHigh();
  }

  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "  模式: " << mode << "\n";
    apply(robot, p);
    std::printf("阈值已设置: contact=%.0f Nm / %.0f N, collision=%.0f Nm / %.0f N\n",
                p.torque_low[4], p.force_low[0], p.torque_high[4], p.force_high[0]);

    // 低刚度: 推得动 -> 关节力矩小 -> 不会撞 tau_J 上限
    std::array<double, 7> K;
    K.fill(stiffness);
    robot.setJointImpedance(K);
    std::printf("关节刚度 = %.0f Nm/rad (越低越软, 避免力矩超限)\n", stiffness);

    franka::RobotState st = robot.readOnce();
    const std::array<double, 7> q_hold = st.q;

    FILE* csv = std::fopen("/tmp/compliant_log.csv", "w");
    if (csv) {
      std::fprintf(csv,
                   "t_s,max_tau_Nm,F_N,Fx,Fy,Fz,contact,collide,"
                   "tau1,tau2,tau3,tau4,tau5,tau6,tau7\n");
      std::fflush(csv);
    }

    std::cout << "\n原地保持 " << seconds
              << " 秒。现在可以用手推/撞机械臂观察反应。\n"
              << "（高压模式下不应急停；每 200ms 打印一次状态）\n\n";
    // 5 秒倒计时，方便操作者就位
    for (int i = 5; i >= 1; --i) {
      std::printf("\r  准备：%d ...", i);
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::printf("\r  >>> 开始推！ <<<                        \n\n");

    std::printf(" %8s | %10s | %10s | %8s | %8s\n", "t(s)", "max|tau|Nm",
                "|F|N", "contact", "collide");
    std::printf("----------+------------+------------+----------+---------\n");

    double t = 0.0;
    double next_print = 0.0;
    double max_tau_seen = 0.0, max_f_seen = 0.0;
    bool contact_seen = false, collide_seen = false;

    robot.control([&](const franka::RobotState& rs,
                      franka::Duration period) -> franka::JointPositions {
      const double dt = period.toSec();
      t += dt;

      double mt = 0.0;
      for (size_t i = 0; i < 7; ++i) {
        mt = std::max(mt, std::abs(rs.tau_ext_hat_filtered[i]));
      }
      const double mf = norm3(rs.O_F_ext_hat_K);
      max_tau_seen = std::max(max_tau_seen, mt);
      max_f_seen = std::max(max_f_seen, mf);

      bool contact = false, collide = false;
      for (size_t i = 0; i < 7; ++i) {
        if (rs.joint_contact[i] > 0) contact = true;
        if (rs.joint_collision[i] > 0) collide = true;
      }
      for (size_t i = 0; i < 6; ++i) {
        if (rs.cartesian_contact[i] > 0) contact = true;
        if (rs.cartesian_collision[i] > 0) collide = true;
      }
      contact_seen |= contact;
      collide_seen |= collide;

      if (csv) {
        std::fprintf(csv, "%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d",
                     t, mt, mf, rs.O_F_ext_hat_K[0], rs.O_F_ext_hat_K[1],
                     rs.O_F_ext_hat_K[2], contact ? 1 : 0, collide ? 1 : 0);
        for (size_t i = 0; i < 7; ++i) {
          std::fprintf(csv, ",%.3f", rs.tau_ext_hat_filtered[i]);
        }
        std::fprintf(csv, "\n");
      }
      if (t >= next_print) {
        std::printf(" %8.2f | %10.2f | %10.2f | %8s | %8s\n", t, mt, mf,
                    contact ? "YES" : "-", collide ? "YES" : "-");
        next_print += 0.2;
      }

      // 原地保持：始终下发同一个目标关节角
      if (t >= seconds) {
        return franka::MotionFinished(franka::JointPositions(q_hold));
      }
      return franka::JointPositions(q_hold);
    },
        franka::ControllerMode::kJointImpedance);

    std::printf("\n峰值统计: max|tau_ext| = %.2f Nm, max|F_ext| = %.2f N\n",
                max_tau_seen, max_f_seen);
    std::printf("是否出现过 contact 标志: %s   是否出现过 collision: %s\n",
                contact_seen ? "是" : "否", collide_seen ? "是" : "否");
    if (csv) {
      std::fclose(csv);
      std::cout << "数据已保存: /tmp/compliant_log.csv\n";
    }
    std::cout << "恢复默认刚度与常规阈值...\n";
    robot.setJointImpedance({3000, 3000, 3000, 2500, 2500, 2000, 2000});
    apply(robot, presetNormal());
    std::cout << "完成。\n";
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    std::cerr << "数据已在 /tmp/compliant_log.csv（若有）。\n";
    std::cerr << "如果仍然触发了急停，说明碰到了安全层硬限制（Desk 安全配置），"
                 "或需要 automaticErrorRecovery。\n";
    return 1;
  }
  return 0;
}
