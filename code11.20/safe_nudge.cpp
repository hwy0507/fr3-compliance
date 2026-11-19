// safe_nudge.cpp — 最小风险的机械臂动作测试
//
// 只动一个关节、幅度很小、速度很慢，结束后停在目标位置。
// 用途：确认 FCI 控制通路 + 运动方向 + 幅度都正常，再考虑更复杂的程序。
//
// 用法:
//   ./safe_nudge <机器人IP> [关节1-7] [幅度角度] [速度rad/s]
// 例:
//   ./safe_nudge 172.16.0.2 7 3 0.05
//     → 第 7 关节（腕部），转 3 度，速度 0.05 rad/s
//
// 轨迹说明:
//   采用余弦(半波)插值 q(t) = q0 + Δ·(1-cos(πt/T))/2
//   起止速度天然为 0，加速度连续，不会触发
//   joint_motion_generator_velocity/acceleration_discontinuity reflex。
//   T 由期望速度反推: T = π·|Δ| / (2·v_peak)
//
// 安全设计:
//   - 幅度硬上限 10°，速度硬上限 0.2 rad/s（命令行超了自动夹住）
//   - 超出关节限位(±2.8 rad)直接拒绝，不动
//   - 起点取控制循环第一帧真实状态，避免首帧跳变
//   - 打印计划后等待回车才动
//   - 到位后保持 1 秒再优雅退出
//   - 任何异常打印后退出，不重试

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {
constexpr double kMaxDeltaRad = 10.0 * M_PI / 180.0;  // 10°
constexpr double kMaxSpeed = 0.2;                     // rad/s
constexpr double kJointLimit = 2.8;                   // rad, 留余量
constexpr double kHoldSeconds = 1.0;
constexpr double kMaxRuntimeSeconds = 20.0;

void printUsage(const char* prog) {
  std::cout << "用法: " << prog << " <IP> [关节1-7] [幅度deg] [速度rad/s]\n"
            << "例:   " << prog << " 172.16.0.2 7 3 0.05\n";
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string ip = argv[1];
  int joint = (argc > 2) ? std::atoi(argv[2]) : 7;            // 1..7
  double delta_deg = (argc > 3) ? std::atof(argv[3]) : 3.0;   // 度
  double speed = (argc > 4) ? std::atof(argv[4]) : 0.05;      // rad/s (峰值)

  if (joint < 1 || joint > 7) {
    std::cerr << "关节编号必须是 1-7\n";
    return 1;
  }
  const size_t idx = static_cast<size_t>(joint - 1);

  double delta_rad = delta_deg * M_PI / 180.0;
  if (std::abs(delta_rad) > kMaxDeltaRad) {
    std::cerr << "[安全] 幅度 " << delta_deg << "° 超过上限 10°，已夹到 10°\n";
    delta_rad = (delta_rad > 0 ? kMaxDeltaRad : -kMaxDeltaRad);
  }
  if (speed <= 0.0 || speed > kMaxSpeed) {
    std::cerr << "[安全] 速度 " << speed << " 不合法，使用 " << kMaxSpeed << " rad/s\n";
    speed = kMaxSpeed;
  }

  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "\n";

    // 温和的碰撞阈值，避免轻碰就触发 reflex
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},   // torque lower (accel)
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},   // torque upper (accel)
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},   // torque lower (nominal)
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},   // torque upper (nominal)
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},         // force lower (accel)
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},         // force upper (accel)
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},         // force lower (nominal)
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});        // force upper (nominal)

    franka::RobotState state = robot.readOnce();
    const std::array<double, 7> q_now = state.q;
    const double q_expect = q_now[idx] + delta_rad;
    if (std::abs(q_expect) > kJointLimit) {
      std::cerr << "[安全] 预计目标 " << q_expect << " rad 超出 ±2.8 rad，拒绝执行\n";
      return 1;
    }
    const double T = M_PI * std::abs(delta_rad) / (2.0 * speed);

    std::cout << "\n计划:\n"
              << "  当前 q" << joint << " = " << q_now[idx] << " rad ("
              << q_now[idx] * 180.0 / M_PI << "°)\n"
              << "  预计目标 q" << joint << " = " << q_expect << " rad ("
              << q_expect * 180.0 / M_PI << "°)\n"
              << "  幅度 " << delta_deg << "°, 峰值速度 " << speed
              << " rad/s, 用时约 " << T << " s\n"
              << "\n确认周围空旷、急停在手，按回车开始 (Ctrl+C 取消)..." << std::endl;
    std::cin.get();

    // ---- 余弦轨迹，起点/终点速度与加速度均连续 ----
    bool initialized = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_target{};
    std::array<double, 7> q_cmd{};
    double t = 0.0;
    double hold = 0.0;
    double traj_T = T;
    bool finished = false;

    robot.control([&](const franka::RobotState& rs,
                      franka::Duration period) -> franka::JointPositions {
      // 第一帧: 以机器人当前真实状态为起点, 原样返回 (零跳变)
      if (!initialized) {
        q_start = rs.q;
        q_target = q_start;
        q_target[idx] += delta_rad;
        q_cmd = q_start;
        initialized = true;
        return franka::JointPositions(q_cmd);
      }

      const double dt = period.toSec();
      t += dt;

      const double s = (t >= traj_T) ? 1.0
                      : (1.0 - std::cos(M_PI * t / traj_T)) / 2.0;
      for (size_t i = 0; i < 7; ++i) {
        q_cmd[i] = q_start[i] + (q_target[i] - q_start[i]) * s;
      }

      if (t >= traj_T) {
        hold += dt;
        if (hold >= kHoldSeconds) {
          std::cout << "\n到位, 保持 " << kHoldSeconds << "s 后退出\n";
          finished = true;
          return franka::MotionFinished(franka::JointPositions(q_cmd));
        }
      }

      if (t > kMaxRuntimeSeconds) {
        std::cerr << "\n[安全] 超过 " << kMaxRuntimeSeconds << "s, 强制结束\n";
        finished = true;
        return franka::MotionFinished(franka::JointPositions(q_cmd));
      }

      return franka::JointPositions(q_cmd);
    });

    if (finished) {
      std::cout << "完成。机械臂停在目标位置。\n";
    }
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    std::cerr << "提示: 若是 reflex, 到 Desk 里 Reset; 若是 brakes locked, 先解锁关节。\n";
    return 1;
  }
  return 0;
}
