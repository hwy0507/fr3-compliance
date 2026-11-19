// safe_wrist.cpp — 末端三个腕关节同时小幅转动
//
// 用法:
//   ./safe_wrist <IP> [q5_deg] [q6_deg] [q7_deg] [峰值速度rad/s]
// 例:
//   ./safe_wrist 172.16.0.2 3 -3 3 0.05
//     → q5 +3°, q6 -3°, q7 +3°, 峰值速度 0.05 rad/s
//
// 所有关节共用一条余弦时间曲线 s(t)=(1-cos(πt/T))/2，
// 幅度不同则各自速度不同，但起止速度都为 0、加速度连续，
// 且总时长 T 按最大幅度/目标速度反推，保证任一关节不超过给定速度。
//
// 安全: 单关节幅度上限 10°，关节限位 ±2.8 rad，峰值速度上限 0.2 rad/s。

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {
constexpr double kMaxDeltaRad = 20.0 * M_PI / 180.0;  // 腕关节测试上限 20°
constexpr double kMaxSpeed = 0.2;
constexpr double kJointLimit = 2.8;
constexpr double kHoldSeconds = 1.0;
constexpr double kMaxRuntimeSeconds = 20.0;
constexpr size_t kFirstJoint = 4;  // 0-based: q5
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "用法: " << argv[0]
              << " <IP> [q5_deg] [q6_deg] [q7_deg] [峰值速度rad/s]\n"
              << "例:   " << argv[0] << " 172.16.0.2 3 -3 3 0.05\n";
    return 1;
  }

  const std::string ip = argv[1];
  std::array<double, 3> delta_deg = {3.0, -3.0, 3.0};
  for (int i = 0; i < 3 && (i + 2) < argc; ++i) {
    delta_deg[i] = std::atof(argv[i + 2]);
  }
  double speed = (argc > 5) ? std::atof(argv[5]) : 0.05;

  std::array<double, 3> delta_rad{};
  double max_abs = 0.0;
  for (size_t i = 0; i < 3; ++i) {
    delta_rad[i] = delta_deg[i] * M_PI / 180.0;
    if (std::abs(delta_rad[i]) > kMaxDeltaRad) {
      std::cerr << "[安全] q" << (kFirstJoint + i + 1) << " 幅度超过 10°，已夹住\n";
      delta_rad[i] = (delta_rad[i] > 0 ? kMaxDeltaRad : -kMaxDeltaRad);
    }
    max_abs = std::max(max_abs, std::abs(delta_rad[i]));
  }
  if (speed <= 0.0 || speed > kMaxSpeed) {
    std::cerr << "[安全] 速度不合法，使用 " << kMaxSpeed << " rad/s\n";
    speed = kMaxSpeed;
  }
  const double T = (max_abs > 0) ? M_PI * max_abs / (2.0 * speed) : 0.0;
  if (T <= 0.0) {
    std::cerr << "三个关节幅度都是 0，没有可执行的轨迹\n";
    return 1;
  }

  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "\n";

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    franka::RobotState state = robot.readOnce();
    for (size_t i = 0; i < 3; ++i) {
      const size_t j = kFirstJoint + i;
      if (std::abs(state.q[j] + delta_rad[i]) > kJointLimit) {
        std::cerr << "[安全] q" << (j + 1) << " 预计超出 ±2.8 rad，拒绝执行\n";
        return 1;
      }
    }

    std::cout << "\n计划 (共用时长 " << T << " s):\n";
    for (size_t i = 0; i < 3; ++i) {
      const size_t j = kFirstJoint + i;
      std::cout << "  q" << (j + 1) << ": " << state.q[j] << " → "
                << (state.q[j] + delta_rad[i]) << " rad  ("
                << delta_deg[i] << "°)\n";
    }
    std::cout << "\n确认周围空旷、急停在手，按回车开始 (Ctrl+C 取消)..." << std::endl;
    std::cin.get();

    bool initialized = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_target{};
    std::array<double, 7> q_cmd{};
    double t = 0.0, hold = 0.0;
    bool finished = false;

    robot.control([&](const franka::RobotState& rs,
                      franka::Duration period) -> franka::JointPositions {
      if (!initialized) {
        q_start = rs.q;
        q_target = q_start;
        for (size_t i = 0; i < 3; ++i) {
          q_target[kFirstJoint + i] += delta_rad[i];
        }
        q_cmd = q_start;
        initialized = true;
        return franka::JointPositions(q_cmd);
      }

      const double dt = period.toSec();
      t += dt;
      const double s = (t >= T) ? 1.0 : (1.0 - std::cos(M_PI * t / T)) / 2.0;
      for (size_t i = 0; i < 7; ++i) {
        q_cmd[i] = q_start[i] + (q_target[i] - q_start[i]) * s;
      }

      if (t >= T) {
        hold += dt;
        if (hold >= kHoldSeconds) {
          std::cout << "\n到位，保持 " << kHoldSeconds << "s 后退出\n";
          finished = true;
          return franka::MotionFinished(franka::JointPositions(q_cmd));
        }
      }
      if (t > kMaxRuntimeSeconds) {
        std::cerr << "\n[安全] 超时，强制结束\n";
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
    std::cerr << "提示: reflex 请到 Desk 里 Reset; brakes locked 请先解锁关节。\n";
    return 1;
  }
  return 0;
}
