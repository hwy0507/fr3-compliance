// safe_goto_retry.cpp — 带自动恢复的关节空间移动
//
// 用法:
//   ./safe_goto_retry <IP> <姿态名> [峰值速度rad/s] [最大尝试次数]
// 例:
//   ./safe_goto_retry 172.16.0.2 pos2 0.1 8
//
// 为什么需要它:
//   系统负载高时会偶发 joint_motion_generator_acceleration_discontinuity,
//   机器人进入 reflex 并中止运动。本程序在异常后调用 automaticErrorRecovery()
//   清除 reflex, 然后从当前实际位置重新规划剩余行程, 直到到位或次数用尽。
//
// 每段都用余弦轨迹, 从控制循环第一帧的实际状态出发。

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

constexpr double kTolRad = 0.03;      // 到位判据（每关节）
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

// 单次尝试：从当前实际位置走到目标。返回是否成功走完。
bool attempt(franka::Robot& robot, const std::array<double, 7>& q_goal,
             double speed) {
  bool initialized = false;
  std::array<double, 7> q_start{};
  std::array<double, 7> q_cmd{};
  double T = 0.0, t = 0.0, hold = 0.0;

  robot.control(
      [&](const franka::RobotState& rs,
          franka::Duration period) -> franka::JointPositions {
        if (!initialized) {
          q_start = rs.q;
          q_cmd = q_start;
          double md = 0.0;
          for (size_t i = 0; i < 7; ++i) {
            md = std::max(md, std::abs(q_goal[i] - q_start[i]));
          }
          T = (md > 0) ? M_PI * md / (2.0 * speed) : 0.0;
          initialized = true;
          return franka::JointPositions(q_cmd);
        }
        const double dt = period.toSec();
        t += dt;
        const double s =
            (T <= 0.0 || t >= T) ? 1.0 : (1.0 - std::cos(M_PI * t / T)) / 2.0;
        for (size_t i = 0; i < 7; ++i) {
          q_cmd[i] = q_start[i] + (q_goal[i] - q_start[i]) * s;
        }
        if (t >= T) {
          hold += dt;
          if (hold >= kHoldSeconds) {
            return franka::MotionFinished(franka::JointPositions(q_cmd));
          }
        }
        return franka::JointPositions(q_cmd);
      },
      franka::ControllerMode::kJointImpedance, true, 100.0);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0]
              << " <IP> <姿态名> [峰值速度rad/s] [最大尝试次数] [容差rad]\n"
              << "例:   " << argv[0] << " 172.16.0.2 pos2 0.1 8\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string name = argv[2];
  double speed = (argc > 3) ? std::atof(argv[3]) : 0.1;
  int max_tries = (argc > 4) ? std::atoi(argv[4]) : 8;
  double tol = (argc > 5) ? std::atof(argv[5]) : 0.01;   // rad, 默认约 0.6°
  if (tol <= 0.0 || tol > 0.3) tol = 0.01;
  if (speed <= 0.0 || speed > 1.0) speed = 0.1;
  if (max_tries < 1 || max_tries > 30) max_tries = 8;

  std::array<double, 7> q_goal{};
  {
    std::ifstream in(posePath(name));
    if (!in) {
      std::cerr << "找不到 " << posePath(name) << "\n";
      return 1;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    if (!parseQ(buf.str(), &q_goal)) {
      std::cerr << "解析失败\n";
      return 1;
    }
  }

  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "  目标: " << name << "  速度 "
              << speed << " rad/s  最多 " << max_tries << " 次\n";

    for (int attempt_no = 1; attempt_no <= max_tries; ++attempt_no) {
      franka::RobotState st = robot.readOnce();
      double md = 0.0;
      for (size_t i = 0; i < 7; ++i) {
        md = std::max(md, std::abs(q_goal[i] - st.q[i]));
      }
      std::printf("[尝试 %d] 当前最大关节差 %.4f rad (%.1f°)\n", attempt_no, md,
                  md * 180.0 / M_PI);
      if (md < tol) {
        std::cout << "已在容差范围内，完成。\n";
        return 0;
      }
      try {
        attempt(robot, q_goal, speed);
        std::cout << "  本次走完，到位。\n";
        return 0;
      } catch (const franka::Exception& e) {
        std::cerr << "  [中断] " << e.what() << "\n";
        try {
          robot.automaticErrorRecovery();
          std::cout << "  已自动恢复 (automaticErrorRecovery)\n";
        } catch (const franka::Exception& e2) {
          std::cerr << "  自动恢复失败: " << e2.what() << "\n"
                    << "  → 需要到 Desk 里手动 Reset\n";
          return 1;
        }
      }
    }
    std::cerr << "达到最大尝试次数仍未到位\n";
    return 1;
  } catch (const franka::Exception& e) {
    std::cerr << "[异常] " << e.what() << "\n";
    return 1;
  }
  return 0;
}
