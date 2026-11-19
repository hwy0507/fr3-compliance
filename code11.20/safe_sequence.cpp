// safe_sequence.cpp — 按顺序平滑走过多个已记录姿态
//
// 用法:
//   ./safe_sequence <IP> <姿态1> <姿态2> ... [峰值速度rad/s]
// 例:
//   ./safe_sequence 172.16.0.2 home pos2          # 默认 0.3 rad/s
//   ./safe_sequence 172.16.0.2 home pos2 0.2      # 更慢
//
// 行为:
//   不管当前处在什么位姿，先走到第一个姿态，再依次走到后面的姿态。
//   每段都用余弦曲线（起止速度/加速度连续），各关节同时到位，
//   段与段之间停顿 0.5 秒，最后一段结束后保持 1 秒再退出。
//
// 为什么走关节空间: 不涉及逆解，不会出现"解不出来/翻手腕"，
//   每次到达的关节配置完全一致 —— 适合 RL 的固定起始状态。
//
// 安全: 速度上限 1.0 rad/s；关节目标必须在 ±2.8 rad；总超时保护。

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

constexpr double kMaxSpeed = 1.0;         // rad/s
constexpr double kJointLimit = 2.8;       // rad
constexpr double kDwellSeconds = 0.5;     // 段间停顿
constexpr double kFinalHoldSeconds = 1.0; // 最后保持
constexpr double kMaxTotalSeconds = 120.0;

struct Pose {
  std::string name;
  std::array<double, 7> q{};
};

std::string posePath(const std::string& name) {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/franka-poses/" + name + ".json";
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

bool loadPose(const std::string& name, Pose* out) {
  std::ifstream in(posePath(name));
  if (!in) return false;
  std::stringstream buf;
  buf << in.rdbuf();
  out->name = name;
  return parseQ(buf.str(), &out->q);
}

bool isNumber(const std::string& s) {
  if (s.empty()) return false;
  char* end = nullptr;
  std::strtod(s.c_str(), &end);
  return end && *end == '\0';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0] << " <IP> <姿态1> <姿态2> ... [峰值速度rad/s]\n"
              << "例:   " << argv[0] << " 172.16.0.2 home pos2\n"
              << "      " << argv[0] << " 172.16.0.2 home pos2 0.2\n";
    return 1;
  }

  const std::string ip = argv[1];
  std::vector<std::string> names;
  double speed = 0.3;

  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (isNumber(a)) {
      speed = std::atof(a.c_str());
    } else {
      names.push_back(a);
    }
  }
  if (names.empty()) {
    std::cerr << "至少给一个姿态名\n";
    return 1;
  }
  if (speed <= 0.0 || speed > kMaxSpeed) {
    std::cerr << "[安全] 速度不合法，使用 " << kMaxSpeed << " rad/s\n";
    speed = kMaxSpeed;
  }

  std::vector<Pose> poses;
  for (const auto& n : names) {
    Pose p;
    if (!loadPose(n, &p)) {
      std::cerr << "找不到或解析失败: " << posePath(n) << "\n";
      return 1;
    }
    for (double v : p.q) {
      if (std::abs(v) > kJointLimit) {
        std::cerr << "[安全] " << n << " 有关节角超出 ±2.8 rad\n";
        return 1;
      }
    }
    poses.push_back(p);
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

    franka::RobotState st = robot.readOnce();

    std::cout << "\n当前关节角: ";
    for (size_t i = 0; i < 7; ++i) std::printf("%.3f ", st.q[i]);
    std::cout << "\n\n计划 (" << poses.size() << " 段, 峰值速度 " << speed
              << " rad/s):\n";

    // 预估总时长: 假定每段从上一段目标出发
    double total = 0.0;
    std::array<double, 7> prev = st.q;
    for (const auto& p : poses) {
      double max_delta = 0.0;
      for (size_t i = 0; i < 7; ++i) {
        max_delta = std::max(max_delta, std::abs(p.q[i] - prev[i]));
      }
      const double T = (max_delta > 0) ? M_PI * max_delta / (2.0 * speed) : 0.0;
      total += T + kDwellSeconds;
      std::printf("  → %-10s (最大关节差 %6.3f rad, 约 %5.2f s)\n",
                  p.name.c_str(), max_delta, T);
      prev = p.q;
    }
    std::printf("  预计总时长 ≈ %.1f s (+ 最后保持 %.1f s)\n", total,
                kFinalHoldSeconds);

    if (total > kMaxTotalSeconds) {
      std::cerr << "[安全] 预计时长超过 " << kMaxTotalSeconds
                << "s，请提高速度\n";
      return 1;
    }

    std::cout << "\n确认周围空旷、急停在手，按回车开始 (Ctrl+C 取消)..." << std::endl;
    std::cin.get();

    // ---- 状态机 ----
    bool initialized = false;
    size_t idx = 0;
    bool segment_active = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_goal{};
    std::array<double, 7> q_cmd{};
    double T_seg = 0.0;
    double t = 0.0;
    double hold = 0.0;
    double total_elapsed = 0.0;
    bool finished = false;

    robot.control([&](const franka::RobotState& rs,
                      franka::Duration period) -> franka::JointPositions {
      if (!initialized) {
        q_cmd = rs.q;
        q_start = q_cmd;
        q_goal = poses[0].q;
        double max_delta = 0.0;
        for (size_t i = 0; i < 7; ++i) {
          max_delta = std::max(max_delta, std::abs(q_goal[i] - q_start[i]));
        }
        T_seg = (max_delta > 0) ? M_PI * max_delta / (2.0 * speed) : 0.0;
        t = 0.0;
        hold = 0.0;
        segment_active = true;
        initialized = true;
        std::cout << "[1/" << poses.size() << "] 前往 " << poses[idx].name
                  << std::endl;
        return franka::JointPositions(q_cmd);
      }

      const double dt = period.toSec();
      total_elapsed += dt;

      if (total_elapsed > kMaxTotalSeconds) {
        std::cerr << "\n[安全] 总时长超限，强制结束\n";
        finished = true;
        return franka::MotionFinished(franka::JointPositions(q_cmd));
      }

      if (segment_active) {
        t += dt;
        const double s =
            (T_seg <= 0.0 || t >= T_seg) ? 1.0
                                         : (1.0 - std::cos(M_PI * t / T_seg)) / 2.0;
        for (size_t i = 0; i < 7; ++i) {
          q_cmd[i] = q_start[i] + (q_goal[i] - q_start[i]) * s;
        }
        if (t >= T_seg) {
          segment_active = false;
          hold = 0.0;
          if (idx + 1 >= poses.size()) {
            std::cout << "  已到 " << poses[idx].name
                      << "（最后一段），保持 " << kFinalHoldSeconds << "s"
                      << std::endl;
          } else {
            std::cout << "  已到 " << poses[idx].name << std::endl;
          }
        }
      } else {
        hold += dt;
        const bool is_last = (idx + 1 >= poses.size());
        const double need = is_last ? kFinalHoldSeconds : kDwellSeconds;
        if (hold >= need) {
          if (is_last) {
            finished = true;
            return franka::MotionFinished(franka::JointPositions(q_cmd));
          }
          ++idx;
          q_start = q_cmd;
          q_goal = poses[idx].q;
          double max_delta = 0.0;
          for (size_t i = 0; i < 7; ++i) {
            max_delta = std::max(max_delta, std::abs(q_goal[i] - q_start[i]));
          }
          T_seg = (max_delta > 0) ? M_PI * max_delta / (2.0 * speed) : 0.0;
          t = 0.0;
          hold = 0.0;
          segment_active = true;
          std::cout << "[" << (idx + 1) << "/" << poses.size() << "] 前往 "
                    << poses[idx].name << std::endl;
        }
      }

      return franka::JointPositions(q_cmd);
    });

    if (finished) {
      std::cout << "序列完成。机械臂停在最后一个姿态。\n";
    }
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n"
              << "提示: 若为 User stopped，先到 Desk 锁上/解锁关节或重开 FCI。\n";
    return 1;
  }
  return 0;
}
