// safe_move_nostop.cpp — 抗干扰运动：中途被推/撞也不急停
//
// 用法:
//   ./safe_move_nostop <IP> <目标姿态名> [峰值速度rad/s]
// 例:
//   ./safe_move_nostop 172.16.0.2 pos2 0.2
//
// 行为:
//   1. 设定阈值: contact 20 Nm/15 N, collision 1000 Nm/1000 N
//      → FCI 层的碰撞 reflex 实际被关闭, 被推撞不会停机
//   2. 按余弦轨迹从当前位姿走到目标姿态
//   3. 全程每 200ms 打印 外部力矩/力 与 contact 标志
//   4. 结束后自动把阈值恢复为常规值 (normal)
//
// ⚠️ 注意:
//   - 机械臂会沿轨迹继续走并"顶"回来, 被挡住时会持续用力
//   - 不要把手指卡在关节/夹具之间
//   - 安全层(Desk Safety Configuration)与急停按钮仍然有效
//   - 若 Ctrl+C 中断, 阈值不会恢复, 请手动执行:
//       ./safe_compliant 172.16.0.2 normal 1

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

constexpr double kHoldSeconds = 1.0;
constexpr double kMaxSegmentSeconds = 60.0;

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

void setNormal(franka::Robot& r) {
  r.setCollisionBehavior({{15, 15, 12, 12, 10, 8, 6}},
                         {{40, 40, 36, 36, 32, 28, 24}},
                         {{15, 15, 12, 12, 10, 8, 6}},
                         {{40, 40, 36, 36, 32, 28, 24}},
                         {{10, 10, 10, 15, 15, 15}},
                         {{40, 40, 40, 60, 60, 60}},
                         {{10, 10, 10, 15, 15, 15}},
                         {{40, 40, 40, 60, 60, 60}});
}

void setHigh(franka::Robot& r) {
  r.setCollisionBehavior({{20, 20, 18, 18, 15, 12, 10}},
                         {{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
                         {{20, 20, 18, 18, 15, 12, 10}},
                         {{1000, 1000, 1000, 1000, 1000, 1000, 1000}},
                         {{15, 15, 15, 20, 20, 20}},
                         {{1000, 1000, 1000, 1000, 1000, 1000}},
                         {{15, 15, 15, 20, 20, 20}},
                         {{1000, 1000, 1000, 1000, 1000, 1000}});
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0] << " <IP> <姿态名> [峰值速度rad/s]\n"
              << "例:   " << argv[0] << " 172.16.0.2 pos2 0.2\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string name = argv[2];
  double speed = (argc > 3) ? std::atof(argv[3]) : 0.2;
  if (speed <= 0.0 || speed > 1.0) speed = 0.2;

  std::array<double, 7> q_goal{};
  {
    std::ifstream in(posePath(name));
    if (!in) {
      std::cerr << "找不到姿态文件: " << posePath(name) << "\n";
      return 1;
    }
    std::stringstream buf;
    buf << in.rdbuf();
    if (!parseQ(buf.str(), &q_goal)) {
      std::cerr << "解析失败\n";
      return 1;
    }
  }

  bool thresholds_set = false;
  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "\n";
    setHigh(robot);
    thresholds_set = true;
    std::cout << "阈值: contact 20Nm/15N, collision 1000Nm/1000N (抗干扰模式)\n";

    franka::RobotState st0 = robot.readOnce();
    double max_delta = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal[i] - st0.q[i]));
    }
    const double T = (max_delta > 0) ? M_PI * max_delta / (2.0 * speed) : 0.0;

    std::cout << "\n目标: " << name << "，最大关节差 " << max_delta
              << " rad，预计 " << T << " s\n"
              << "可以中途推/撞机械臂，它不会急停（但会顶回来）。\n"
              << "按回车开始 (Ctrl+C 取消)...\n";
    std::cin.get();

    bool initialized = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_cmd{};
    double t = 0.0, hold = 0.0, next_print = 0.0;
    double max_tau = 0.0, max_f = 0.0;
    bool contact_seen = false;

    std::printf("\n %7s | %10s | %9s | %8s\n", "t(s)", "max|tau|", "|F|", "contact");
    std::printf("---------+------------+-----------+---------\n");

    robot.control([&](const franka::RobotState& rs,
                      franka::Duration period) -> franka::JointPositions {
      if (!initialized) {
        q_start = rs.q;
        q_cmd = q_start;
        initialized = true;
        return franka::JointPositions(q_cmd);
      }

      const double dt = period.toSec();
      t += dt;

      const double s = (T <= 0.0 || t >= T) ? 1.0
                                            : (1.0 - std::cos(M_PI * t / T)) / 2.0;
      for (size_t i = 0; i < 7; ++i) {
        q_cmd[i] = q_start[i] + (q_goal[i] - q_start[i]) * s;
      }

      double mt = 0.0;
      for (size_t i = 0; i < 7; ++i) {
        mt = std::max(mt, std::abs(rs.tau_ext_hat_filtered[i]));
      }
      const double mf = std::sqrt(rs.O_F_ext_hat_K[0] * rs.O_F_ext_hat_K[0] +
                                  rs.O_F_ext_hat_K[1] * rs.O_F_ext_hat_K[1] +
                                  rs.O_F_ext_hat_K[2] * rs.O_F_ext_hat_K[2]);
      max_tau = std::max(max_tau, mt);
      max_f = std::max(max_f, mf);

      bool contact = false;
      for (size_t i = 0; i < 7; ++i) {
        if (rs.joint_contact[i] > 0) contact = true;
      }
      for (size_t i = 0; i < 6; ++i) {
        if (rs.cartesian_contact[i] > 0) contact = true;
      }
      contact_seen |= contact;

      if (t >= next_print) {
        std::printf(" %7.2f | %10.2f | %9.2f | %8s\n", t, mt, mf,
                    contact ? "YES" : "-");
        next_print += 0.2;
      }

      if (t >= T) {
        hold += dt;
        if (hold >= kHoldSeconds) {
          return franka::MotionFinished(franka::JointPositions(q_cmd));
        }
      }
      if (t > kMaxSegmentSeconds) {
        std::cerr << "[安全] 段超时，结束\n";
        return franka::MotionFinished(franka::JointPositions(q_cmd));
      }
      return franka::JointPositions(q_cmd);
    });

    std::printf("\n峰值: max|tau_ext| = %.2f Nm, max|F_ext| = %.2f N, "
                "出现过 contact: %s\n",
                max_tau, max_f, contact_seen ? "是" : "否");
    std::cout << "恢复常规阈值...\n";
    setNormal(robot);
    thresholds_set = false;
    std::cout << "完成。机械臂停在 " << name << "。\n";
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    if (thresholds_set) {
      std::cerr << "注意: 阈值可能仍处于抗干扰模式，请执行 "
                   "safe_compliant <IP> normal 1 恢复。\n";
    }
    return 1;
  }
  return 0;
}
