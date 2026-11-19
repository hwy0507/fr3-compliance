// safe_soft_move.cpp — 柔顺运动：低刚度阻抗模式，被推也不急停
//
// 用法:
//   ./safe_soft_move <IP> <姿态名> [峰值速度rad/s] [刚度Nm/rad]
// 例:
//   ./safe_soft_move 172.16.0.2 pos2 0.2 300
//
// 原理:
//   位置控制下，你推机械臂 = 它用力顶回来 = 关节力矩飙升 -> tau_J_range_violation。
//   阻抗控制下，刚度调低(默认 3000 -> 300)，机械臂"让"着你走，
//   力矩始终很小，因此不会被力矩限制打断。
//
// 行为:
//   1. 设关节刚度 = 参数值 (默认 300)
//   2. 用 kJointImpedance 模式按余弦轨迹走到目标
//   3. 全程打印外部力矩/力
//   4. 结束后恢复默认刚度 3000
//
// ⚠️ 刚度越低越"软"：200 = 很软(会明显下垂), 300~500 = 柔顺, 1000+ = 偏硬

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

constexpr double kHoldSeconds = 1.0;
constexpr double kMaxSegmentSeconds = 90.0;
constexpr std::array<double, 7> kDefaultStiffness = {3000, 3000, 3000, 2500,
                                                     2500, 2000, 2000};

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

// 抗干扰阈值: contact 保留可观测, collision 放开(1000) -> 推撞不触发 reflex
void setCollisionSoft(franka::Robot& r) {
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
  if (argc < 3) {
    std::cout << "用法: " << argv[0]
              << " <IP> <姿态名> [峰值速度rad/s] [刚度Nm/rad]\n"
              << "例:   " << argv[0] << " 172.16.0.2 pos2 0.2 300\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string name = argv[2];
  double speed = (argc > 3) ? std::atof(argv[3]) : 0.2;
  double stiffness = (argc > 4) ? std::atof(argv[4]) : 300.0;

  if (speed <= 0.0 || speed > 1.0) speed = 0.2;
  if (stiffness < 50.0 || stiffness > 3000.0) {
    std::cerr << "[安全] 刚度应在 50~3000，使用 300\n";
    stiffness = 300.0;
  }

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

  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "\n";

    setCollisionSoft(robot);   // 抗干扰: 推撞不触发 reflex
    std::array<double, 7> K;
    K.fill(stiffness);
    robot.setJointImpedance(K);
    std::cout << "关节刚度设为 " << stiffness
              << " Nm/rad（默认 3000，越低越软）\n";

    franka::RobotState st0 = robot.readOnce();
    double max_delta = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      max_delta = std::max(max_delta, std::abs(q_goal[i] - st0.q[i]));
    }
    const double T = (max_delta > 0) ? M_PI * max_delta / (2.0 * speed) : 0.0;
    std::printf("目标 %s，最大关节差 %.3f rad，预计 %.1f s\n", name.c_str(),
                max_delta, T);
    std::cout << "阻抗模式下可以推它，它会顺着让、不会急停。\n"
              << "按回车开始 (Ctrl+C 取消)...\n";
    std::cin.get();

    bool initialized = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_cmd{};
    double t = 0.0, hold = 0.0, next_print = 0.0;
    double max_tau = 0.0, max_dev = 0.0, max_force = 0.0;

    std::printf("\n %7s | %11s | %9s | %9s\n", "t(s)", "max|tau|Nm", "|F|N", "偏差deg");
    std::printf("---------+-------------+-----------+-----------\n");

    robot.control(
        [&](const franka::RobotState& rs,
            franka::Duration period) -> franka::JointPositions {
          if (!initialized) {
            q_start = rs.q;
            q_cmd = q_start;
            initialized = true;
            return franka::JointPositions(q_cmd);
          }
          const double dt = period.toSec();
          t += dt;

          const double s = (T <= 0.0 || t >= T)
                               ? 1.0
                               : (1.0 - std::cos(M_PI * t / T)) / 2.0;
          for (size_t i = 0; i < 7; ++i) {
            q_cmd[i] = q_start[i] + (q_goal[i] - q_start[i]) * s;
          }

          double mt = 0.0, md = 0.0;
          for (size_t i = 0; i < 7; ++i) {
            mt = std::max(mt, std::abs(rs.tau_ext_hat_filtered[i]));
            md = std::max(md, std::abs(rs.q[i] - q_cmd[i]) * 180.0 / M_PI);
          }
          const double mf = std::sqrt(rs.O_F_ext_hat_K[0] * rs.O_F_ext_hat_K[0] +
                                      rs.O_F_ext_hat_K[1] * rs.O_F_ext_hat_K[1] +
                                      rs.O_F_ext_hat_K[2] * rs.O_F_ext_hat_K[2]);
          max_tau = std::max(max_tau, mt);
          max_dev = std::max(max_dev, md);
          max_force = std::max(max_force, mf);

          if (t >= next_print) {
            std::printf(" %7.2f | %11.2f | %9.2f | %9.2f\n", t, mt, mf, md);
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
        },
        franka::ControllerMode::kJointImpedance);

    std::printf("\n峰值: max|tau_ext| = %.2f Nm, max|F_ext| = %.2f N, "
                "最大被动偏移 = %.2f°\n",
                max_tau, max_force, max_dev);

    std::cout << "恢复默认刚度与常规碰撞阈值...\n";
    robot.setJointImpedance(kDefaultStiffness);
    setCollisionNormal(robot);
    std::cout << "完成。机械臂停在 " << name << "。\n";
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    std::cerr << "提示: 若刚度设得过低(如 100 以下)可能因重力下垂触发关节限位。\n";
    return 1;
  }
  return 0;
}
