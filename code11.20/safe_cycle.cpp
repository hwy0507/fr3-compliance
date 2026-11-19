// safe_cycle.cpp — 一个完整的抓取循环：
//   home ──→ 目标位 ──→ 闭合夹爪 ──→ 回 home ──→ 结束
//
// 用法:
//   ./safe_cycle <IP> <home名> <目标名> [宽度m] [力N] [速度rad/s]
// 例:
//   ./safe_cycle 172.16.0.2 home pos2                  # 默认: 全闭, 20N, 0.3rad/s
//   ./safe_cycle 172.16.0.2 home pos2 0.03 40 0.2      # 夹到 3cm, 40N, 慢速
//
// 夹爪参数说明 (Franka Hand):
//   宽度 width : 目标开口, 单位 m。0.0 = 完全闭合(碰到物体就停)
//   速度 speed : 夹爪速度, 单位 m/s (内部固定 0.1 左右)
//   力   force : 夹持力, 单位 N, 范围 0~70
//
// 关于"测力": Franka Hand 没有力传感器。force 是"设定值",
//   能读回的只有 width / is_grasped / temperature。
//   实际夹持力只能靠设定值 + is_grasped 推断。
//
// 安全: 关节速度上限 1.0 rad/s；关节目标 ±2.8 rad；每段余弦轨迹。

#include <array>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>

namespace {

constexpr double kMaxSpeed = 1.0;
constexpr double kJointLimit = 2.8;
constexpr double kHoldSeconds = 1.0;
constexpr double kMaxSegmentSeconds = 30.0;

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

void setCollision(franka::Robot& robot) {
  // 力阈值提到 40N：抓取接近物体时手指接触属于正常接触，
  // 20N 阈值会在碰到的瞬间触发 cartesian_reflex。
  robot.setCollisionBehavior(
      {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
      {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
      {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
      {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
      {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}},
      {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}},
      {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}},
      {{40.0, 40.0, 40.0, 45.0, 45.0, 45.0}});
}

// 关节空间余弦轨迹移动，起止速度/加速度连续
void moveTo(franka::Robot& robot, const std::array<double, 7>& target,
            double speed, const std::string& label) {
  const double T_est = [&] {
    franka::RobotState st = robot.readOnce();
    double md = 0.0;
    for (size_t i = 0; i < 7; ++i) md = std::max(md, std::abs(target[i] - st.q[i]));
    return (md > 0) ? M_PI * md / (2.0 * speed) : 0.0;
  }();

  bool initialized = false;
  std::array<double, 7> q_start{};
  std::array<double, 7> q_cmd{};
  double T = T_est, t = 0.0, hold = 0.0;

  robot.control([&](const franka::RobotState& rs,
                    franka::Duration period) -> franka::JointPositions {
    if (!initialized) {
      q_start = rs.q;
      q_cmd = q_start;
      double md = 0.0;
      for (size_t i = 0; i < 7; ++i) md = std::max(md, std::abs(target[i] - q_start[i]));
      T = (md > 0) ? M_PI * md / (2.0 * speed) : 0.0;
      initialized = true;
      return franka::JointPositions(q_cmd);
    }
    const double dt = period.toSec();
    t += dt;
    const double s = (T <= 0.0 || t >= T) ? 1.0 : (1.0 - std::cos(M_PI * t / T)) / 2.0;
    for (size_t i = 0; i < 7; ++i) {
      q_cmd[i] = q_start[i] + (target[i] - q_start[i]) * s;
    }
    if (t >= T) {
      hold += dt;
      if (hold >= kHoldSeconds) {
        return franka::MotionFinished(franka::JointPositions(q_cmd));
      }
    }
    if (t > kMaxSegmentSeconds) {
      std::cerr << "  [安全] 段超时，提前结束\n";
      return franka::MotionFinished(franka::JointPositions(q_cmd));
    }
    return franka::JointPositions(q_cmd);
  });
  std::cout << "  已到 " << label << "（用时约 " << T << " s）\n";
}

void printGripper(const franka::GripperState& gs, const char* tag) {
  std::printf("  [夹爪%s] 宽度=%.4f m (%.1f mm), 夹住=%s, 温度=%.1f°C\n", tag,
              gs.width, gs.width * 1000.0, gs.is_grasped ? "是" : "否",
              gs.temperature);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cout << "用法: " << argv[0]
              << " <IP> <home名> <目标名> [宽度m] [力N] [速度rad/s]\n"
              << "例:   " << argv[0] << " 172.16.0.2 home pos2\n"
              << "      " << argv[0] << " 172.16.0.2 home pos2 0.03 40 0.2\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string home_name = argv[2];
  const std::string target_name = argv[3];
  double grasp_width = (argc > 4) ? std::atof(argv[4]) : 0.0;   // m, 0=全闭
  double grasp_force = (argc > 5) ? std::atof(argv[5]) : 20.0;  // N
  double speed = (argc > 6) ? std::atof(argv[6]) : 0.3;         // rad/s

  if (speed <= 0.0 || speed > kMaxSpeed) {
    std::cerr << "[安全] 速度不合法，使用 0.3 rad/s\n";
    speed = 0.3;
  }
  if (grasp_width < 0.0 || grasp_width > 0.08) {
    std::cerr << "[安全] 夹爪宽度应在 0~0.08 m，使用 0\n";
    grasp_width = 0.0;
  }
  if (grasp_force <= 0.0 || grasp_force > 70.0) {
    std::cerr << "[安全] 夹持力应在 0~70 N，使用 20 N\n";
    grasp_force = 20.0;
  }

  std::array<double, 7> q_home{}, q_target{};
  if (!loadQ(home_name, &q_home)) {
    std::cerr << "找不到姿态: " << posePath(home_name) << "\n";
    return 1;
  }
  if (!loadQ(target_name, &q_target)) {
    std::cerr << "找不到姿态: " << posePath(target_name) << "\n";
    return 1;
  }
  for (double v : q_home) {
    if (std::abs(v) > kJointLimit) {
      std::cerr << "[安全] " << home_name << " 有关节超限\n";
      return 1;
    }
  }
  for (double v : q_target) {
    if (std::abs(v) > kJointLimit) {
      std::cerr << "[安全] " << target_name << " 有关节超限\n";
      return 1;
    }
  }

  try {
    franka::Robot robot(ip);
    franka::Gripper gripper(ip);
    std::cout << "已连接 " << ip << "（机械臂 + 夹爪）\n";

    setCollision(robot);
    printGripper(gripper.readOnce(), "初始");

    std::cout << "\n流程: " << home_name << " → " << target_name
              << " → 夹爪(" << (grasp_force) << "N, 宽度 " << grasp_width
              << " m) → " << home_name << "\n"
              << "峰值速度 " << speed << " rad/s\n"
              << "\n确认周围空旷、急停在手，按回车开始 (Ctrl+C 取消)..." << std::endl;
    std::cin.get();

    std::cout << "[1/4] 前往 " << home_name << "\n";
    moveTo(robot, q_home, speed, home_name);

    // 关键: 接近抓取位之前先把夹爪张开，
    // 闭合状态下伸向物体会直接顶在物体上触发 reflex。
    std::cout << "  夹爪张开中...\n";
    try {
      gripper.move(0.08, 0.1);
    } catch (const franka::Exception& e) {
      std::cout << "  (张开失败，继续: " << e.what() << ")\n";
    }
    printGripper(gripper.readOnce(), "张开后");

    std::cout << "[2/4] 前往 " << target_name << "\n";
    moveTo(robot, q_target, speed, target_name);

    std::cout << "[3/4] 夹爪动作\n";
    franka::GripperState before = gripper.readOnce();
    printGripper(before, "动作前");

    const bool ok = gripper.grasp(grasp_width, 0.1, grasp_force, 0.005, 0.005);
    franka::GripperState after = gripper.readOnce();
    printGripper(after, "动作后");

    if (ok) {
      std::cout << "  夹爪命令成功（到达了目标宽度）\n";
    } else {
      std::cout << "  夹爪命令未到达目标宽度（手指被挡住）\n";
    }

    // 宽度判据: 实际宽度明显大于命令宽度 => 指间有物体
    // 不依赖 is_grasped (本机该字段因遥测异常不可用)
    constexpr double kWidthEpsilon = 0.005;  // 5 mm
    const bool grasped = (after.width > grasp_width + kWidthEpsilon);
    std::printf("  判定: %s  (实际 %.1f mm vs 命令 %.1f mm, 判据 实际 > 命令+5mm)\n",
                grasped ? "夹到物体" : "空夹", after.width * 1000.0,
                grasp_width * 1000.0);
    std::printf("  参考: is_grasped=%s (本机遥测异常, 不可信)\n",
                after.is_grasped ? "true" : "false");

    // 保持 1 秒让夹持稳定, 再确认是否滑动
    std::cout << "  保持 1 秒观察是否滑动...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    franka::GripperState settled = gripper.readOnce();
    printGripper(settled, "保持1s后");

    if (settled.width > after.width + 0.001) {
      std::cout << "  检测到滑动，用最大力重新夹紧...\n";
      gripper.grasp(grasp_width, 0.1, grasp_force, 0.005, 0.005);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      settled = gripper.readOnce();
      printGripper(settled, "重夹后");
      after = settled;
    } else {
      std::cout << "  夹持稳定，未见滑动\n";
    }

    std::cout << "[4/4] 返回 " << home_name << "\n";
    moveTo(robot, q_home, speed, home_name);

    franka::GripperState final_state = gripper.readOnce();
    printGripper(final_state, "结束");

    // is_grasped 在本机不可信, 用宽度判断: 明显大于 10mm 说明指间有物体
    const bool holding = (final_state.width > 0.010);
    std::printf("\n循环完成。夹爪当前: %s (宽度 %.1f mm)\n",
                holding ? "保持夹持" : "张开/空夹", final_state.width * 1000.0);
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n"
              << "提示: User stopped → Desk 里锁/解锁关节或重开 FCI; "
              << "reflex → Desk 里 Reset。\n";
    return 1;
  }
  return 0;
}
