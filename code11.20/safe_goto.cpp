// safe_goto.cpp — 关节空间平滑回放到已记录的姿态
//
// 用法:
//   ./safe_goto <IP> <名字> [峰值速度rad/s]
// 例:
//   ./safe_goto 172.16.0.2 home 0.4
//
// 行为:
//   1. 读取 ~/franka-poses/<名字>.json 里的 q
//   2. 读取机械臂当前 q，打印两边的角度差
//   3. 按余弦曲线 (起止速度/加速度连续) 从当前走到目标，各关节同时到位
//   4. 到位后保持 1 秒，停在目标位置
//
// 为什么走关节空间而不是笛卡尔:
//   不涉及逆解，不存在"解不出来"或"翻手腕/换肘型"的问题，
//   每次回放到的关节配置完全一致 —— 对 RL 的固定初始状态很重要。
//
// 安全: 速度上限 1.0 rad/s，超限自动夹住；目标必须已记录且关节在 ±2.8 rad 内。

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

constexpr double kMaxSpeed = 1.0;      // rad/s
constexpr double kJointLimit = 2.8;    // rad
constexpr double kHoldSeconds = 1.0;
constexpr double kMaxRuntimeSeconds = 30.0;

std::string posePath(const std::string& name) {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/franka-poses/" + name + ".json";
}

// 从 JSON 文本里取出 "q": [ ... ] 的 7 个数 (足够用的简易解析)
bool parseQ(const std::string& text, std::array<double, 7>* q) {
  const std::string key = "\"q\"";
  size_t pos = text.find(key);
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

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0] << " <IP> <名字> [峰值速度rad/s]\n"
              << "例:   " << argv[0] << " 172.16.0.2 home 0.4\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string name = argv[2];
  double speed = (argc > 3) ? std::atof(argv[3]) : 0.4;

  if (speed <= 0.0 || speed > kMaxSpeed) {
    std::cerr << "[安全] 速度不合法，使用 " << (speed > 0 ? kMaxSpeed : 0.1)
              << " rad/s\n";
    speed = (speed > 0 ? kMaxSpeed : 0.1);
  }

  const std::string path = posePath(name);
  std::ifstream in(path);
  if (!in) {
    std::cerr << "找不到姿态文件: " << path << "\n"
              << "先用 safe_record 记录一个。\n";
    return 1;
  }
  std::stringstream buf;
  buf << in.rdbuf();
  const std::string text = buf.str();

  std::array<double, 7> q_goal{};
  if (!parseQ(text, &q_goal)) {
    std::cerr << "解析失败: " << path << " 里没有合法的 7 维 q\n";
    return 1;
  }
  for (double v : q_goal) {
    if (std::abs(v) > kJointLimit) {
      std::cerr << "[安全] 目标关节角超出 ±2.8 rad，拒绝执行\n";
      return 1;
    }
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

    double max_delta = 0.0;
    std::cout << "\n目标: " << name << "  (" << path << ")\n";
    for (size_t i = 0; i < 7; ++i) {
      const double d = q_goal[i] - st.q[i];
      max_delta = std::max(max_delta, std::abs(d));
      std::printf("  q%zu: %9.4f → %9.4f   Δ=%8.4f rad (%7.2f°)\n", i + 1,
                  st.q[i], q_goal[i], d, d * 180.0 / M_PI);
    }

    const double T = (max_delta > 0) ? M_PI * max_delta / (2.0 * speed) : 0.0;
    std::cout << "\n最大关节差 " << max_delta << " rad，峰值速度 " << speed
              << " rad/s → 预计用时 " << T << " s\n";
    if (T > kMaxRuntimeSeconds) {
      std::cerr << "[安全] 预计用时超过 " << kMaxRuntimeSeconds
                << "s，请提高速度参数\n";
      return 1;
    }
    std::cout << "\n确认周围空旷、急停在手，按回车开始 (Ctrl+C 取消)..." << std::endl;
    std::cin.get();

    bool initialized = false;
    std::array<double, 7> q_start{};
    std::array<double, 7> q_cmd{};
    double t = 0.0, hold = 0.0;
    bool finished = false;

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
      std::cout << "完成。机械臂停在 " << name << " 姿态。\n";
    }
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n"
              << "提示: reflex 需在 Desk 里 Reset；brakes locked 需先解锁关节。\n";
    return 1;
  }
  return 0;
}
