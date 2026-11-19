// safe_fk.cpp — 用正运动学估算「当前位姿 → 目标位姿」末端会走多远（不动机械臂）
//
// 用法:
//   ./safe_fk <IP> <姿态名>
// 例:
//   ./safe_fk 172.16.0.2 pos2
//
// 输出: 当前位置、目标位置、直线距离、姿态差角、各关节差值。
// 只读 + 本地计算，不会给机械臂发任何运动指令。

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {

std::array<double, 7> parseQ(const std::string& text, bool* ok) {
  std::array<double, 7> q{};
  *ok = false;
  size_t pos = text.find("\"q\"");
  if (pos == std::string::npos) return q;
  pos = text.find('[', pos);
  if (pos == std::string::npos) return q;
  size_t end = text.find(']', pos);
  if (end == std::string::npos) return q;
  std::string body = text.substr(pos + 1, end - pos - 1);
  for (char& c : body) {
    if (c == ',') c = ' ';
  }
  std::istringstream iss(body);
  for (size_t i = 0; i < 7; ++i) {
    if (!(iss >> q[i])) return q;
  }
  *ok = true;
  return q;
}

std::string posePath(const std::string& name) {
  const char* h = std::getenv("HOME");
  return std::string(h ? h : ".") + "/franka-poses/" + name + ".json";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0] << " <IP> <姿态名>\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string name = argv[2];

  std::ifstream in(posePath(name));
  if (!in) {
    std::cerr << "找不到 " << posePath(name) << "\n";
    return 1;
  }
  std::stringstream buf;
  buf << in.rdbuf();
  bool ok = false;
  auto q_goal = parseQ(buf.str(), &ok);
  if (!ok) {
    std::cerr << "解析失败\n";
    return 1;
  }

  try {
    franka::Robot robot(ip);
    franka::Model model = robot.loadModel();
    franka::RobotState st = robot.readOnce();

    const std::array<double, 16> T_now =
        model.pose(franka::Frame::kEndEffector, st);

    franka::RobotState st_goal = st;
    for (size_t i = 0; i < 7; ++i) st_goal.q[i] = q_goal[i];
    const std::array<double, 16> T_goal =
        model.pose(franka::Frame::kEndEffector, st_goal);

    auto pos = [](const std::array<double, 16>& T) {
      return std::array<double, 3>{T[12], T[13], T[14]};
    };
    const auto p0 = pos(T_now);
    const auto p1 = pos(T_goal);

    const double dx = p1[0] - p0[0], dy = p1[1] - p0[1], dz = p1[2] - p0[2];
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // 姿态差: 旋转矩阵迹 → 角度
    auto rotAngle = [](const std::array<double, 16>& A,
                       const std::array<double, 16>& B) {
      double tr = 0.0;
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) tr += A[c * 4 + r] * B[c * 4 + r];
      }
      double cosv = (tr - 1.0) / 2.0;
      if (cosv > 1.0) cosv = 1.0;
      if (cosv < -1.0) cosv = -1.0;
      return std::acos(cosv) * 180.0 / M_PI;
    };

    std::printf("\n当前位置 (base):  x=%7.4f  y=%7.4f  z=%7.4f  m\n", p0[0], p0[1], p0[2]);
    std::printf("目标位置 (base):  x=%7.4f  y=%7.4f  z=%7.4f  m  [%s]\n", p1[0], p1[1], p1[2], name.c_str());
    std::printf("末端直线距离:     %.4f m (%.1f cm)\n", dist, dist * 100.0);
    std::printf("姿态变化角:       %.1f°\n", rotAngle(T_now, T_goal));

    std::printf("\n各关节差值:\n");
    double maxd = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      const double d = q_goal[i] - st.q[i];
      maxd = std::max(maxd, std::abs(d));
      std::printf("  q%zu: %8.2f° → %8.2f°   Δ=%8.2f°\n", i + 1,
                  st.q[i] * 180.0 / M_PI, q_goal[i] * 180.0 / M_PI,
                  d * 180.0 / M_PI);
    }
    std::printf("\n最大单关节差: %.1f°\n", maxd * 180.0 / M_PI);

    // 粗略风险提示
    if (q_goal[1] > 0.0) {
      std::printf("[提示] 目标 q2 = %.1f° 为正 → 大臂会向后仰，确认后方没有墙/障碍\n",
                  q_goal[1] * 180.0 / M_PI);
    }
    if (dist > 0.5) {
      std::printf("[提示] 末端要移动 %.1f cm，跨度较大 → 建议速度 ≤ 0.2 rad/s\n",
                  dist * 100.0);
    }
    if (std::abs(q_goal[5]) > 2.5) {
      std::printf("[提示] 目标 q6 = %.1f° 接近腕部限位\n",
                  q_goal[5] * 180.0 / M_PI);
    }
  } catch (const franka::Exception& e) {
    std::cerr << "[异常] " << e.what() << "\n";
    return 1;
  }
  return 0;
}
