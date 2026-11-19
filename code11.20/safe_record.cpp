// safe_record.cpp — 示教记录：把当前机械臂姿态存成 JSON
//
// 用法:
//   ./safe_record <IP> [名字]
// 例:
//   ./safe_record 172.16.0.2 home
//     → 存到 ~/franka-poses/home.json
//
// 流程: 先在 Desk 里解锁并用手把机械臂拖到目标姿态，然后运行本程序，
//       它会读一次当前状态并写文件 (不会让机械臂动)。
//
// 记录内容: q(7 关节角, rad) + O_T_EE(4x4 位姿, 列主序, 与 libfranka 一致)

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <franka/exception.h>
#include <franka/robot.h>

namespace {

std::string poseDir() {
  const char* home = std::getenv("HOME");
  return std::string(home ? home : ".") + "/franka-poses";
}

bool ensureDir(const std::string& dir) {
  struct stat st{};
  if (stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
  return mkdir(dir.c_str(), 0755) == 0;
}

std::string nowString() {
  std::time_t t = std::time(nullptr);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", std::localtime(&t));
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "用法: " << argv[0] << " <IP> [名字]\n"
              << "例:   " << argv[0] << " 172.16.0.2 home\n";
    return 1;
  }
  const std::string ip = argv[1];
  std::string name = (argc > 2) ? argv[2] : "";

  try {
    franka::Robot robot(ip);
    std::cout << "已连接 " << ip << "\n";

    franka::RobotState st = robot.readOnce();

    std::cout << "\n当前关节角:\n";
    for (size_t i = 0; i < 7; ++i) {
      std::printf("  q%zu = %9.5f rad  (%8.2f°)\n", i + 1, st.q[i],
                  st.q[i] * 180.0 / M_PI);
    }
    std::cout << "\n当前末端位置 (base 坐标系, m):\n";
    std::printf("  x = %8.4f  y = %8.4f  z = %8.4f\n",
                st.O_T_EE[12], st.O_T_EE[13], st.O_T_EE[14]);

    // 简单的姿态合理性提示
    for (size_t i = 0; i < 7; ++i) {
      if (std::abs(st.q[i]) > 2.60) {
        std::cout << "  [提示] q" << (i + 1) << " 接近关节限位 (" << st.q[i]
                  << " rad)，建议留更多余量\n";
      }
    }
    if (std::abs(st.q[3] + 1.5708) < 0.15) {
      std::cout << "  [提示] q4 接近 -90°，属正常抓取姿态\n";
    }

    if (name.empty()) {
      std::cout << "\n给这个姿态起个名字 (如 home / pick / place): ";
      std::getline(std::cin, name);
    }
    if (name.empty()) {
      std::cerr << "名字为空，取消\n";
      return 1;
    }

    const std::string dir = poseDir();
    if (!ensureDir(dir)) {
      std::cerr << "无法创建目录 " << dir << "\n";
      return 1;
    }
    const std::string path = dir + "/" + name + ".json";
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
      std::cerr << "无法写入 " << path << "\n";
      return 1;
    }

    std::fprintf(f, "{\n  \"name\": \"%s\",\n", name.c_str());
    std::fprintf(f, "  \"created\": \"%s\",\n", nowString().c_str());
    std::fprintf(f, "  \"source_ip\": \"%s\",\n", ip.c_str());
    std::fprintf(f, "  \"note\": \"O_T_EE is column-major, same as libfranka\",\n");

    std::fprintf(f, "  \"q\": [");
    for (size_t i = 0; i < 7; ++i) {
      std::fprintf(f, "%.17g%s", st.q[i], (i < 6) ? ", " : "");
    }
    std::fprintf(f, "],\n");

    std::fprintf(f, "  \"O_T_EE\": [");
    for (size_t i = 0; i < 16; ++i) {
      std::fprintf(f, "%.17g%s", st.O_T_EE[i], (i < 15) ? ", " : "");
    }
    std::fprintf(f, "]\n}\n");
    std::fclose(f);

    std::cout << "\n已保存: " << path << "\n";
    std::cout << "回放: " << "safe_goto " << ip << " " << name << "\n";
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n";
    return 1;
  }
  return 0;
}
