// safe_gripper.cpp — 夹爪工具：查询状态 / 张合 / 回零
//
// 用法:
//   ./safe_gripper <IP> status              # 只读状态，不动作
//   ./safe_gripper <IP> open [宽度m]        # 张开到指定宽度 (默认 0.08)
//   ./safe_gripper <IP> grasp <宽度m> <力N> # 夹到指定宽度，指定力
//   ./safe_gripper <IP> home                # 回零 (homing)，会张合一次
//
// 注意: width=0.0 表示"闭合到底"，会让手指堵转，不建议用来测试；
//       空夹测试建议用 0.01 (留 1cm 缝)。

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/gripper.h>

namespace {
void print(const franka::GripperState& gs, const char* tag) {
  std::printf("[%s] 宽度=%.4f m (%.1f mm), 最大=%.4f m, 夹住=%s, 温度=%.1f°C\n",
              tag, gs.width, gs.width * 1000.0, gs.max_width,
              gs.is_grasped ? "是" : "否", gs.temperature);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cout << "用法: " << argv[0] << " <IP> status|open|half|grasp|home [参数]\n"
              << "  status            查询状态(不动)\n"
              << "  open [宽度m]      张开(默认0.08)\n"
              << "  half [力N]        夹到一半开口(默认20N)\n"
              << "  grasp <宽度m> <力N>  夹持到指定宽度\n"
              << "  home              回零(会张合一次)\n";
    return 1;
  }
  const std::string ip = argv[1];
  const std::string cmd = argv[2];

  try {
    franka::Gripper gripper(ip);
    std::cout << "已连接夹爪 " << ip << "\n";
    print(gripper.readOnce(), "当前");

    if (cmd == "status") {
      return 0;
    }
    if (cmd == "open") {
      double w = (argc > 3) ? std::atof(argv[3]) : 0.08;
      if (w <= 0.0 || w > 0.08) w = 0.08;
      std::cout << "\n张开到 " << w << " m ...\n";
      const bool ok = gripper.move(w, 0.1);
      print(gripper.readOnce(), "结果");
      std::cout << (ok ? "成功\n" : "未达到目标宽度\n");
      return ok ? 0 : 1;
    }
    if (cmd == "half") {
      const double f = (argc > 3) ? std::atof(argv[3]) : 20.0;
      const double force = (f > 0.0 && f <= 70.0) ? f : 20.0;
      const double max_w = gripper.readOnce().max_width;
      const double w = max_w / 2.0;
      std::printf("\n夹一半: 最大开口 %.4f m → 目标 %.4f m (%.1f mm), 力 %.1f N ...\n",
                  max_w, w, w * 1000.0, force);
      const bool ok = gripper.grasp(w, 0.1, force, 0.005, 0.005);
      const franka::GripperState gs = gripper.readOnce();
      print(gs, "结果");
      std::cout << (ok ? "到达目标宽度\n" : "未到达目标宽度(手指被挡住)\n");
      const bool grasped = (gs.width > w + 0.005);
      std::printf("判定: %s (实际 %.1f mm vs 命令 %.1f mm)\n",
                  grasped ? "夹到物体" : "空夹", gs.width * 1000.0, w * 1000.0);
      return 0;
    }
    if (cmd == "grasp") {
      if (argc < 5) {
        std::cerr << "grasp 需要 <宽度m> <力N>\n";
        return 1;
      }
      double w = std::atof(argv[3]);
      double f = std::atof(argv[4]);
      if (w < 0.0 || w > 0.08) w = 0.01;
      if (f <= 0.0 || f > 70.0) f = 20.0;
      std::cout << "\n夹持: 宽度 " << w << " m, 力 " << f << " N ...\n";
      const bool ok = gripper.grasp(w, 0.1, f, 0.005, 0.005);
      const franka::GripperState gs = gripper.readOnce();
      print(gs, "结果");
      std::cout << (ok ? "达到目标宽度\n" : "未达到目标宽度(手指被挡住)\n");
      const bool grasped = (gs.width > w + 0.005);
      std::printf("判定: %s (实际 %.1f mm vs 命令 %.1f mm)\n",
                  grasped ? "夹到物体" : "空夹", gs.width * 1000.0, w * 1000.0);
      return 0;
    }
    if (cmd == "home") {
      std::cout << "\n回零中 (会张合一次) ...\n";
      gripper.homing();
      print(gripper.readOnce(), "结果");
      std::cout << "完成\n";
      return 0;
    }
    std::cerr << "未知命令: " << cmd << "\n";
    return 1;
  } catch (const franka::Exception& e) {
    std::cerr << "\n[异常] " << e.what() << "\n"
              << "若提示连接失败，检查 Desk 里 End Effector 是否 Connected，"
              << "并点 Reinitialize。\n";
    return 1;
  }
}
