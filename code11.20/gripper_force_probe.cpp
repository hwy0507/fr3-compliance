// gripper_force_probe.cpp — 扫掠夹持力，找出 is_grasped 翻转的阈值
//
// 用法:
//   ./gripper_force_probe <IP> [起始N] [结束N] [步长N]
// 例:
//   ./gripper_force_probe 172.16.0.2 5 70 5
//
// 原理: Franka Hand 没有力传感器, force 只是"电流上限设定值"。
//       但 is_grasped 是内部估计出的布尔量, 会随力变化翻转,
//       扫掠一遍就能得到"保持当前物体所需的最小设定力"经验值。
//
// 注意: 扫掠到低力时物体可能滑落, 请手扶或确保下方安全。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <unistd.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "用法: " << argv[0] << " <IP> [起始N] [结束N] [步长N]\n"
              << "例:   " << argv[0] << " 172.16.0.2 5 70 5\n";
    return 1;
  }
  const std::string ip = argv[1];
  double f_start = (argc > 2) ? std::atof(argv[2]) : 5.0;
  double f_end = (argc > 3) ? std::atof(argv[3]) : 70.0;
  double f_step = (argc > 4) ? std::atof(argv[4]) : 5.0;
  if (f_step <= 0.0) f_step = 5.0;
  if (f_start < 0.0) f_start = 0.0;
  if (f_end > 70.0) f_end = 70.0;

  try {
    franka::Gripper gripper(ip);
    franka::GripperState st = gripper.readOnce();
    std::printf("当前: 宽度=%.4f m (%.1f mm), 夹住=%s\n", st.width,
                st.width * 1000.0, st.is_grasped ? "是" : "否");

    const double target_w = st.width;  // 保持当前宽度，只改力
    if (target_w < 0.001) {
      std::cout << "夹爪当前没有夹住东西(宽度接近0)，扫掠无意义\n";
      return 1;
    }
    std::printf("\n保持宽度 %.1f mm，扫掠力 %.0f → %.0f N，步长 %.0f N\n\n",
                target_w * 1000.0, f_start, f_end, f_step);
    std::printf(" %8s | %10s | %8s | %6s\n", "设定力N", "实际宽度mm", "夹住", "结果");
    std::printf("----------+------------+----------+------\n");

    double first_stable = -1.0;
    for (double f = f_start; f <= f_end + 1e-9; f += f_step) {
      try {
        const bool ok = gripper.grasp(target_w, 0.1, f, 0.005, 0.005);
        usleep(300000);  // 等 300ms 让夹持稳定
        franka::GripperState s = gripper.readOnce();
        std::printf(" %8.1f | %10.2f | %8s | %s\n", f, s.width * 1000.0,
                    s.is_grasped ? "是" : "否", ok ? "命令成功" : "未达目标");
        if (s.is_grasped && first_stable < 0.0) first_stable = f;
      } catch (const franka::Exception& e) {
        std::printf(" %8.1f | %10s | %8s | 异常: %s\n", f, "-", "-", e.what());
      }
    }

    std::printf("\n结果汇总:\n");
    if (first_stable > 0) {
      std::printf("  首次判定\"夹住\"的设定力: %.1f N\n", first_stable);
      std::printf("  建议工作力: %.1f N (留 %.0f%% 余量)\n",
                  first_stable * 1.5, 50.0);
    } else {
      std::printf("  整个范围内都没有判定\"夹住\"，可能物体已滑落或检测失效\n");
    }

    // 恢复到最大力，避免物体掉落
    std::cout << "\n恢复到 70N 保持...\n";
    gripper.grasp(target_w, 0.1, 70.0, 0.005, 0.005);
    franka::GripperState fin = gripper.readOnce();
    std::printf("最终: 宽度=%.1f mm, 夹住=%s\n", fin.width * 1000.0,
                fin.is_grasped ? "是" : "否");
  } catch (const franka::Exception& e) {
    std::cerr << "[异常] " << e.what() << "\n";
    return 1;
  }
  return 0;
}
