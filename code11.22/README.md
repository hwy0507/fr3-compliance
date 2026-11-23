# code11.22 — FR3 + ESN 顺应控制套件

本目录包含 2026-11-22 的完整环境快照。

| 项 | 值 |
|---|---|
| 文件 | `franka-fci-suite_代码与参数_2026-11-22.tar.gz` |
| 大小 | 1.4 MB |
| SHA-256 | `57aac9c4102e819181ac3d0af08cfbb7b06a1bb8224e8c11b23002f56204fe58` |

解压并编译：

```bash
tar -xzf franka-fci-suite_代码与参数_2026-11-22.tar.gz
cd franka-fci-suite/franka-safe
cmake -S . -B build && cmake --build build -j$(nproc)
```

---

## 一、这套东西是做什么的

在 Ubuntu 22.04 + PREEMPT_RT 实时内核上，用 **libfranka**（不经过 ROS）控制 **Franka Research 3** 机械臂。核心是一个 **ESN（Echo State Network，128 单元储备池）状态策略**：

- **25 Hz** 读取 48 维观测（关节角/速度、末端名义速度、位姿误差、六维外力、任务方向）
- 输出 **7 个数**：1 个减速激活量 + 6 个让位通道（XYZ 平移 / Roll-Pitch-Yaw 转动）
- **1 kHz** 的执行层把这些数字转成关节位置指令下发给机械臂

达到的效果：

| 情况 | 行为 |
|---|---|
| 无外力 | 指令与纯轨迹完全一致（丝滑，看不出有策略在跑） |
| 检测到接触（>12 N） | 主动减速 + 朝让位方向退开，实测最多 30 cm |
| 外力消失（<9 N 持续 0.3 s） | 让位归零、速度恢复，继续走到目标 |
| 全程 | 不触发急停（reflex），终点误差 2~8 mm |

实测对照（同一路径、相近推力水平）：

| 条件 | 推力 | 主动让位 | 减速 |
|---|---|---|---|
| ESN 开 | 11~18 N | 46~230 mm | 速度降到 50~80% |
| ESN 关 | 25~48 N | **0 mm**（仅 10~30 mm 被动偏移） | 无 |

---

## 二、目录结构

```
franka-fci-suite/
├── franka-safe/        自研工具（13 个程序，约 4000 行 C++）
├── FR3_ESN128_main/    ESN 模型包（策略本体 + 冻结权重）
├── franka-poses/       示教姿态（home / pos2）
├── rt-setup/           实时内核、CPU 隔离、网卡调优脚本
├── scripts/            多段运行脚本（往返）
├── docs/               手册、复现包、实验图表
└── std-logs/           典型运行日志
```

---

## 三、`franka-safe/` —— 每个程序做什么

### 核心控制（带 ESN）

| 文件 | 作用 |
|---|---|
| **`esn_yield.cpp`** | **主程序**。走完"起点→终点"的余弦轨迹，同时每 40 ms 跑一次 ESN：拿减速量调推进速度，拿让位量经雅可比伪逆转成关节偏移。带接触门控、外力零偏校准、二阶平滑、CSV 日志、90 s 超时保护。 |
| `esn_slowdown.cpp` | 只启用减速通道（不用让位），用于单独验证减速效果。 |
| `esn_shadow.cpp` | **影子模式**：机械臂按正常轨迹走，ESN 只计算不施加，用于安全调试观测组装是否合理。 |
| `esn_vs_vmc.cpp` | 离线分析工具。把同一批观测同时喂给 ESN 和 VMC(teacher)，比较两者让位方向与外力方向的一致性。 |

### 运动与姿态

| 文件 | 作用 |
|---|---|
| `safe_record.cpp` | 示教记录：读当前关节角 + 末端位姿，存成 JSON（`~/franka-poses/名字.json`）。 |
| `safe_fk.cpp` | 正运动学校验：**只读不动**，算出"当前位置→目标姿态"末端要走多远、姿态差多少、各关节差多少，并给出风险提示（如目标落在桌面以下、q2 后仰等）。 |
| `safe_goto.cpp` | 关节空间余弦插值回放到某个姿态，位置控制，精度约 0.1 mm。 |
| `safe_goto_retry.cpp` | 同上，但触发 reflex 时自动 `automaticErrorRecovery()` 并从当前实际位置重新规划，最多重试 N 次。系统负载高时用它更稳。 |
| `safe_sequence.cpp` | 多姿态序列：一次走完 `home pos2 pos3 ...`，段间停顿、末段保持。 |
| `safe_nudge.cpp` | 单关节微动（默认 3°），最小风险的"能动就行"测试。 |
| `safe_wrist.cpp` | 腕部 q5/q6/q7 三轴同时小幅转动（幅度、速度可调）。 |

### 柔顺与碰撞实验

| 文件 | 作用 |
|---|---|
| `safe_soft_move.cpp` | **低刚度阻抗模式**运动：刚度可传参（150~800），被推时"让开"而不是硬顶。 |
| `safe_move_nostop.cpp` | 位置控制 + 碰撞阈值放开到 1000，用于测"推撞不触发 reflex"。 |
| `safe_compliant.cpp` | 阈值预设（high/normal/off）+ 力测量：保持位置 N 秒，1 kHz 记录力/力矩到 CSV，给出峰值统计。 |
| `gripper_force_probe.cpp` | 夹持力扫掠：5 N → 70 N 逐步改变设定力，找 `is_grasped` 翻转阈值。 |

### 夹爪与任务

| 文件 | 作用 |
|---|---|
| `safe_gripper.cpp` | 夹爪工具：`status` / `open` / `half` / `grasp <宽> <力>` / `home`。夹持成功用**宽度判据**（实际宽度 > 命令 + 5 mm），因为本机 `is_grasped` 不可靠。 |
| `safe_cycle.cpp` | 完整抓取循环：回 home → 到目标 → 闭合夹爪（含 1 s 稳定观察与滑动重夹）→ 回 home。 |

### 参数与备份

| 文件 | 作用 |
|---|---|
| `esn_yield_params.md` | 当前认可版本的**全部参数**：刚度、门控阈值、让位量程/速率、零偏校准、方向无关性说明。 |
| `esn_yield_good_2026-11-21.cpp.bak` | 认可版本的源码快照，改动前可回滚。 |

---

## 四、`FR3_ESN128_main/` —— 模型包

| 文件 | 作用 |
|---|---|
| `fr3_esn128/esn_actor.hpp` | **策略本体**。`Esn128::step(obs)` 实现储备池递推 + 线性读出；`ActionFilter` 把归一化动作换成米/弧度并限速。 |
| `fr3_esn128/model_generated.inc` | **冻结权重**（617 KB）：128 单元输入矩阵、循环矩阵、泄漏率、552×7 读出层。 |
| `fr3_contact/active_vmc.hpp` | **VMC teacher**：六维虚拟质量-弹簧-阻尼模型，外力→位移。是 ESN 模仿的对象，也作为对照基准。 |
| `fr3_contact/profile_generated.inc` | VMC 参数表（刚度、阻尼、最大让位 0.18 m / 0.35 rad 等）。 |
| `main.cpp` | 官方**离线推理示例**：读一列 48 维观测，逐帧打印 7 个动作，不需要机器人。 |
| `example_obs48.txt` | 示例输入（9 帧，模拟 8 N 外力出现再撤除）。 |

---

## 五、`rt-setup/` —— 实时环境（一次性配置）

| 文件 | 作用 |
|---|---|
| `apply.sh` | 建 `realtime` 组、装 `rtprio 99 / memlock unlimited`、往 GRUB 加 `isolcpus`、装编译依赖。 |
| `build-rt-kernel.sh` | 下载 6.8.x-rt 源码打补丁并编成 deb（30~60 分钟）。 |
| `tune-latency.sh` | 调优：performance governor + 禁用 C2/C3 深睡 + 中断避开隔离核（运行时/持久化两种模式）。 |
| `tune-nic.sh` | 网卡调优：关 offload、调 ring、网络栈参数，并自动测 500 包延迟。 |
| `verify.sh` | 验收：RT 内核、隔离核、rtprio、cyclictest 抖动。 |
| `build-libfranka.sh` | 编译 pinocchio 3.4.0 + libfranka 0.21.3（jammy 源里没有，必须源码编）。 |
| `99-realtime.conf` / `rollback.sh` | limits 配置 / 一键回滚。 |

---

## 六、其他

| 路径 | 作用 |
|---|---|
| `franka-poses/home.json`、`pos2.json` | 两个示教姿态（q + O_T_EE），程序按名字读取 |
| `scripts/esn_roundtrip.sh` | 往返运行：A→B（ESN）→ 停 N 秒 → B→A（ESN），任一段失败即中止 |
| `docs/` | 操作手册 PDF、环境复现包、力曲线对比图 |
| `std-logs/` | 典型运行日志（含 ESN 开/关对照） |

---

## 七、代码里的关键设计（为什么这么写）

1. **余弦轨迹**：`q = q0 + Δ·(1−cos(πt/T))/2`，起止速度和加速度都连续。用恒定速度或瞬时启停会让 libfranka 报 `velocity/acceleration_discontinuity` 并急停。
2. **二阶平滑**：让位量先限速度（0.15 rad/s）再限加速度（1.0 rad/s²），防止策略输出的阶跃把指令打飞。
3. **接触门控**：`|F| > 12 N` 才启用策略，`< 9 N` 持续 0.3 s 退出。门控关闭时指令等于纯轨迹 —— 这是"平时丝滑"的来源。
4. **雅可比阻尼最小二乘**：`dq = Jᵀ(JJᵀ+λ²I)⁻¹·y`，λ=0.05，避免伪逆在奇异位形附近放大成大幅关节跳变。
5. **外力零偏校准**：启动静置 0.5 s 取 400 帧中位数，再按末端姿态旋转补偿；否则夹爪自重残差会被当成接触，导致无接触也持续让位。
6. **三层频率解耦**：策略 25 Hz、执行 1 kHz、日志 1 kHz —— 推理不需要跑进 1 ms，执行又必须。

---

## 八、已知局限（如实记录）

- **让位方向与外力方向的相关性偏低**（余弦 +0.33；同批观测下 VMC teacher 为 +0.95）。推测训练时的观测定义（速度/误差通道的坐标与缩放）与当前组装不一致，需要训练侧代码才能彻底对齐。
- **Franka Hand 遥测异常**：`max_width` 恒为 0.0523、`temperature` 坏值、`is_grasped` 不可靠，因此全部改用宽度判据。
- **网卡为 Realtek RTL8111**：偶发延迟尖峰（最坏 3.9 ms），高负载下会丢控制周期，建议换 Intel I210。
- **虚拟机不可用**：FCI 需要 1 kHz 硬实时，本套件仅在物理机上验证。
