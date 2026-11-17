# FR3 力矩参考点修复、静止残差与碰撞阈值来源

2026-11-15。**修复和离线回归完成；定点轻推仍未放行。**
本轮仅处理已有文件和官方资料，没有连接机器人、启动控制、修改负载/相机/安全参数、
夹爪动作或自动恢复。没有用减去均值、裁剪外力或提高阈值消除失败。

## 修复与回归

旧适配器直接把 `O_F_ext_hat_K` 的力矩当作 K 点力矩。当前保存的 2,000 帧中，
`EE_T_K=I`，实际满足 `M_O = R M_K + p_O_K × (R F_K)`；
全帧力关系最大误差 1.675e-7 N，力矩关系最大误差 1.576e-7 Nm。
忽略参考点平移产生的力矩向量误差平均 0.53218、最大 0.57145 Nm。
这是本次固件/配置记录的观察，不外推成所有版本的 O 字段语义保证。

现在显式使用 `K_F_ext_hat_K`，其中力矩参考点与表达坐标均为 K：

```
R = O_R_EE * EE_R_K
f_O = -R * f_K_api
m_K_O = -R * m_K_api
r = O_R_EE * EE_p_K
m_EE_O = m_K_O + r × f_O
```

负号遵循安装版 API 的 robot-on-environment 约定；真实推力的方向仍需后续受控验证。
原始 K 点与转换后的 EE 点分别进入保护，不使用 O 原点力矩替代末端力矩。
O 字段仍检查有限性和力分量一致性（误差上限 `1e-4*(1+|F_O|)`），
力矩参考点不在运行时猜测，不发生 K 无效后静默回退到 O。
见 [官方 RobotState 定义](https://raw.githubusercontent.com/frankarobotics/libfranka/0.21.3/include/franka/robot_state.h)。

实际记录的旋转矩阵正交误差最大 1.2234e-7；原 1e-7 数值检查会误拒绝部分单精度输出。
数值容差修正为 1e-6，保留有限值/行列式/齐次结构检查，没有归一化或改变实际位姿。
合成非刚性剪切矩阵仍被拒绝；这不是放宽物理姿态、速度或力矩边界。

验证包括：

- 非零 EE/K 平移与非单位旋转、独立 O 原点→EE 路径、功率不变性、改变基座原点不改变
  EE wrench；NaN、力字段不一致、非法变换拒绝。
- 原 C++ 核心 13,845 条断言通过（含循环，不是同数量独立场景）；原 Python/C++ 主动恢复
  actor 1,800 步最大差 1.8319e-15。优化编译和 ASan/UBSan 均通过。
- 真实 2,000 帧逐帧输入同一 C++ 转换函数，与独立 NumPy 计算最大差 2.2204e-16。
  全部转换配置检查通过；把这些 wrench 放进其余字段有效的合成状态，2,000 帧均被未改的
  `Fault::Wrench` 拒绝。这个隔离检查不是完整真机预检，没有伪造未采集的 SDK 控制字段。
- 真实数据回归也通过 ASan/UBSan。LSan 因现有 ptrace 环境未启用，不声称内存泄漏检查通过。
  原部署离线契约 24 项通过；没有实机时限或动力学稳定性认证。

`regress_recorded_wrench.py` 显式验证输入 hash、完成标记、静止模式/错误/接触标记和配置匹配。
本次模型采样未逐帧保存 EE_T_K，它来自相邻 `state.npz` 元数据；这是保留的证据限制。
后续采集应逐帧或会话冻结保存该变换及工具配置，不能默认任意旧记录都有效。

## 静止残差结论

操作者已确认空夹爪、无接触、无人扶持、线缆松弛。以下是已采集窗口，非本轮实时状态：

| 量 | 均值 | 范围 | 对当前应用候选的影响 |
|---|---:|---:|---|
| 外力范数 | 1.896 N | 1.844–1.953 N | 全部超过启动 1 N |
| 修正后 EE 力矩范数 | 1.027 Nm | 1.002–1.055 Nm | 全部超过启动 0.15 Nm，也超过退出 1 Nm |
| 原 O 字段力矩范数 | 1.554 Nm | 1.503–1.621 Nm | 不能当作 EE 点力矩使用 |

外力范数标准差约 0.020 N，修正后力矩范数标准差约 0.0073 Nm；
前后半段力均值向量差 0.0115 N、力矩均值向量差 0.0023 Nm。
这是短窗口内稳定残差，不是把随机波动平均一下就归零。

独立计算 `tau_J - model.gravity - model.coriolis`，其各轴均值与
`tau_ext_hat_filtered` 均值最大差仅 0.000252 Nm，残差已存在于关节模型对比层。
瞬时两者仍有滤波/噪声差；厂家内部摩擦和观察器实现未建模，不能把该关系当独立测力标定。
单姿态下，负载/质心误差、关节偏置/摩擦等不能唯一分离；静止数据也不能识别完整惯量。
配置仍是前次读到的约 1.191 kg 总工具质量、附加 load=0，标签带 `UNVERIFIED`；
没有因读回一致而认定夹爪、相机与支架实际质量/质心已正确。

## 碰撞阈值的来源与证据边界

| 层级 | 查到的来源 | 能确认什么 |
|---|---|---|
| 本项目应用保护 | `contact_core.hpp::Limits` | 软件候选范数、速度、位移、时间边界；本轮未改 |
| Desk 任务 | 昨日只读保存的 `task_test_readonly.json`，`parameter.safety`；Move 继承父任务 safety | 任务保存值，不是当前 FCI 读回 |
| FCI 保护 | 安装版 `Robot::setCollisionBehavior` 与 RobotState 标记 | 有设置接口、没有公开数值 getter；当前有效数组未知 |
| Watchman | 上次只读 Validated Work 页面 | 无附加规则的显示，不等于 FCI 碰撞保护关闭 |

Desk `test` 保存的候选来源值为：
平移 x/y/z 各 **10 N**，旋转三轴各 **5 Nm**；关节 1–7 为
**[10, 10, 10, 8, 8, 4, 4] Nm**。没有点击 Run、重载任务或调用 setter 验证。
这个快照缺少证明其被应用、持续有效或映射到 FCI 加速/匀速上下阈值的证据，
所以不能承诺在这些值以下触碰一定不退出。

本轮扫描 `tools/` 与 `deploy/` 共 36 个源文件，并查询这些路径的 Git 历史，
未找到 `setCollisionBehavior`、`set_collision_behavior` 或 `setDefaultBehavior` 调用。
只能排除所查代码路径，不能排除 Desk、其他客户端或更早的控制柜配置。
历史慢速返回失败日志明确为 `cartesian_reflex`，说明曾发生笛卡尔保护停止；
日志未给出碰撞阈值或触发轴。不能用这条错误反算所有阈值。

[官方 Robot API](https://raw.githubusercontent.com/frankarobotics/libfranka/0.21.3/include/franka/robot.h)
区分较低接触阈值和较高碰撞阈值，并说明设置可跨控制循环/连接保留至重启。
[官方 examples_common](https://raw.githubusercontent.com/frankarobotics/libfranka/0.21.3/examples/examples_common.cpp)
中的 `setDefaultBehavior` 会主动写阈值与刚度；那是示例设置，不能拿来查询本机现值。
任务的逐轴阈值、应用的向量范数阈值及不同参考点必须分别审查；例如软件 12 N 退出值
不能凭空保证先于某个可能为 10 N 的厂家单轴阈值动作。

## 轻推前仍需完成

1. 核实整套末端工具（夹爪、相机、支架等）的实际质量和质心依据；优先现有称重/CAD/
   厂家标定记录。若需要拆装、人工改变姿态或厂家辨识流程，另行约定步骤并授权，当前不执行。
2. 找到现有 FCI 碰撞配置的可信来源与应用记录，或通过厂家支持的只读诊断确认。
   当前没有公开 getter；不能靠试撞、重启或写入“默认值”充当读取。
3. 解决并复核静止残差：必要时另行授权多姿态无接触采集，区分负载误差和偏置。
   不自动减均值掩盖持续接触，不单纯提高阈值让程序启动。
4. 形成明确接触点、施力方向、测力依据、软件/厂家边界及停止流程；先在不运行主动控制的
   独立受控检查中验证外力方向/量级，再审查控制会话。需要运动或施力均另行取得授权。
5. 定点试验的启动入口仍未提供。前项通过后再固定会话配置、实时/停止检查与现场净空，
   等用户授权执行。定点参考无路径进度，先检验基础柔顺；主动越障恢复仍需后续短段轨迹试验。

## 复现与存储

优化和 sanitizer 合成结果及真实回归小摘要：`docs/evidence/fr3_wrench_reference_fix_20261115/`。
原始数据、完整残差和阈值来源审计保留本地：
`/home/airpig/桌面/hwy/compliance_control/artifacts/fr3_contact_readonly_20261115_01/`，
新增 `WRENCH_REGRESSION.json`、`WRENCH_REGRESSION_ASAN.json`、`COLLISION_SOURCE_AUDIT.json`。
真实逐帧数据、关节配置和 Desk 原件不纳入本次 Git 提交；仅数值摘要与 hash。

复现（只读文件，无网络或机器人实例）：

```sh
/home/airpig/miniforge3/envs/sii/bin/python tools/fr3_contact/check_offline.py
/home/airpig/miniforge3/envs/sii/bin/python tools/fr3_contact/regress_recorded_wrench.py \
  --record-dir ../artifacts/fr3_contact_readonly_20261115_01 \
  --evidence /tmp/fr3_wrench_regression_new.json
```

加 `--sanitize` 可独立重跑 ASan/UBSan；证据必须写新文件，拒绝覆盖。
开工实查并 fetch/read main `1a0b11163e7b868634b37db0a35cb6f833e64a0c`、
baseline `020448203ffd5fb50f81ffc7975b9ff5a2a06048`，本任务相关接口无远端新变更。
`arm1@192.168.31.70` 本轮 SSH 仍超时，只有本地副本，服务器归档待同步；
未修改或停止其他研究作业，保留其工作树改动。
