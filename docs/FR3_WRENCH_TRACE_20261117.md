# 返回执行器的逐帧受力与故障尾部日志

`tools/fr3_taught_path.cpp` 新增诊断日志，保持原运动计划、模式、控制器、限速和保护检查。
没有新增参数 setter、清错、自动重试或连续动作。真机原始数据仅留各自本地 run 目录；
本文说明日志契约，不包含真实轨迹、关节值或负载配置。

## 为什么需要补充

旧 `motion.csv` 只有 t/dt、实测 q/dq、原始位置指令和 SDK 期望位置；
`diagnostic_states.txt` 仅保留预检、首回调和末回调的完整状态。
因此只能知道少数时刻的估计外力，不能重建受力增长过程。
此外，SDK 可能在收到 reflex 状态后直接抛出 `ControlException`，不再将该帧送给用户回调；
旧代码只保存 `what()`，丢掉了异常附带的最后状态/指令记录。
**历史缺失的逐帧受力无法补录，不插值或伪造为真实测量。**

## 新增输出

| 文件 | 内容 |
|---|---|
| `wrench.csv` | 每次回调收到的原始状态，146列，检查之前就捕捉，包含被应用检查拒绝的回调 |
| `wrench_trace_info.txt` | schema、行数、容量/溢出、分配字节数、单位、时间及参考点说明 |
| `sdk_exception_wrench.csv` | `ControlException.log` 中的状态，使用相同列定义；仅在 SDK 控制异常时生成 |
| `sdk_exception_states.txt` | 异常尾部的完整原始 RobotState，保留具体错误名称和完整配置字段 |
| `sdk_exception_commands.csv` | SDK 记录中的上一周期限速后位置指令；用于当前关节位置运动模式 |
| `sdk_exception.txt` | 原始异常文字、记录数量和状态/指令对齐规则 |

原 `motion.csv`、`status.txt`、`diagnostic_states.txt` 格式保持，旧解析器继续适用。
`--offline` 和 `--read-only` 不启动运动，会生成只有表头的 `wrench.csv` 和零行元数据，
不能把它当作已采集了真实运动受力。

## 逐帧字段与语义

- 绝对 `robot_time_s`、SDK 回调 `dt`、通信成功率、模式、当前及最近错误汇总；
  另记当前/最近的 cartesian_reflex、joint_reflex 和 communication_constraints_violation。
- q/dq、q_d/dq_d/ddq_d、tau_J/tau_J_d/dtau_J、tau_ext_hat_filtered。
- 原始 `O_F_ext_hat_K`、`K_F_ext_hat_K`，joint/cartesian contact 与 collision 标记。
- **逐帧**列主序 `O_T_EE` 与 `EE_T_K`，便于离线核对参考点，避免只用邻近会话配置推断。

数组列使用 `_0` 开始的索引。单位为 rad、s、Nm、N；没有自动去偏、裁剪或平滑这些日志。
O/K 原始力矩不能直接混用；离线转换按明确的 K→O 旋转和 K→EE 平移执行。
模式编码：0 Other、1 Idle、2 Move、3 Guiding、4 Reflex、5 UserStopped、6 AutomaticErrorRecovery。

回调行表示“收到的状态”，不表示相应原始指令一定已通过检查或发送。
SDK `Record` 则配对 **command(n) 与 state(n+1)**；其位置指令已经过 SDK 限速，
不能与同一时刻的原始应用指令直接视为同一个量。SDK 尾部第一行 `dt=0` 是缺前帧的占位，
不是控制会话首回调。两种日志按机器人绝对时间关联，丢包不补帧、不等距化。

## 时序和失败处理

`fr3_wrench_trace.hpp` 使用固定字段和有界缓冲：执行模式在控制开始前分配并触页 92,000 帧；
回调内仅复制数字，不写文件、格式化字符串、构造模型或增加网络调用。
SDK 自身的尾部容量仍为 Robot 构造器默认的50条，没有扩大或改动其控制配置。
容量耗尽则中止，保留 `overflow=1`；不会环形覆盖早期记录。

文件在控制退出后写入。SDK异常原文与执行失败状态保留；尾部日志写失败另记且返回非零，
不把失败写成成功，不清错、不再次调用控制。硬断电/进程被强杀/磁盘故障仍可能造成日志缺失；
有界内存日志不是掉电持久化方案。增加内存复制的真实耗时和通信裕量仍须审查，不能凭编译
或单次轨迹完成认定已通过柔顺控制的实时条件。

## 排查“无人触碰但估计外力大”的顺序

1. 区分人手接触与其他外载：绷紧的线缆、软管或支架受阻也会产生真实拉力/力矩。
2. 对照同姿态停稳前后，检查外力及关节模型残差是否持续；不能只根据峰值给出新负载。
3. 检查受力变化是否随姿态、速度、加速度、跟踪误差、通信间隔或接触标记变化。
4. 检查 Jacobian 条件和测力参考点；数值换算一致不等于独立测力验证。
5. 查实际有效碰撞阈值及来源；截图、历史任务配置、SDK示例均不直接证明当前值。

单次受阻/松线前后比较可能同时改变起点、速度和路径，不是隔离变量的标定实验。
在缺少逐帧受力、有效阈值和独立测量时，保留多个原因，不宣称唯一归因或负载标定成功。

## 离线验证

`tools/test_fr3_wrench_trace_offline.cpp` 只构造合成 RobotState/ControlException，
覆盖原始受力、NaN、错误/Reflex状态、变换、容量耗尽、CSV列对应、SDK独有故障尾部与写入失败。
原返回数学/首帧检查和四项计划导出/解析回归同时保留。
证据见 `docs/evidence/fr3_wrench_trace_offline_20261117/CHECKS.json`；不含真实机器人数据。

官方类型依据：
- [ControlException 与异常日志](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1ControlException.html)
- [SDK Record 对齐语义](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1Record.html)
- [RobotState](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html)
