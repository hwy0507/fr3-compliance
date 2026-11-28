# 2026-11-28：ESN 语义修正（位移 → 速度残差）与一键回 home

## 一、今天做了什么

1. 按"ESN 训练时的输出就是**速度残差**"这一修正，把 ESN 的 `a1..a6` 从"末端位移"改解释为"末端速度"，
   **另写**两个程序实现，原始 `esn_yield.cpp` 一行未改。
2. 新增 `esn_velocity.cpp`：**纯 P 速度伺服**（无位置反馈）+ ESN，用于回答
   "把底层从阻抗换成速度环之后，ESN 还起不起作用"。
3. 新增 `scripts/go_home.sh`：任意状态一键回 home。
4. 真机做了 4 次点对点移动（pos4→home、home→pos2、pos2→pos3、pos3→pos4），
   记录到一个稳定复现的现象：**reflex 几乎只发生在运动第一帧**。

代码与参数归档在 [code11.28/](../code11.28/README.md)。

---

## 二、为什么要改语义

之前的实现（`esn_yield.cpp`，效果不错、保持不变）把 ESN 的输出当**末端位移偏移**用：

```cpp
y = 0.32 * a[1..3]  (m)     0.60 * a[4..6]  (rad)
q_cmd = q_ref + J^+ y        // 直接叠加到关节目标
```

由此产生两个说不通的地方：

| 现象 | 位移解释下 | 速度残差解释下 |
|---|---|---|
| 同批观测里让位方向与外力余弦只有 **+0.33**（VMC teacher 为 +0.95） | 应该是"往力推的方向让"，对不上 | 输出是速度，积分后相位/幅值都会变，方向关系本来就不该按位移比 |
| 推力越大让位越"饱和"（90→306 mm 后不再增长） | 位移量程 `Y_MAX=0.18` 卡住了 | 速度残差被 `VEL_LIM`/`ACC_LIM` 限幅，本来就不该累积到位移量程 |

也就是说：**模型本身可能没错，是部署侧把"速度"当"位移"用了。**
本次改动只动部署侧的解释，不动训练侧权重（`model_generated.inc`）。

---

## 三、三个控制器的结构性区别

```
esn_yield.cpp        关节阻抗  q_cmd  = q_ref + ∫v_yield dt                a1..a6 = 位移(m/rad)
esn_yield_vres.cpp   关节阻抗  q_cmd  = q_ref + clamp(∫v_res dt, ±0.35)    a1..a6 = 速度(m/s, rad/s)
esn_velocity.cpp     速度环    dq_cmd = dq_ff + dq_res + KP(dq_ff − dq_meas)
```

### 1. `esn_yield_vres.cpp`（150 刚度阻抗 + 速度残差）

相对原文件只有 3 处改动（完整 diff 见 [code11.28/esn_yield__to__esn_yield_vres.diff](../code11.28/esn_yield__to__esn_yield_vres.diff)）：

```cpp
// 25 Hz：输出当速度
y << V_SIGN*KV_LIN*a[1], V_SIGN*KV_LIN*a[2], V_SIGN*KV_LIN*a[3],
     V_SIGN*KV_ROT*a[4], V_SIGN*KV_ROT*a[5], V_SIGN*KV_ROT*a[6];   // m/s, rad/s
dq = J^T (J J^T + λ²I)^{-1} y;                                      // rad/s
dq_target = clamp(dq, ±VEL_LIM);                                    // 默认 0.3

// 1 kHz：加速度限幅后积分成位置偏移
dq_vel  += clamp(dq_target − dq_vel, ±ACC_LIM*dt);                  // 默认 2.0
dq_prev = clamp(dq_prev + dq_vel*dt, ±OFFSET_MAX);                  // 默认 0.35 rad
q_cmd   = q_ref + dq_prev;
```

新增/变更环境变量：

| 变量 | 默认 | 含义 |
|---|---|---|
| `KV_LIN` | 0.32 | 平移通道 a→m/s 的比例（沿用原 ActionFilter 数值）|
| `KV_ROT` | 0.60 | 转动通道 a→rad/s |
| `V_SIGN` | **−1** | 整体符号；若实测让位方向反了，设 `V_SIGN=1` |
| `VEL_LIM` | 0.3 | 关节速度残差上限 rad/s |
| `ACC_LIM` | 2.0 | 残差加速度上限 rad/s² |
| `OFFSET_MAX` | 0.35 | 积分得到的位置偏移上限 rad（安全兜底）|

> **`V_SIGN` 是这次上机最先要确认的参数。** 原实现里为了让位方向正确，写死了取负号
> （注释记录实测 cos=−0.70）；改成速度语义后这个负号是否仍成立，没有理论依据，必须实测。

### 2. `esn_velocity.cpp`（纯 P 速度伺服 + 速度残差）

```cpp
dq_cmd[i] = dq_ff[i] + dq_res[i] + KP * (dq_ff[i] − rs.dq[i]);
return franka::JointVelocities(dq_cmd);
```

| 变量 | 默认 | 说明 |
|---|---|---|
| `KP` | 10 | 速度误差 → 速度修正 |
| `VMAX` | 0.4 | 速度指令上限 rad/s |
| `VACC` | 0.5 | 速度指令变化率上限 rad/s²（**必须小**，5.0 会直接 `acceleration_discontinuity`）|
| `KV_LIN`/`KV_ROT` | 0.32 / 0.60 | 同 `_vres` |
| `ESN_OFF` | — | 设 `1` 关闭策略（对照组）|

关键的物理性质，也是这个实验的意义所在：**速度环没有位置反馈，被推偏之后不会自己回到轨迹**。
一旦加入位置项或 D 项，它就变回阻抗控制，"速度伺服 + ESN"和"阻抗 + 调刚度"就没有区别了
（这一点此前被指出过，是本次设计的核心约束）。

---

## 四、四个部署命令

```bash
# (1) 150 刚度阻抗 + ESN（速度残差语义）
STIFFNESS=150 ~/franka-fci-suite/franka-safe/build/esn_yield_vres 172.16.0.2 home pos2 0.15 high

# (2) 150 刚度阻抗，无 ESN
ESN_OFF=1 STIFFNESS=150 ~/franka-fci-suite/franka-safe/build/esn_yield_vres 172.16.0.2 home pos2 0.15 high

# (3) 纯 P 速度伺服 + ESN
~/franka-fci-suite/franka-safe/build/esn_velocity 172.16.0.2 home pos2 0.15 high

# (4) 纯 P 速度伺服，无 ESN
ESN_OFF=1 ~/franka-fci-suite/franka-safe/build/esn_velocity 172.16.0.2 home pos2 0.15 high
```

建议都套实时优先级：`chrt -f 80 taskset -c 14 <二进制> ...`。

多点链路（本次对应用户的 4 段往返需求）：

```bash
# 开 ESN，位移语义（原始版本，未经修改）
STIFFNESS=150 ~/franka-fci-suite/scripts/esn_chain.sh 172.16.0.2 home pos2 pos3 pos4 home 0.15

# 关 ESN 的对照
ESN_OFF=1 STIFFNESS=150 ~/franka-fci-suite/scripts/esn_chain.sh 172.16.0.2 home pos2 pos3 pos4 home 0.15

# 换底层程序（脚本新增 ESN_BIN 支持）
ESN_BIN=esn_yield_vres STIFFNESS=150 ~/franka-fci-suite/scripts/esn_chain.sh 172.16.0.2 home pos2 pos3 pos4 home 0.15
ESN_BIN=esn_velocity  ~/franka-fci-suite/scripts/esn_chain.sh 172.16.0.2 home pos2 pos3 pos4 home 0.15
```

`esn_chain.sh` 的行为：每段独立调一次底层程序 → 先位置控制走到本段起点 → 重做 0.5 s 外力零偏校准
→ 再用 ESN 走完终点；任一段失败立即中止，不静默继续。

---

## 五、真机观察：reflex 集中在运动第一帧

今天做了 4 次点对点移动，全部用 `safe_goto_retry`（`0.12 rad/s`，容差 0.01 rad，最多 8 次尝试）：

| 段 | 关节行程 | 第 1 次尝试 | 恢复后 | 最终误差 |
|---|---|---|---|---|
| pos3 → pos4 | 89.2° | reflex（velocity + acceleration） | 第 2 次走完 7.4° | **0.17 cm** / 0.4° |
| home → pos2 | 72.4° | reflex（acceleration） | 第 2 次走完 1.3° | 1.3 cm / 0.9° |
| pos2 → pos3 | 54.3° | reflex（velocity + acceleration） | 第 2 次走完 19.1° | 1.1 cm / 1.0° |
| pos4 → home | 52.5° | **无 reflex，一次走完** | — | 0.8 cm / 0.7° |

分析：

- 恢复机制本身可靠——4 次里有 3 次中断，**每次 `automaticErrorRecovery()` 后重规划都能走完**，
  最终精度 0.2~1.3 cm。
- `control_command_success_rate` 在中断时是 0.95~0.99，说明是单帧抖动，不是持续丢包。
- 主机负载当时已降到 load 0.71（此前一次失败定位到 Firefox `Isolated Web Co` 占 50% CPU），
  所以这次不是负载问题。
- 更可能的原因在**轨迹本身**：余弦位置轨迹在 t=0 时速度为零但**加速度最大**
  （`a_max = Δπ²/(2T²)`），从静止直接阶跃到 a_max。行程越大 a_max 越大——89°、72°、54° 都断了，
  52.5° 那次没断，阈值就在这附近。现有的"第一帧播种"只保证位置连续，没有处理加速度连续性。
- 待办：给 `safe_goto_retry` 的轨迹换成 S 曲线（余弦加速度），代价是每次起步慢约 0.3 s。

---

## 六、新增：任意状态一键回 home

```bash
~/franka-fci-suite/scripts/go_home.sh            # 默认 172.16.0.2 / 0.12 rad/s
~/franka-fci-suite/scripts/go_home.sh 172.16.0.2 0.10
```

流程：探 1337 端口 → `safe_goto_retry`（从**当前实际位置**出发，遇 reflex 自动恢复并重规划，
最多 8 次）→ 外层再重试 2 轮 → 用 `safe_fk` 量末端到 home 的直线距离，< 1 cm 判定成功
→ 失败时按日志里的错误类型直接给出 Desk 侧处置步骤（并打印日志路径 `/tmp/go_home_*.log`）。

**不能自动处理的情况**（必须人工）：

| 状态 | 处理 |
|---|---|
| `User stopped` | Desk 点 Resume / ▶；或 FCI OFF → 等 3 s → ON |
| reflex 反复锁存、`automaticErrorRecovery` 失败 | Desk 点 Reset |
| 急停按钮按下 | 先松开急停 |
| 关节 Locked（brakes） | Desk → Joints → Unlocked |

这四类属于机器人侧安全状态，FCI 客户端无权自行解除；脚本的定位是"能自动的都自动，
不能自动的立刻说清按哪里"。

---

## 七、未完成 / 下一步

1. **`esn_yield_vres` 与 `esn_velocity` 均未上真机**，目前只有编译通过。
   首次上机顺序建议：
   - `_vres` + `ESN_OFF=1`（先确认速度和加速度限幅不触发 discontinuity，且能到达目标）
   - `_vres` + ESN 开，**手指轻推确认让位方向**，方向反了就 `V_SIGN=1`
   - `esn_velocity` + `ESN_OFF=1`（确认纯速度环能走完、能停在目标附近）
   - `esn_velocity` + ESN 开
2. `V_SIGN` 的最优值需要实测确认。
3. `esn_velocity` 没有位置反馈，终点会有残余偏差（速度环只能保证速度跟踪），
   需要量化"每段停在哪里"的偏差量级。
4. `safe_goto_retry` 的 S 曲线改造（见第五节）。
5. 相机轨迹录制（11-26 遗留）：1 kHz `O_T_EE` 记录器 + 叠加渲染器仍未写；相机摆位未定。
6. 仓库里 11-26 的日志已推送（`786dc3f`）。
