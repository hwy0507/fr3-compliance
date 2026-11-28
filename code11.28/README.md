# code11.28 — 速度残差语义的三个控制器 + 一键回 home

本目录是 2026-11-28 的全部代码变更。**原始 `esn_yield.cpp` 未做任何修改**
（仍与 11-22 归档一致，`diff` 为空），新语义全部放在独立文件里，便于随时回退。

| 文件 | 作用 | 状态 |
|---|---|---|
| `esn_yield_vres.cpp` | 关节阻抗 + ESN(让位 = 速度残差) | 编译通过，**未上真机** |
| `esn_velocity.cpp` | 纯 P 速度伺服 + ESN(速度残差) | 编译通过，**未上真机** |
| `esn_yield__to__esn_yield_vres.diff` | 上面两个"位移版 → 速度残差版"的完整差异 | — |
| `esn_chain.sh` | 多姿态链路跑批，新增 `ESN_BIN` 切换底层 | 已用于 11-26/11-28 实验 |
| `go_home.sh` | 任意状态一键回 home（自动恢复 reflex，失败给 Desk 处置提示） | 已手工验证同等的 `safe_goto_retry` 路径 |
| `safe_goto_retry.cpp` | `go_home.sh` 依赖的底层移动程序（带 `automaticErrorRecovery`） | 已在真机多次验证 |

## 三个控制器的区别

```
esn_yield.cpp        位置环阻抗      q_cmd = q_ref + ∫v_yield dt        a1..a6 = 位移偏移(m/rad)
esn_yield_vres.cpp   位置环阻抗      q_cmd = q_ref + clamp(∫v_res dt)   a1..a6 = 速度残差(m/s, rad/s)
esn_velocity.cpp     速度环(无位置反馈) dq_cmd = dq_ff + dq_res + KP(dq_ff − dq_meas)
```

- 前两者底层都是 FR3 关节阻抗（`kJointImpedance`），只是 ESN 输出的物理解释不同：
  `esn_yield` 把 `a1..a6` 乘 0.32 m / 0.60 rad 直接当位移叠加；`esn_yield_vres` 把它当速度，
  经 DLS 转关节速度、加速度限幅（`ACC_LIM`）、再积分成位置偏移（`OFFSET_MAX`）。
- 第三个换成 `franka::JointVelocities`，**没有位置反馈**：被推偏后不会自己回到原轨迹，
  这正是"换成 P 速度伺服后 ESN 是否还起作用"这个实验要问的东西。
  > 注意：如果给它加 D 项或位置项，就退化回阻抗控制，实验就失去意义了。

## 四个部署命令

```bash
# (1) 150 刚度阻抗 + ESN（速度残差语义）
STIFFNESS=150 ~/franka-fci-suite/franka-safe/build/esn_yield_vres 172.16.0.2 home pos2 0.15 high

# (2) 150 刚度阻抗，无 ESN
ESN_OFF=1 STIFFNESS=150 ~/franka-fci-suite/franka-safe/build/esn_yield_vres 172.16.0.2 home pos2 0.15 high

# (3) 纯 P 速度伺服 + ESN（速度残差）
~/franka-fci-suite/franka-safe/build/esn_velocity 172.16.0.2 home pos2 0.15 high

# (4) 纯 P 速度伺服，无 ESN
ESN_OFF=1 ~/franka-fci-suite/franka-safe/build/esn_velocity 172.16.0.2 home pos2 0.15 high

# 对照：原始位移语义（未修改的版本）
STIFFNESS=150 ~/franka-fci-suite/franka-safe/build/esn_yield 172.16.0.2 home pos2 0.15 high
```

差速运行建议加实时优先级：

```bash
chrt -f 80 taskset -c 14 <上面的二进制> ...
```

## SHA256

```
34de0e10e8a334e953de600e486e3c02784c99ba880f0f919cefddf3d390a9b1  esn_velocity.cpp
c083d5b0d99c2ce5062649c88e57d5c286a3f6356b2f02bf4a0a8c09a3901fdb  esn_yield_vres.cpp
b9aedbe73b317af08e63f734bdd89b0b79c613f51eba61da0e80bb1e9ae7936c  safe_goto_retry.cpp
240dac449aec5c1fd2766ddf1536ba33580fd2720cba0412a956f34c3a914226  esn_chain.sh
9f4d1b3fbfdb9522ce12ceef62007f15aa8f5fbf0f0dcc6cbbf5b011fc02dfda  go_home.sh
```

详细说明见 [../docs/FR3_VELOCITY_RESIDUAL_20261128.md](../docs/FR3_VELOCITY_RESIDUAL_20261128.md)。
