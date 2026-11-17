# FCI 通信诊断工具与 EEE 读回修复

本工具用于区分网卡链路、主机调度和客户端计算成本，不提供机器人运动入口。原生柔顺控制走 FCI 时，内部阻抗执行与外部参考指令通信是两层；只读接收合格不能替代双向控制验收。

## 只读探针

`tools/fr3_comm/read_only_probe.cpp` 以 `RealtimeConfig::kEnforce` 建立连接，仅使用 `read/readOnce`、`loadModel`、本地模型和速度边界计算。最多接收5000帧，回调要求 Idle、无当前错误且关节速度模长各分量不超过0.02rad/s；遇到变化退出读取，不发停止/恢复命令。调用者应另设20秒进程超时，通信超时由SDK报告。

探针使用预分配缓冲，只在退出接收后写文件；以 `mlockall(MCL_CURRENT)` 锁当前进程已有内存，可绑定本线程CPU，未更改其他进程或全局CPU/IRQ配置。输出机器人时间、主机单调时钟、模型与状态转换成本、执行CPU及实际调度策略/优先级。调度与锁内存设置随进程退出释放。读取过程中没有完整控制状态机、SDK发送和指令往返，因此转换成本不是控制回调总成本或最坏时延证明。

编译与只读运行（先审阅源码，并确认没有其他FCI控制客户端）：

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Werror -isystem /usr/include/eigen3 \
  tools/fr3_comm/read_only_probe.cpp -o /tmp/fr3_read_only_probe -lfranka
timeout 20s /tmp/fr3_read_only_probe --read-only ROBOT_IP NEW_DIRECTORY CPU
```

CPU为有效CPU编号，`-1`表示不绑定。目录必须不存在。接收时钟间隙同时包含网络、调度及客户端读取延后，不能仅凭间隙定位物理网线；Idle中的 `control_command_success_rate` 也不能用于控制丢包验收。

## 主机 EEE 工具

`tools/fr3_comm/host_link.py` 仅作用于源码中固定接口与PCI身份匹配的主机网卡，不连接机器人。`--show` 为读取；`--disable NEW_BACKUP_JSON` 和 `--restore BACKUP_JSON` 需要管理员权限。修改可能短暂中断链路，必须在所有FCI控制与Desk任务停止后实施。此工具不是通用网卡扫描器，不自动应用任何设置。

禁用前以独占方式保存并fsync原始配置，已有备份拒绝覆盖。恢复时核对接口、PCI与MAC身份，恢复原EEE开关、TX LPI、广告掩码和计时器。

读回修复：某些驱动在禁用EEE后返回广告掩码为0，即使请求沿用了原掩码。现在禁用时只接受0或原请求掩码，且严格要求EEE关闭、未激活、TX LPI和计时器符合请求；重新开启/恢复时仍须精确匹配原请求广告掩码。不忽略真正的状态不匹配，也不把协商尚未激活当作开启失败。

如果旧版本在写入后报 `advertised` 不匹配，设置可能已经生效。先 `--show` 读取并核对原备份，不删除备份、不再次禁用以覆盖原值。失败不触发自动恢复或重复写入。

## 验证与边界

8项离线回归覆盖驱动禁用时清零/保留掩码、恢复、错误开关/激活/掩码/计时器拒绝、设置载荷和备份不可覆盖。执行：

```bash
python -m unittest discover -s tools/fr3_comm -p 'test_*.py' -v
```

证据：`docs/evidence/fr3_comm_tools_offline_20261117/CHECKS.json`。严格编译并检查动态导入，探针没有机器人control、setter、recovery或夹爪接口导入。合成检查不修改真实网卡。实际链路测量、机器身份及用户备份仅本地保留，未随本提交外发。

仍需在另行授权的控制测试中核实真实命令成功率、总回调耗时、周期、缺页及停止行为。不会运行官方 `communication_test` 来冒充无运动测试，必须先审阅所安装版本源码及其可能的运动/设置行为。不为使测试通过而放宽当前原生轻推候选保护，原刚度/恢复值、有效碰撞配置和现场条件仍需独立确认。

官方依据：[FCI通信排查](https://frankarobotics.github.io/docs/troubleshooting.html)、[libfranka 0.21.3线程优先级实现](https://github.com/frankarobotics/libfranka/blob/0.21.3/src/control_tools.cpp)。

本地源码位于当前checkout的 `tools/fr3_comm/`。本轮归档服务器不可达，未完成服务器镜像，不提供未经核验的远端产物地址。
