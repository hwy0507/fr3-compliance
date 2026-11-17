# FR3 Compliance

Franka Research 3 的FCI原生阻抗/有限随动准备、示教与位置重放、通信/受力诊断，以及既有ESN64/16柔顺控制候选。
**已做过位置重放和原生阻抗保持，免按钮自由拖动尚未验证通过，沿轨迹VMC/ESN接触实验尚未完成。**

## 今天下午的完整交接

- **[2026-11-17下午工作与FCI柔顺控制总结](docs/FR3_FCI_AFTERNOON_SUMMARY_20261117.md)**：目标、实际操作、各次失败、当前参数、代码入口和后续待办。
- [真实实验精简摘要](docs/evidence/fr3_afternoon_20261117/REAL_EXPERIMENT_SUMMARY.json)：公开的数值摘要与源文件hash；原始零位/轨迹、凭据和全true真实授权文件不在仓库。
- [原生控制的清单、刚度保留与0.5/1Nm候选](docs/FR3_NATIVE_RETAIN_TRIAL_20261117.md)。1Nm是相对变化候选；最新真机启动在静态本底检查拒绝，未进入控制。
- [新增源码逐文件导入清单](AFTERNOON_IMPORT_MANIFEST_20261117.json)：冻结自SII_compliance b36ea725；保留既有ESN导入及其原清单。

## 整体离线检查（不连接机器人）

依赖C++17、Eigen3、libfranka 0.21.3、Python≥3.10、NumPy和SciPy。本轮Python依赖版本记录在 `requirements-offline.txt`。请使用同时装有NumPy/SciPy的解释器；可先运行 `python -c "import numpy, scipy"` 检查。输出目录必须不存在：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python tools/check_package_offline.py \
  --output /tmp/fr3-afternoon-offline-new-run
```

覆盖原生核心/受控启动、有限随动、EEE mock回归、逐帧日志、位置曲线/导出解析、VMC C++合成核心与本地include闭合。
[本仓库实际检查结果](docs/evidence/fr3_afternoon_20261117/PACKAGE_OFFLINE_CHECKS.json)。本轮不重跑需要外部原始包的ESN一致性或需要完整研究源码的VMC Python一致性；历史证据保留其适用SHA。

原生真实控制入口默认构建关闭；位置执行器另有显式已审阅单段入口，不默认启动，也不自动串联回零/重放。
新候选参数和离线检查不能代替现场授权、工具/受力/保护来源核实与物理验收。

## 既有ESN64/16导入

下文保留原ESN导入入口；目前仍是**通过离线检查的集成候选**，尚未通过真机接触测试或完整sim2real验收。

### ESN入口

- [真机适配说明与上机前待办](docs/FR3_ESN_SIM2REAL_PREPARATION_20261117.md)
- [代码结构和离线检查方法](tools/fr3_esn/README.md)
- [离线数值检查](docs/evidence/fr3_esn_port_20261117/OFFLINE_CHECKS.json)
- [ASan/UBSan 检查](docs/evidence/fr3_esn_port_20261117/SANITIZER_CHECKS.json)
- [导入来源与逐文件 SHA256](IMPORT_MANIFEST.json)

### ESN当前实现

冻结 seed45 ESN（64维储层、16条训练轨迹），25Hz 有状态推理和动作滤波；
1kHz FR3 接口候选、独立保护检查、有界速度/力矩、接触超时、故障锁存和预分配日志。
必要的共用头文件位于 `tools/fr3_contact/`，该目录同时保留原工具的历史测试脚本。
本仓库的ESN适配检查入口为 **`tools/fr3_esn/check_offline.py`**；旧 VMC 检查脚本依赖原研究仓库，
不作为本仓库的独立检查入口。

没有一键运动 CLI。会话审阅默认未通过，观测器校准默认未确认；不能通过改标志跳过验证。
原仿真 paper_mpc、场景避障约束、夹爪流程和完整示教轨迹尚未作为真实任务接入。

### ESN完整一致性复现

依赖 C++17、Eigen3、libfranka 0.21.3、Python 3.10或更高版本及 NumPy。
需要原始部署包及其解压目录；大型部署包和原始采样没有发布到此仓库。
将以下路径换成本机路径，并使用不存在的新输出目录：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python tools/fr3_esn/check_offline.py \
  /path/to/fr3_esn_w64_n16_deploy_20261117 \
  --archive /path/to/FR3_ESN64_16_部署准备包_20261117.tar.gz \
  --output /tmp/fr3_esn_offline_new_run
```

该命令校验输入、编译和运行离线测试，不连接机器人。加 `--sanitize` 可检查 ASan/UBSan；
历史环境不支持 LSan，现有证据没有声称通过 LSan。

上机前必须解决已记录的无接触保护退出、负载/TCP依据、观测器方向/摩擦/噪声与延迟、
有效保护阈值以及实际实时性和停止行为。具体测试必须另外取得现场授权。

### ESN原始来源与状态

由用户指定的 `SII_compliance` 本地提交
`08a9a1fb87a539966de27c14df27c23a8c58fe64` 独立导入。
21个来源文件逐字保留，导入没有修改权重、控制器、保护阈值或检查结果。
历史文档中的本机路径和原仓库信息是原验证环境记录，不代表当前机器上的可用路径。

原数值检查与 ASan/UBSan 各通过30,574项断言，4,160帧ESN输出与原Python最大差约1.2e-14。
这些是代码一致性证据，不是真机安全认证或任务成功率。
