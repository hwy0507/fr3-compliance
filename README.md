# FR3 Compliance

面向 Franka Research 3 的 ESN64/16 柔顺控制真机适配代码。
当前为**通过离线检查的集成候选**，尚未通过真机接触测试或完整 sim2real 验收。

## 从这里开始

- [真机适配说明与上机前待办](docs/FR3_ESN_SIM2REAL_PREPARATION_20261117.md)
- [代码结构和离线检查方法](tools/fr3_esn/README.md)
- [离线数值检查](docs/evidence/fr3_esn_port_20261117/OFFLINE_CHECKS.json)
- [ASan/UBSan 检查](docs/evidence/fr3_esn_port_20261117/SANITIZER_CHECKS.json)
- [导入来源与逐文件 SHA256](IMPORT_MANIFEST.json)

## 当前实现

冻结 seed45 ESN（64维储层、16条训练轨迹），25Hz 有状态推理和动作滤波；
1kHz FR3 接口候选、独立保护检查、有界速度/力矩、接触超时、故障锁存和预分配日志。
必要的共用头文件位于 `tools/fr3_contact/`，该目录同时保留原工具的历史测试脚本。
本仓库的适配检查入口为 **`tools/fr3_esn/check_offline.py`**；旧 VMC 检查脚本依赖原研究仓库，
不作为本仓库的独立检查入口。

没有一键运动 CLI。会话审阅默认未通过，观测器校准默认未确认；不能通过改标志跳过验证。
原仿真 paper_mpc、场景避障约束、夹爪流程和完整示教轨迹尚未作为真实任务接入。

## 离线复现

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

## 来源与状态

由用户指定的 `SII_compliance` 本地提交
`08a9a1fb87a539966de27c14df27c23a8c58fe64` 独立导入。
21个来源文件逐字保留，导入没有修改权重、控制器、保护阈值或检查结果。
历史文档中的本机路径和原仓库信息是原验证环境记录，不代表当前机器上的可用路径。

原数值检查与 ASan/UBSan 各通过30,574项断言，4,160帧ESN输出与原Python最大差约1.2e-14。
这些是代码一致性证据，不是真机安全认证或任务成功率。
