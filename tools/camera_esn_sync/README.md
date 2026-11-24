# FR3 相机 × ESN 储备池同步交接（2026-11-24）

这是已完成真实实验的同步记录/离线融合工具包，配套 **128 维 ESN** 控制器源码与权重。先看 [交接说明](../../docs/CAMERA_ESN_SYNC_HANDOFF_20261124.md) 和 [8 秒真实视频](../../docs/evidence/camera_esn_sync_20261124/contact_highlight.mp4)。

本目录可独立构建，不依赖未提交的 `code11.24/` 或 `webui/`。启动控制会让机器人运动；构建、单元测试、`--check`、`align.py`、`render_fusion.py` 不连接机器人。原工作区演示入口未被替换。

## 本机接手：先离线检查

在仓库根目录执行，Python 启动器使用系统 Python（需要 Linux pidfd）：

```bash
/usr/bin/python3 tools/camera_esn_sync/fr3_demo/sync/build_candidate.py
/usr/bin/python3 -m unittest discover -s tools/camera_esn_sync/fr3_demo -p 'test_start_esn.py'
/usr/bin/python3 -m unittest discover -s tools/camera_esn_sync/fr3_demo/sync -p 'test_*.py'
/usr/bin/python3 tools/camera_esn_sync/fr3_demo/sync/start_synced.py --check
```

构建需要 g++、C++17、Eigen3、libfranka（本机现有环境）。构建仅编译，不运行机器人程序；生成本包 `artifacts/fr3_camera_sync_20261124/` 和本地 `fr3_demo/profile.json`，不提交生成的二进制与运行状态。

`profile.example.json` 保留本机已核实配置及 home/pos2 文件的 SHA256。姿态文件仍在 `/home/airpig/franka-poses/`，未随本次发布。缺少文件、换用户、改了姿态或二进制时，`--check` 会拒绝；不要随意替换 hash 使其通过。设备稳定 ID 固定在 `start_synced.py`；相机是本机 D435i，控制核固定 21。换机器必须重新审核这些绑定，并核对 libfranka/机器人版本、实时环境及实际安装配置。

## 明确获准后才运行真机

现场核对 EXEC + FCI、无 Desk 任务、当前接近已保存 home、空夹爪、全臂/相机/线缆净空及可停止。基线阶段撤手，收到机器人 **READY** 后再按实验约定施力。相机 READY 不等于机器人 READY。每次只执行一个 home→pos2→home 连续往返，不自动回到起点、不恢复错误、不重试。

```bash
# ESN 开启 + 同步记录（会运动）
/usr/bin/python3 tools/camera_esn_sync/fr3_demo/sync/start_synced.py --start

# 无 ESN 对照 + 同步记录（会运动；不要施力）
/usr/bin/python3 tools/camera_esn_sync/fr3_demo/sync/start_synced.py --start --no-esn

# 在另一个终端请求停止本入口启动的控制进程
/usr/bin/python3 tools/camera_esn_sync/fr3_demo/sync/start_synced.py --stop
```

停止命令使用 PID/start ticks/argv + pidfd 验证目标身份；它不代替现场停止装置。不要同时使用旧入口、WebUI 和本包启动任务。新包会跨目录识别已存在的已知控制进程；独立副本的文件锁不能保证两个入口同时启动时互斥。`--stop` 只管理本包的 active.json，不能用来停止任意旧进程。

本包默认运动配置保持原实验：名义速度 0.105 rad/s（同事速度的 0.7 倍），7 关节运动刚度 300 Nm/rad，有限 normal 碰撞保护。运行本身会调用原控制器的刚度/保护配置；此次发布没有调用它们。退出时控制器尝试恢复硬编码较高刚度，见交接说明。`--no-esn` 旁路减速/让位应用，仍推理、记录神经元；不要把原始 action 当成实际执行量。

## 离线重新生成融合视频

Python 依赖见 `requirements-render.txt`；中文字体依赖 Noto Sans CJK（Ubuntu 包 `fonts-noto-cjk`）。现有主机可用 `/home/airpig/miniforge3/envs/maniskill/bin/python`。编码器由 imageio-ffmpeg 提供。

每次启动最后会打印 `Sync session: ...`。以下 `SESSION` 必须替换为这个实际目录，不能仅用一个 mp4 替代：

```bash
# 自动配对已完成时跳过此步；输出目录须不存在
python tools/camera_esn_sync/fr3_demo/sync/align.py \
  --camera SESSION/camera --policy SESSION/cycle_xxx/execute_diagnostics \
  --output SESSION/alignment

# 输出 SESSION/fusion/，该目录须不存在，避免覆盖旧结果
python tools/camera_esn_sync/fr3_demo/sync/render_fusion.py --session SESSION
```

融合输入：camera.yuyv、camera_frames.csv、clock.json、complete.json；policy_sync.csv、policy_clock.json、policy_sync_status.json；session.json、execute_result.json 以及 alignment 文件。没有原始数据，不能从发布的 8 秒视频恢复完整数值状态。原数据在原主机保存，见交接说明。

输出包括全有效区间视频、受力峰值前后片段、峰值 PNG、每个输出帧的来源映射及摘要。原始相机约 18.4 MB/s，录制前需要足够磁盘；录制期间不编码视频。相机被占用时关闭预览，不要强杀桌面服务。实时画面+神经元 GUI 尚未实现。

## 源码索引

- `fr3_demo/sync/capture_v4l2.cpp`：相机单调时钟采集，YUYV 640×480，约30 Hz。
- `policy_sync.hpp`：控制回调时间 + 状态就绪时间，直接记录 48 输入/128 状态/7 输出，25 Hz；预分配，控制结束后写盘。
- `build_candidate.py`：验证基线源码 hash，四处插桩，编译控制器及相机工具，生成本地配置。
- `start_synced.py` / `../start_esn.py`：相机先 READY，受控启动、基线、ESN 开关、停止、结束后配对。
- `align.py`：按时间而非帧号配对，拒绝跨重启、时间倒退、超限缺口和记录溢出。
- `render_fusion.py`：相机+神经元热图+外力/状态变化/输出曲线；异常退出及无 ESN 模式独立标注。
- `source/`：原控制器、参考轨迹、诊断日志、模型头文件及权重。
- `test_*.py`、`test_policy_logger.cpp`、`clock_probe.cpp`：离线测试与明确标记的合成时钟探针。
