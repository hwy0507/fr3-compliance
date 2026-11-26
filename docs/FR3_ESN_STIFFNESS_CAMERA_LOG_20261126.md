# 2026-11-26：ESN 刚度实验 + RealSense 相机接入

## 一、今天做了什么

1. **把 RealSense D435i 接到本机**，解决了 USB 2.0/3.0 与驱动取流问题，装上官方 SDK 与 Viewer。
2. **用 ESN 模型做了刚度 50 / 100 / 300 的对照实验**，并做了 ESN 开/关的 A/B 对照。
3. 新增/修改了几个工具与可调参数（见第四节）。

---

## 二、RealSense D435i 接入

### 设备信息

| 项 | 值 |
|---|---|
| 型号 | Intel RealSense D435i（`8086:0b3a`）|
| 序列号 | 262422074978 |
| 固件 | 5.15.1.55（Viewer 提示可升 5.16.0.1，未升级）|
| USB 链路 | 必须 **USB 3.0（5000 Mbps）** |

### 节点映射（本机）

```
/dev/video2  → 深度 gray16le
/dev/video6  → 彩色 YUYV 4:2:2（320x180 ~ 1920x1080）
/dev/video3,5,7 → 元数据节点
/dev/video0,1 → 笔记本自带摄像头（无关）
```

### 踩过的两个坑

**① 插到 USB 2.0 口：能枚举、不能出流**

```
设备 1-9: 480 Mbps (USB 2.10)
ffmpeg 取帧 → 卡死（424x240/640x480 都不行）
```

换到 USB3 口（`2-2`，5000 Mbps）后一切正常。**RealSense 在 USB2 上经常"能识别但开流卡死"。**

**② ffplay 直接读 /dev/video6 看不到画面**

日志反复出现：

```
Dequeued v4l2 buffer contains corrupted data (0 bytes).
```

UVC 开流时前 ~32 个缓冲是空的，ffmpeg 会跳过、ffplay 会一直等。所以：

- ✅ `ffmpeg -frames:v N` 抓单帧/多帧：正常
- ❌ `ffplay -f v4l2 -i /dev/video6`：黑屏无帧

**替代方案**：`scripts/camera_preview.sh`（录 1.5 s → 播放 → 循环），延迟 2~3 秒、窗口每轮重开。

### 官方 SDK 安装

Intel 的 apt 源在本网络**DNS 解析不了**，GitHub Release **没有 Linux .deb**（只有 Windows 安装包），因此**源码编译**：

```bash
# 依赖
sudo apt install -y git cmake build-essential libssl-dev libusb-1.0-0-dev \
  libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# 源码（v2.55.1）
wget -c https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.55.1.tar.gz \
     -O librealsense-2.55.1.tar.gz
tar -xzf librealsense-2.55.1.tar.gz && cd librealsense-2.55.1

# 关键：关掉 CHECK_FOR_UPDATES，否则会去 GitHub 拉 libcurl（本网络会卡住）
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DCHECK_FOR_UPDATES=OFF
make -j$(nproc)

# 安装 + udev 权限
sudo make install
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

验证：

```bash
cd ~/librealsense-2.55.1/build
LD_LIBRARY_PATH=$PWD/Release ./Release/rs-enumerate-devices    # 能列出 D435i
LD_LIBRARY_PATH=$PWD/Release ./Release/realsense-viewer        # 官方 GUI
```

> Viewer 需要在**自己的终端**里启动（作为后台任务托管会被提前结束）。
> 打开画面：左侧 `RGB Camera` 开关 → on。

---

## 三、ESN 刚度实验

### 实验设置

| 项 | 值 |
|---|---|
| 路径 | home ↔ pos2（行程 51 cm），部分为 home→pos2→home 连续往返 |
| 名义速度 | 0.15 rad/s |
| 门控 | 进入 12 N / 退出 9 N（0.3 s 确认） |
| 刚度 | 50 / 100 / 300（`STIFFNESS` 环境变量） |
| 碰撞阈值 | contact 20Nm/15N，collision 1000Nm/1000N |
| ESN 开关 | `ESN_OFF=1` 关闭 |

### 结果总表（2026-11-26）

| 刚度 | ESN | 推力峰值 | 让位峰值 | a0 均值 | 终点误差 |
|---|---|---|---|---|---|
| 300 | 开 | 27.4 N | 161 mm | 0.46 | 4.0 cm |
| 300 | 开 | 57.0 N | 178 mm | 0.34 | 0.2 cm |
| 300 | 开 | 49.8 N | 178 mm | 0.52 | 0.5 cm |
| 300 | 开 | 48.4 N | 154 mm | 0.30 | 0.2 cm |
| 100 | 开 | 22.8 N | 136 mm | 0.25 | 1.4 cm |
| 100 | 开 | 30.7 N | 138 mm | 0.21 | 0.8 cm |
| 100 | 开 | 31.2 N | 152 mm | 0.32 | 0.5 cm |
| 100 | 开 | 39.6 N | 205 mm | 0.28 | 0.5 cm |
| 50 | 开 | 28.2 N | 132 mm | 0.25 | 1.9 cm |
| 50 | 开 | 32.5 N | 187 mm | 0.20 | 1.9 cm |
| 50 | 开 | 30.9 N | 108 mm | 0.19 | 2.0 cm |
| 50 | 开 | 20.0 N | 306 mm | 0.47 | 1.8 cm |
| 50 | 开 | 13.4 N | 179 mm | 0.76 | 0.5 cm |
| 50 | 开（未推） | 9.8 N | **0 mm** | 0 | 0.3 cm |
| 50 | 关 | 31.4 N | **0 mm** | 0 | 2.4 cm |
| 50 | 关 | 13.3 N | **0 mm** | 0 | 0.2 cm |
| 50 | 关 | 28.9 N | **0 mm** | 0 | 0.4 cm |

### 结论

1. **让位与减速完全由 ESN 提供**：ESN 关闭时，即使推 30 N 也 **0 mm 让位、速度始终 100%**。
2. **让位量随推力与持续时间增长**：同一刚度下，推力越大、接触越久，让位越多（90 mm → 306 mm）。
3. **刚度只决定"被动柔顺"和末端精度**，不决定"是否让位"：

   | 刚度 | 不推时终点误差 | 被推后终点误差 |
   |---|---|---|
   | 300 | 0.1~0.2 cm | 0.2~0.5 cm |
   | 100 | — | 0.5~1.4 cm |
   | 50 | 0.3 cm | 1.8~2.4 cm |

4. **全程无 reflex**（0 次），推力最高到 57 N。
5. 程序退出时自动恢复刚度 `[3000,3000,3000,2500,2500,2000,2000]`。

### 推荐刚度

| 场景 | 刚度 |
|---|---|
| 演示"极柔顺"（10 N 就能推动） | 50~100 |
| **日常使用 / ESN 默认** | **300** |
| 需要 mm 级精度 | 800~3000 |

---

## 四、代码与工具变更

### 新增环境变量

| 变量 | 默认 | 作用 |
|---|---|---|
| `STIFFNESS` | 300 | `esn_yield` 运行时的关节刚度 |
| `GATE_ENTER` | 12.0 | 接触门控进入阈值（N）|
| `GATE_EXIT` | 9.0 | 接触门控退出阈值（N）|
| `Y_MAX` / `R_MAX` | 0.18 / 0.35 | 让位量程（m / rad）|
| `VEL_LIM` / `ACC_LIM` | 0.15 / 1.0 | 让位速度 / 加速度上限 |
| `ESN_OFF` | 未设置 | 设为非空（如 `1`）关闭策略输出 |

### 新增程序 / 脚本

| 文件 | 作用 |
|---|---|
| `franka-safe/set_stiffness.cpp` | 只设置关节刚度，**不还原**，不做运动 |
| `scripts/esn_chain.sh` | 任意多个姿态的 ESN 链路（A→B→C…），段间停顿可调 |
| `scripts/camera_preview.sh` | RealSense 准实时预览（录 1.5 s → 播放 → 循环） |

### 修复

- **`ESN_OFF` 空字符串误判**：脚本传 `ESN_OFF="${ESN_OFF:-}"` 会传空值，而 C++ 只判断"变量存在"，导致误关闭策略。现在空字符串与 `"0"` 都视为未开启。
- `esn_roundtrip.sh` 的二进制路径修正（`../franka-safe/build/`）。
- `esn_yield` 收尾时用最后下发的 `q_cmd` 结束，并在收尾阶段冻结策略，避免 `Motion finished commanded, but the robot is still moving`。

---

## 五、常用命令

```bash
# 设置刚度（保持，不还原）
~/franka-fci-suite/franka-safe/build/set_stiffness 172.16.0.2 50

# 单程 ESN（可指定刚度、门控）
STIFFNESS=50 GATE_ENTER=12 GATE_EXIT=9 \
  ~/franka-fci-suite/franka-safe/build/esn_yield 172.16.0.2 home pos2 0.15 high

# 多点链路
STIFFNESS=50 ~/franka-fci-suite/scripts/esn_chain.sh 172.16.0.2 home pos2 home 0.15

# 关闭 ESN 做对照
ESN_OFF=1 STIFFNESS=50 ~/franka-fci-suite/scripts/esn_chain.sh 172.16.0.2 home pos2 home 0.15

# 纯位置控制
chrt -f 80 taskset -c 14 ~/franka-fci-suite/franka-safe/build/safe_goto_retry \
  172.16.0.2 home 0.15 5 0.02

# 相机
~/franka-fci-suite/scripts/camera_preview.sh /dev/video6 1.5 200
```

---

## 六、未完成 / 下一步

- **轨迹录制**：用侧面相机拍 home→pos2 的末端轨迹。已有素材：`capture_v4l2.cpp`（带 CLOCK_MONOTONIC 时间戳）、`align.py`、`calibrate_camera_pose.py`；还需写 1 kHz 记录 `O_T_EE` 的记录器与轨迹叠加渲染器。
- **相机摆位**：尚未定好（当前画面只拍到手臂上部与笔记本屏幕）。
- **让位方向一致性**：同批观测下 ESN 让位方向与外力余弦约 +0.33（teacher VMC 为 +0.95），推测训练时观测定义不同，需要训练侧代码。
- 固件升级（5.16.0.1）未做。
