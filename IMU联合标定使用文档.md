## 相机 + IMU 联合标定使用文档（basalt）

> 前置：已完成**单相机内参标定**（生成 `calibration.json`）。本流程基于已部署的
> `basalt_calibrate_imu`（二进制已随 basalt 安装于 `~/.local/bin`）。
>
> 适用相机：迈德威视 / 海康威视工业相机
> IMU：位于**独立主控板**（通过串口 / CAN / ROS 等读取的加速度计 + 陀螺仪）

---

### 1. 原理与目标

相机 - IMU 联合标定是为了求取：

1. **相机到 IMU 的外参** `T_imu_cam`（旋转 + 平移）
2. **相机与 IMU 的时间偏移** `cam_time_offset_ns`
3. （可选）IMU 的 **scale / 轴向旋转 / 失准** 校正参数
4. （可选）IMU 与 **Mocap 外参**（需 Mocap 数据）

basalt 在标定时，用 AprilGrid 标定板的视觉观测（相机）+ 相互独立的 IMU 测量，
共同约束一个 B-spline 运动轨迹，从而解出以上参数。

> 关键前提：**相机与 IMU 必须刚性固定**，标定过程中两者相对位置不允许变化。

---

### 2. 环境与依赖

```bash
source ~/.basalt/env          # 每个新终端都要执行，加载 basalt 环境
which basalt_calibrate_imu    # 确认程序存在
# 应输出: ~/.local/bin/basalt_calibrate_imu
```

需要 python3 + `rosbags` + `pyserial` 库（打包 bag / 读串口 IMU）：
```bash
pip install rosbags pyserial
```

---

### 3. 数据采集

`basalt_calibrate_imu` 需要输入一个**同时包含图像和 IMU 数据**的 rosbag：

- 相机图像话题：`sensor_msgs/Image`（每路相机一个，basalt 按图像话题数识别相机数）
- IMU 话题：`sensor_msgs/Imu`（加速度计 + 陀螺仪）
- 时间戳：相机帧与 IMU 原始时间戳（ns）需可靠；相机-IMU 的固定时间偏移由程序估计

> basalt 的 bag 读取逻辑与单相机标定一致：图像话题自动识别。
> 但 IMU 联合标定**必须有 IMU 话题**，否则 `loadDataset` 会因无 IMU 数据而失败。

#### 3.1 采集要求

- 相机 + IMU 刚性固定在同一载体上
- 标定板（AprilGrid）完整、清晰、占画面足够大
- **充分激励六自由度运动**：不能只原地平移，要多做旋转 + 平移组合（摆动、绕各轴转动），这是 IMU 标定收敛的关键
- 采集时长建议 **30 ~ 60 秒**（太长会让 bag 过大、标定极慢）
- 图像帧率 **3 ~ 5 Hz** 即可（帧数太多会导致 basalt 优化耗时数小时，100~150 帧足够）
- IMU 采样率较高（C 板 200 Hz）效果更好
- **图像分辨率必须与运行时一致**（见 §3.3）。脚本默认已按运行时尺寸写入，不用手动处理
- **必须使用手动曝光**（传 `--exposure`），禁止自动曝光——自动曝光会导致亮度变化、角点检测不稳定、时间戳不准（见 §3.5）

##### 运动怎么做

标定板**固定不动**，人抱着车（相机+IMU 刚性固定）在板前做以下动作，**尽量让板子一直在画面内**（短暂消失 <1 秒没问题，长时间 >3 秒不建议）：

1. **俯仰**：枪口/相机上下快速点头，pitch ±30°，来回 20+ 次
2. **偏航**：车头左右快速甩动，yaw ±45°，来回 20+ 次
3. **横滚**：把车体左右倾斜，roll ±30°，来回 10+ 次
4. **平移**：推着车前后、左右移动（1m 范围），配合上面的旋转一起做
5. **大幅混合**：把上面 4 个动作连续组合着做，像"端着车绕圈 + 画 8 字"

> **角速度要够快**：旋转速度应达到 **1~2 rad/s（60~120 deg/s）**。实测若角速度均值
> 仅 0.07 rad/s、最大 <1 rad/s，说明运动太缓慢，IMU 旋转激励不足，外参会退化。
> 用 `--display` 看不到 IMU 曲线，可在采集后用以下命令快速检查激励是否充分：
> ```bash
> python3 -c "
> from rosbags.rosbag1 import Reader
> from rosbags.typesys import Stores, get_typestore
> import numpy as np
> st = get_typestore(Stores.ROS1_NOETIC)
> r = Reader('你的bag.bag'); r.open()
> g = []
> for c, t, d in r.messages():
>     if c.topic == '/imu/data_raw':
>         m = st.deserialize_ros1(d, c.msgtype)
>         g.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
> r.close()
> g = np.array(g)
> print(f'角速度均值: {np.mean(np.linalg.norm(g,axis=1)):.3f} rad/s')
> print(f'角速度最大: {np.max(np.linalg.norm(g,axis=1)):.3f} rad/s')
> print('合格: 最大 > 1.0 rad/s')
> "
> ```

时间分配：30~60 秒里大部分时间保持旋转运动，**最后 5 秒完全静止不动**（这能显著改善零偏估计）。

**错误示范（会标定失败）**：
- ❌ 云台"锁定"板子、只推着车平移——没有旋转激励，`initCamImuTransform` 会崩溃
- ❌ 板子填满整个画面 / 画面模糊——角点检测不到
- ❌ 只有水平方向转，没有俯仰和横滚
- ❌ 运动太慢（角速度 < 0.5 rad/s）——IMU 激励不足，外参退化
- ❌ 用自动曝光（不传 `--exposure`）——亮度波动导致角点检测不稳定

判断标准：采集中用 `--display` 看实时预览，**绿色 tag 框数量 ≥ 9 且清晰可见**才合格。
#### 3.2 打包 bag（现成脚本）

已提供可直接运行的采集脚本 `capture_cam_imu.py`，同时驱动一台相机（海康 / 迈德威视）
+ 大疆 C 板 IMU，用**同一单调时钟**（`time.monotonic_ns()`）给两者打时间戳，
写入同一 bag，供 `basalt_calibrate_imu` 使用：

| 内容 | 话题名 | 类型 |
|------|--------|------|
| 相机0 图像 | `/cam0/image_raw` | `sensor_msgs/Image`（`mono8`） |
| IMU | `/imu/data_raw` | `sensor_msgs/Imu` |

**IMU 数据来源（大疆 C 板，实测可用）**：

C 板（RoboMaster 开发板C型）通过 USB 枚举成 `/dev/ttyACM0`（STM32 Virtual ComPort）。
用 `--imu-fmt cboard` 解析它内置 BMI088 的二进制数据流（帧头 `A5 00 90`，
64 字节/帧，`四元数 + 陀螺仪(rad/s) + 加速度(m/s²)`），已在 ~203 Hz 实测通过，
单位即 basalt 所需的 SI 单位，无需换算。

运行示例（海康相机 + C 板）：
```bash
source ~/.basalt/env

# 海康：
python3 capture_cam_imu.py --camera hikvision \
    --out ~/calib_data/imu_calib.bag \
    --port /dev/ttyACM0 --baud 115200 \
    --imu-fmt cboard \
    --exposure 20 --duration 40 --fps 3

# 迈德威视：
python3 capture_cam_imu.py --camera mindvision \
    --out ~/calib_data/imu_calib.bag \
    --port /dev/ttyACM0 --baud 115200 \
    --imu-fmt cboard \
    --exposure 20 --duration 40 --fps 3
```

> **参数要点**：
> - `--exposure` **必须传**（手动曝光），值根据光照调整（室内 10~30 ms）。不传则海康相机会
>   用自动曝光，迈德威视相机会关闭自动曝光但用上次保存的默认值，都不适合标定。
> - `--fps 3` + `--duration 40` → 约 120 帧图像。帧数太多（如 20fps × 90s = 1800 帧）
>   会让 basalt 联合优化耗时 **5 小时以上**且容易不收敛。
> - `--imu-rate 200` 是默认值，C 板直接用即可。

> C 板没接 / 设备名不是 ttyACM0 时：先 `ls /dev/ttyACM* /dev/ttyUSB*` 确认，
> 再接上后 `python3 -m serial.tools.list_ports -v` 看描述是否为 "STM32 Virtual ComPort"。
> 若你的板子跑的是**自定义固件**、串口输出不是上述二进制帧，改用文本格式：
> `--imu-fmt text`（每行 `ax,ay,az,gx,gy,gz`）或 `--imu-fmt nmea`
> （每行 `PIMU,ax,ay,az,gx,gy,gz`），单位统一为 **m/s² 与 rad/s**。

> **加 `--display` 开实时预览窗口**（含 AprilTag 检测叠加，与 basalt 相同的检测器）：
> 采集中可直接看到绿色 tag 框和整板轮廓，确认**对焦清晰、板子距离合适**
> （每 tag 约 40–60px，不要填满画面）。`q`/`Esc` 或 `Ctrl+C` 结束：
> ```bash
> python3 capture_cam_imu.py --camera mindvision \
>     --out ~/calib_data/imu_e2e_v2.bag \
>     --port /dev/ttyACM0 --imu-fmt cboard \
>     --display --grid 60 --cols 10 --rows 6 \
>     --exposure 20 --duration 40 --fps 3
> ```
> 若预览中绿色检测数为 0，说明失焦或板子图案无法解码，先调整后再正式采集。

常用参数：`--fps` 图像写入帧率上限（默认 5）；`--index` 海康设备序号；
`--imu-fmt` 串口协议（`cboard` / `text` / `nmea`）；`--imu-rate` IMU 采样率
（默认 200，用于时间戳重建，见 §3.4）；`--out-size` 写入 bag 的图像尺寸
（默认 `1280x960`，见 §3.3）。按 `Ctrl+C` 提前结束，脚本会自动
正常关闭 bag（不会出现 0 帧 bag）。

> 关键前提：**相机与 IMU 必须刚性固定**；两条 stream 的时间戳都换算为
> `time.monotonic_ns()`（脚本内置），同一时钟基准，basalt 才能正确估计
> 相机-IMU 时间偏移。

#### 3.3 分辨率必须对齐运行时（重要）

自瞄运行时 [image_utils.h](../../src/hw_io/camera/image_utils.h) 的
`resizeFrameForOutput` 会先按 `mindvision.yaml` 的 `preprocess` 处理原图，
再喂给检测器。当前配置是：

| 环节 | 配置 | 效果 |
|------|------|------|
| 相机原生 | 1280 × 1024 | SDK 直接输出 |
| `preprocess.crop` | `enabled: false` | 不裁切，整幅原图 |
| `preprocess.resize` | `1280 × 960`, `linear` | **非等比拉伸**（x ×1.0、y ×0.9375） |

所以**运行时实际看到的图像是 1280×960 的拉伸图**，不是原生 1280×1024。

标定 bag 里的图像必须经过**完全相同的变换**，basalt 解出的 `fx/fy/cx/cy`
才能直接填进配置使用。脚本默认已经这样做（`--out-size 1280x960`），
**不需要再做任何尺寸换算**。

> 若以后改了 `mindvision.yaml` 的 `preprocess`（例如把 `crop.enabled` 改成
> `true`、或把 `resize` 改成别的尺寸），**必须同步改采集脚本的 `--out-size`**
> 并重新标定，否则内参与实际图像不匹配。
>
> 换算是线性可逆的：设运行时相对标定图像的缩放为 `sx, sy`，则
> `fx' = fx·sx`、`fy' = fy·sy`、`cx' = cx·sx`、`cy' = cy·sy`。
>
> **单相机标定的 bag 也必须是同一尺寸**（`capture_mindvision_sdk.py` /
> `capture_hikvision.py` 录制时默认已按 `1280x960` 写入，见下）。因为本步联合标定会读取
> 单相机标定产出的 `calibration.json` 作为内参**初值且默认不优化**（`opt_intr=false`），
> 两边尺寸不一致会让内参与图像不匹配，联合标定直接发散。
>
> **注**：单相机标定现在分两段——先**录制** `*_raw.bag`（按 R 开始/停止），
> 再用 `extract_calib_frames.py` **离线抽帧**得到 `*_calib.bag`，尺寸在录制阶段就已固定为
> `1280x960`。抽帧只挑帧、不改尺寸，所以不影响本步的内参一致性。详见
> 《迈德威视海康相机标定使用文档》§4。

#### 3.4 IMU 时间戳为什么要重建

串口 IMU 数据是**成批到达**的：主线程调用 `cam.grab()` 取图会被阻塞几十毫秒
（实测约 116 ms），这段时间串口缓冲区会攒下约 23 帧 IMU。如果按"读取时刻"
给整批打同一个时间戳，IMU 时间轴会被压扁——实测 23 个样本挤进 **0.2 ms**，
而它们的真实跨度是 **116 ms**。

后果：basalt 会把 IMU 采样率估成 287 Hz（真实 200 Hz），IMU 因子完全失效，
`T_imu_cam` 的平移会发散成天文数字（实测 `pz = -234 m`，而相机到 IMU 只有几厘米）。

脚本现在的做法（两层修复）：

1. **读串口放在独立线程**（轮询间隔 ~0.5 ms，每次只拿到 1~2 个样本），
   再在批内按 `--imu-rate` 回填时间戳（末样本对齐读取时刻，前面的按 1/rate 往前推），
   保证样本间隔正确。

2. **跨批次单调递增**（`_next_ts` 机制）：如果相邻两批读取间隔小于 `1/rate`，
   简单的"末样本对齐 now"会导致批次间时间戳重叠（实测 9.8% 的样本间隔 <1 ms，
   应该是 5 ms）。脚本维护一个全局 `_next_ts`，每批的第一个样本时间戳取
   `max(now - (n-1)*period, _next_ts)`，保证全局单调递增、间隔严格均匀。

> 大疆 C 板实测 IMU 为 **200 Hz**，默认值 `--imu-rate 200` 直接可用。
> 若你的主控 IMU 是别的速率，按实际值传（例如 `--imu-rate 1000`）。

> **采集后快速验证时间戳是否正确**：
> ```bash
> python3 -c "
> from rosbags.rosbag1 import Reader
> from rosbags.typesys import Stores, get_typestore
> import numpy as np
> st = get_typestore(Stores.ROS1_NOETIC)
> r = Reader('你的bag.bag'); r.open()
> ts = [t for c,t,d in r.messages() if c.topic=='/imu/data_raw']
> r.close()
> ts.sort()
> diffs = np.diff(ts) / 1e6  # ms
> print(f'IMU 平均速率: {len(ts)/(ts[-1]-ts[0])*1e9:.1f} Hz (应≈200)')
> print(f'间隔中位数: {np.median(diffs):.3f} ms (应≈5.0)')
> print(f'间隔<1ms占比: {np.mean(diffs<1)*100:.1f}% (应≈0%)')
> "
> ```
> 合格标准：平均速率 ≈200 Hz，间隔中位数 ≈5.0 ms，<1ms 占比 ≈0%。

#### 3.5 曝光模式（必须手动曝光）

标定时**必须使用手动曝光**（传 `--exposure`），禁止自动曝光，原因：

1. **角点检测不稳定**：自动曝光会随画面亮度变化调整曝光时间，导致 aprilgrid 检测成功率波动
2. **时间戳不准**：曝光时长变化，"曝光中心时刻"不固定，`cam_time_offset_ns` 估计偏差大
3. **运动模糊不一致**：快速旋转时自动曝光可能变长，模糊加剧

| 相机 | 不传 `--exposure` 的行为 |
|------|--------------------------|
| 迈德威视 | 总是关闭自动曝光，使用相机上次保存的手动曝光值 |
| 海康 | **保持自动曝光**（不适合标定） |

所以**两台相机采集时都必须传 `--exposure`**。曝光时间选择：
- 室内正常光照：**10–30 ms**
- 用 `--display` 预览，调到画面亮度适中、tag 边界锐利、无明显运动模糊

---

### 4. 运行 `basalt_calibrate_imu`

#### 4.1 命令

**必须**指向和单相机标定**同一个** `--result-path`，这样它会读取该目录下的
`calibration.json`（内含相机内参），并与 IMU 联合优化：

```bash
source ~/.basalt/env

basalt_calibrate_imu \
  --dataset-path /home/adolf/calib_data/imu_calib.bag \
  --dataset-type bag \
  --aprilgrid /home/adolf/calib_data/hikrobot_aprilgrid.json \
  --result-path /home/adolf/calib_results/hikvision/ \
  --gyro-noise-std 0.000282 \
  --accel-noise-std 0.016 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001
```

参数含义：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--dataset-path` | 必填 | 含图像 + IMU 的 bag |
| `--dataset-type` | 必填 | `bag`（或 `euroc`） |
| `--aprilgrid` | 必填 | 与单相机标定同一个 JSON |
| `--result-path` | 必填 | **同单相机标定目录**，读取 calibration.json |
| `--gyro-noise-std` | 0.000282 | 陀螺仪噪声标准差，按你的 IMU 数据手册填 |
| `--accel-noise-std` | 0.016 | 加速度计噪声标准差 |
| `--gyro-bias-std` | 0.0001 | 陀螺仪零偏漂移标准差 |
| `--accel-bias-std` | 0.001 | 加速度计零偏漂移标准差 |
| `--skip-images` | 1 | 每 N 帧取一帧 |
| `--no-gui` | 关 | 加此参数则全自动跑完（适合无显示器） |

> 噪声参数若不匹配你的 IMU，标定效果会变差。建议从 IMU 数据手册取正确的
> Noise Density 与 Bias Instability（basalt 需要的是 std 而非 density，需按带宽换算）。

> **大疆 C 板（BMI088）参考**：直接沿用上面示例参数即可，实测可收敛。
> 若联合优化发散，可把 `--gyro-noise-std` 放宽到 0.001、`--accel-noise-std` 放宽到 0.05
> （相当于认为测量噪声更大、权重更低，先拿到收敛结果再收紧）。

> **Result-path 必须与对应相机的单相机标定目录一致**（否则读不到内参）：
> - 海康相机 → `~/calib_results/hikvision/`（前面例子）和 `--aprilgrid .../hikrobot_aprilgrid.json`
> - 迈德威视相机 → 都用 `~/calib_results/mindvision/` 和 `--aprilgrid .../mindvision_aprilgrid.json`
> 两套参数不要混用（尤其 `--result-path`）。basalt 会读取该 `--result-path` 目录下的 `calibration.json`（相机内参），再与 IMU 联合优化。

#### 4.2 无 GUI 全自动运行

确认相机内参已在 result-path 存在后，可加 `--no-gui` 自动跑完整流程：

```bash
basalt_calibrate_imu \
  --dataset-path /home/adolf/calib_data/imu_calib.bag \
  --dataset-type bag \
  --aprilgrid /home/adolf/calib_data/hikrobot_aprilgrid.json \
  --result-path /home/adolf/calib_results/hikvision/ \
  --gyro-noise-std 0.000282 \
  --accel-noise-std 0.016 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001 \
  --no-gui
```

运行过程会依次执行：
`loadDataset → detectCorners → initCamPoses → initCamImuTransform → initOptimization`
→ 多轮 `optimizeWithParam(true)` 收敛 →（开启时间偏移/IMU scale 再优化）→ `saveCalib`

#### 4.3 GUI 手动模式（去掉 `--no-gui`，有窗口、有标定结果绘制）

想**像单相机标定一样边看边标**，就不加 `--no-gui` 运行：

```bash
basalt_calibrate_imu \
  --dataset-path /home/adolf/calib_data/imu_calib.bag \
  --dataset-type bag \
  --aprilgrid /home/adolf/calib_data/hikrobot_aprilgrid.json \
  --result-path /home/adolf/calib_results/hikvision/ \
  --gyro-noise-std 0.000282 \
  --accel-noise-std 0.016 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001
```

弹出 Pangolin 窗口，**上半屏**是相机图像视图（可切帧号，会**绘制检测到的角点 /
重投影的角点**，绿点=匹配、红点=剔除，和单相机标定的窗口一样）；
**下半屏**是 IMU 数据曲线（加速度计 / 陀螺仪，以及样条拟合/重投影误差）。
**左侧竖条**是按钮面板。

按序点击左侧按钮：

1. `load_dataset` —— 读入 bag，窗口出现图像和 IMU 曲线
2. `detect_corners` —— 检测角点（图像视图出现绿色/红色角点）
3. `init_cam_poses` —— 初始化相机位姿（窗口开始显示板子网格投影）
4. `init_cam_imu_transform` —— 初始化相机-IMU 外参
5. `init_opt` —— 初始化联合优化
6. `optimize` —— 每次点一下做一轮优化；或勾选上方 `opt_until_convg`
   让它一直优化到收敛（左下角会显示收敛状态）
7. 收敛后勾选 `opt_cam_time_offset`、`opt_imu_scale` 再点几次 `optimize`（微调）
8. `save_calib` —— 保存结果到 result-path 的 `calibration.json`

**如何判断标定效果（在窗口里看）**：
- 图像视图里重投影的绿色角点与板子实际角点**贴合**（几乎不偏移）= 标定准
- 下方曲线里 `show_rot_error`（旋转残差）变小、接近 0 = 收敛好
- 勾 `show_spline` / `show_data` 可看 IMU 数据与样条是否吻合（吻合=时间偏移/外参正确）

> GUI 需要显示器。无显示器环境（如远程、工控机无桌面）用 §4.2 的 `--no-gui` 自动跑。
> Wayland/GNOME 下若窗口不弹或点按钮无响应，退回 `--no-gui` 方式即可（结果相同）。

---

### 5. 结果

在 `--result-path` 目录会更新/生成：

- `calibration.json` —— 内含：
  - `intrinsics`：相机内参（与单相机标定一致）
  - **`T_imu_cam`**：IMU 到相机的外参（本次联合标定的核心产出）
  - **`cam_time_offset_ns`**：相机相对 IMU 的时间偏移

查看：
```bash
cat /home/adolf/calib_results/hikvision/calibration.json
```

重点看 `T_imu_cam` 项（旋转四元数 + 平移）：

```json
"T_imu_cam": [{
    "px": 0.0, "py": 0.0, "pz": 0.0,
    "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0
}]
```

**判断标定是否成功**（同时满足以下条件）：

| 指标 | 合格标准 | 不合格的含义 |
|------|----------|-------------|
| `T_imu_cam` 平移模 | **< 0.3 m**（相机到 IMU 通常几厘米到十几厘米） | >1 m 说明 IMU 因子失效，时间戳有问题 |
| `imu_update_rate` | **≈ 实际 IMU 采样率**（C 板 ≈200 Hz，偏差 <10%） | 偏离大说明时间戳被压扁 |
| `cam_time_offset_ns` | 非 0（通常几 ms），或 0 也可能 | — |
| `intrinsics` | 与单相机标定结果一致 | 不一致说明尺寸没对齐 |
| 重投影误差 | < 1 px | 大说明角点检测或外参有问题 |

- 若 `T_imu_cam` 平移是天文数字（如 `pz = -8 m`）→ IMU 时间戳问题，见 §3.4 重新采集
- 若 `imu_update_rate` 偏离 200 Hz 超过 10% → 同上
- 若单位接近 / 合理且 GUI 中重投影误差小，则标定有效
- `cam_time_offset_ns` 非 0 说明估计出了相机-IMU 时间偏移

#### 5.1 把内参写进自瞄配置（可直接使用）

bag 里的图像已经是运行时尺寸（§3.3），所以 `intrinsics` 里的数值**可以直接抄**，
不需要任何换算。

查看：
```bash
cat /home/adolf/calib_results/mindvision/calibration.json
```

关注 `value0.intrinsics`（cereal 会多包一层 `value0`）和 `value0.resolution`：

```json
{
  "value0": {
    "resolution": [[1280, 960]],          // 必须与 bag / preprocess.resize 一致
    "intrinsics": [{
      "camera_type": "ds",
      "intrinsics": { "fx": 3048.05, "fy": 2857.82,
                      "cx": 648.79, "cy": 538.12,
                      "xi": 0.0012, "alpha": 1.0 }
    }],
    "imu_update_rate": 200.0,             // 应 ≈ 你的 IMU 采样率
    "cam_time_offset_ns": 0
  }
}
```

> `resolution` 若显示 `[[1280, 1024]]`，说明这份 `calibration.json` 是**旧的 1280×1024
> 标定结果**，必须删掉重跑，否则联合标定会拿它当内参固定值用（尺寸不匹配 → 结果全错）。

**判断能否直接当针孔用**：看 `xi` 和 `alpha`。

- `xi ≈ 0` 且 `alpha ≈ 1.0` → Double Sphere 模型**已退化为针孔**（`fx ≈ fy`、畸变 ≈ 0），
  可以直接填进配置，畸变写全 0。**正常镜头基本都是这种情况。**
- 若 `xi` 明显非 0（如 > 0.1）或 `alpha` 明显不等于 1，说明镜头有强畸变，
  需要先把 DS 模型转成针孔+径向切向畸变，再填入（这种情况请找负责人处理）。

填入 [mindvision.yaml](../../src/config/camera/mindvision.yaml)：

```yaml
intrinsics:
  width: 1280          # 必须与 preprocess.resize 一致
  height: 960
  camera_matrix:
    rows: 3
    cols: 3
    # [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    data: [3048.05, 0.0, 648.79, 0.0, 3048.34, 573.99, 0.0, 0.0, 1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [0.0, 0.0, 0.0, 0.0, 0.0]
```

> **`T_imu_cam` / `cam_time_offset_ns` 当前自瞄链路用不到**：自瞄没有 IMU 融合模块，
> 云台姿态来自串口帧的四元数，PnP 直接在相机系解算。这两项先留档，将来做
> camera-gimbal 外参或 IMU 融合时才会用到。

> **标定板参数必须与实际板子一致**：`~/calib_data/mindvision_aprilgrid.json` 里的
> `tagSize`（单个 tag 边长，米）和 `tagSpacing`（tag 间距 / tagSize，**比值不是绝对值**）
> 若填错，板子几何会被误解，内参和外参都会偏。basalt 自带参考值
> `~/.local/etc/basalt/aprilgrid_6x6.json` 是 `tagSpacing: 0.3`（间距 = tagSize 的 30%）。
> 量一下实际板子的 tag 边长 `L` 与相邻 tag 的间隙 `G`，则
> `tagSize = L`（米）、`tagSpacing = G / L`。
>
> **本项目（迈德威视）的实际板子**（已量好，直接用）：
>
> ```json
> { "tagCols": 10, "tagRows": 6, "tagSize": 0.0137, "tagSpacing": 0.32 }
> ```
>
> 即横向 10 个、纵向 6 个 tag（共 60 个），单个 tag 边长 13.7 mm，间隙约 4.4 mm。
> 采集时对应 `--cols 10 --rows 6 --grid 60`。详见《迈德威视海康相机标定使用文档》§3.0。

---

### 6. 常见问题

| 现象 | 原因 / 处理 |
|------|-------------|
| `Total number of messages` 里 IMU 为 0 | bag 中没有 `sensor_msgs/Imu` 话题，检查 IMU 打包 |
| **程序崩溃 / core dump（`initCamImuTransform` 处）** | 角点检测到的帧太少。原因多为：① 画面失焦（先 `--display` 确认绿色框 ≥9）② 板子离太近填满画面 ③ 只有平移没有旋转激励。按 §3.1 重新采集 |
| **采集中 `--display` 预览绿色框为 0** | 画面失焦或板子图案不清晰。调焦距、拉开距离到板子占画面 1/3~1/2 |
| 标定不收敛 / 误差大 | 旋转激励不足、时长不够、噪声参数不对。按 §3.1 重采，或放宽 `--gyro-noise-std 0.001`、`--accel-noise-std 0.05` |
| GUI 窗口不弹出 / 按钮点不动 | 无显示器，或用 `--no-gui` 自动跑（结果相同） |
| 需要先标相机内参 | `basalt_calibrate_imu` 依赖 result-path 下已有 `calibration.json`，必须先跑 `basalt_calibrate` 单相机标定 |
| `T_imu_cam` 全 0 / 奇异 | 相机-IMU 时间戳不同步或外参初始化失败，检查时间戳时钟是否统一 |
| **`T_imu_cam` 平移是天文数字**（如 `pz = -234 m`） | IMU 时间戳被压扁：整批样本共用同一时刻，`imu_update_rate` 被估错，IMU 因子失效。用 §3.4 的脚本（已修复）重新采集；若自己改过脚本，确认读串口在独立线程且做了批内回填 |
| **内参填进去后 PnP 距离明显不准** | bag 图像尺寸与运行时 `preprocess.resize` 不一致。确认采集时 `--out-size` 与 `mindvision.yaml` 的 `resize` 相同（见 §3.3） |
| `Could not open aprilgrid configuration` | `--aprilgrid` 路径拼错，用 `ls ~/calib_data/*.json` 核对 |

---

### 7. 完整流程速查

> 前置：先按《迈德威视海康相机标定使用文档》完成单相机标定（生成 `calibration.json`）。
> 单相机标定为**两段式**：录制 `*_raw.bag`（按 R 开始/停止）→ `extract_calib_frames.py` 离线抽帧
> 得到 `*_calib.bag` → `basalt_calibrate` 标定。本步的 `capture_cam_imu.py` 不受影响，
> 仍是**边录边写**（无需抽帧）。

```bash
# 0. 环境（每个新终端都要执行）
source ~/.basalt/env

# 0b. 【必做】清掉旧的 result-path：basalt 会无条件加载该目录下的
#     calibration.json(内参初值, 默认不优化) 和 *_detected_corners.cereal /
#     *_init_poses.cereal 缓存, 旧文件(如 1280x1024 的结果)会让新标定静默出错。
mkdir -p ~/calib_results/mindvision
rm -f ~/calib_results/mindvision/calibration.json \
      ~/calib_results/mindvision/*.cereal

# 1. 采集（相机 + C板 IMU 打包进 bag；加 --display 可实时看检测结果）
python3 capture_cam_imu.py --camera mindvision \
    --out ~/calib_data/imu_calib.bag \
    --port /dev/ttyACM0 --imu-fmt cboard \
    --display --grid 60 --cols 10 --rows 6 \
    --exposure 20 --duration 40 --fps 3

#   采集中：预览窗口绿色框 ≥ 9 个才算合格；
#   按 §3.1 动作做 30~60s 旋转运动（角速度 1~2 rad/s），最后 5s 静止；q/Esc 结束。
#   采集后用 §3.1 的脚本检查角速度激励、§3.4 的脚本检查时间戳是否正常。

# 2. 相机-IMU 联合标定（result-path 与单相机标定相同）
basalt_calibrate_imu \
  --dataset-path /home/adolf/calib_data/imu_calib.bag \
  --dataset-type bag \
  --aprilgrid /home/adolf/calib_data/mindvision_aprilgrid.json \
  --result-path /home/adolf/calib_results/mindvision/ \
  --gyro-noise-std 0.000282 \
  --accel-noise-std 0.016 \
  --gyro-bias-std 0.0001 \
  --accel-bias-std 0.001 \
  --no-gui

# 想边看边标（有窗口绘制角点/重投影）就去掉 --no-gui，按 §4.3 点按钮

# 3. 查看结果
cat /home/adolf/calib_results/mindvision/calibration.json
#   看 "T_imu_cam"（不再是单位阵 = 标定成功）和 "cam_time_offset_ns"
#   同时看 "intrinsics" 的 fx/fy/cx/cy（xi≈0、alpha≈1 即可当针孔用）

# 4. 把内参写进自瞄配置（bag 已是 1280x960，数值直接抄，无需换算）
#    按 §5.1 填入 src/config/camera/mindvision.yaml 的 intrinsics 段
```

> **关于 `--result-path` 里的 `calibration.json`**：`basalt_calibrate_imu` 会先尝试
> 读取该目录下的 `calibration.json` 作为内参初值；文件不存在时会打印
> `No calibration found. Run camera calibration first!!!` 并退回到默认初值继续跑
> （不中断）。为拿到更可靠的内参，建议先按《迈德威视标定使用文档》跑一次
> **单相机标定**，把 `calibration.json` 放到同一个 `--result-path` 目录下。

> 海康相机：把上面 `--camera hikvision`、`--aprilgrid .../hikrobot_aprilgrid.json`、
> `--result-path .../hikvision/` 三处换掉，其余不变。

---

### 8. 同步采集脚本工作原理

你已经有了现成脚本 `capture_cam_imu.py`（见 §3.2），本节说明它的工作逻辑，
便于你根据主控协议调整：

```text
主线程:   循环取相机帧 (SDK, 会阻塞) -> 灰度 -> 缩放到 --out-size
          -> 记时间戳 -> 写入 /cam0/image_raw
          + 每轮把 IMU 队列里的样本写入 /imu/data_raw
IMU 线程: 高频轮询串口 (间隔 ~0.5ms) -> 解析样本
          -> 批内按 1/--imu-rate 回填时间戳 -> 压入队列
```

要点：
- **IMU 必须单独线程读**：主线程被 `cam.grab()` 阻塞约 116 ms，若在同一循环里
  读串口，整批样本会共用同一个时间戳，IMU 时间轴被压扁（详见 §3.4）
- **批内回填时间戳**：串口一次读出 N 个样本时，末样本对齐读取时刻，前面的按
  `1/--imu-rate` 往前推，保证样本间隔正确
- **跨批次单调递增**：维护全局 `_next_ts`，避免相邻批次时间戳重叠（详见 §3.4）
- 相机时间戳与 IMU 时间戳使用**同一时钟源**（`time.monotonic_ns()`，脚本内置）
- 图像写入前会做与运行时相同的缩放（§3.3），保证内参可直接使用
- 采集时让载具做**大幅旋转 + 平移**，覆盖六自由度激励，角速度 1~2 rad/s
- 相机 3~5 Hz（`--fps` 控制写入帧率），IMU 尽量 ≥100 Hz
- 时长 30~60 秒，标定板尽量在画面内（短暂消失 <1s 可接受）
- 必须传 `--exposure` 手动曝光，禁止自动曝光（§3.5）
- 若要改 IMU 串口协议，改 `parse_imu_cboard()`（`cboard`）或
  `parse_imu_line()`（`text` / `nmea`）即可