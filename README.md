# Vision 2027 相机标定工具集

RoboMaster 自瞄项目的相机标定与内参验证工具。覆盖「数据采集 → 离线抽帧 → basalt 标定 → PnP 测距验证」完整流程，支持迈德威视 / 海康工业相机 + 大疆 C 板 IMU 联合标定。

本仓库是从主工程 `Vision2027` 中拆出的标定工具子集，独立维护、可单独分发。

---

## 目录结构

```
vision_calibration_tools/
├── README.md                          # 本文件
├── .gitignore
├── CMakeLists.txt                     # 编译 libapriltag2.so
├── .vscode/settings.json
│
├── 采集脚本（Python）
├── calib_common.py                    # 公共库：AprilTag 检测叠加、消息打包、尺寸对齐
├── capture_mindvision_sdk.py          # 迈德威视（SDK）录制 → bag
├── capture_mindvision.py              # 迈德威视（UVC）录制 → bag
├── capture_hikvision.py               # 海康录制 → bag
├── capture_cam_imu.py                 # 相机 + C 板 IMU 同步录制 → bag（联合标定用）
├── extract_calib_frames.py            # 离线自动抽帧：质量过滤 + 姿态多样性
│
├── 标定验证
├── verify_pnp_distance.py             # Python 版 PnP 测距校验（手工点 4 角点）
├── armor_pnp_distance_debug.cpp       # C++ 版 PnP 测距校验（自动检测 + 画面分格统计）
│
├── AprilTag 检测桥接
├── apriltag2_bridge.cpp               # ethz_apriltag2 的 C 接口封装
├── libapriltag2.so                    # 预编译的检测库（采集脚本实时叠加用）
├── tagdiag.cpp / taggen_diag.cpp      # 检测器诊断工具
│
└── 文档
    ├── 迈德威视海康相机标定使用文档.md
    ├── IMU联合标定使用文档.md
    └── 装甲板PnP测距校验使用文档.md
```

---

## 环境依赖

| 依赖 | 用途 | 安装 |
|---|---|---|
| Python 3.8+ | 所有采集/抽帧脚本 | 系统自带 |
| `opencv-python` | 图像读写、预览 | `pip install opencv-python` |
| `rosbags` | 读写 rosbag | `pip install rosbags` |
| `pyserial` | 串口读 IMU（联合标定） | `pip install pyserial` |
| `numpy` | 数值计算 | `pip install numpy` |
| basalt | 相机/IMU 标定 (`basalt_calibrate`, `basalt_calibrate_imu`) | 见下文 |
| 相机 SDK | 迈德威视 / 海康厂商 SDK | 厂商提供 |
| OpenCV (C++) | 编译 `libapriltag2.so` | `apt install libopencv-dev` |

**basalt 安装**：本仓库不含 basalt 源码（约 40 MB，第三方库）。两种方式：

1. **使用系统已安装的 basalt**（推荐）：确认 `which basalt_calibrate` 有输出即可。
2. **从源码编译**：克隆上游 `https://gitlab.com/VladyslavUsenko/basalt.git` 按其 README 编译安装。

---

## 快速开始

### 1. 编译 AprilTag 检测库（仅首次）

`libapriltag2.so` 已预编译在仓库内。如需从源码重新编译（依赖 basalt 的 ethz_apriltag2 头文件 + OpenCV）：

```bash
mkdir build && cd build
cmake .. && make -j$(nproc)
```

产物 `libapriltag2.so` 放回仓库根目录（采集脚本通过同目录查找）。

### 2. 单相机标定流程

```bash
# 2.1 录制（迈德威视 SDK，板子 10×6 = 60 个 tag）
python3 capture_mindvision_sdk.py \
    --out ~/calib_data/mindvision_raw.bag \
    --exposure 3 --cols 10 --rows 6 --grid 60

# 2.2 离线抽帧
python3 extract_calib_frames.py \
    --in ~/calib_data/mindvision_raw.bag \
    --out ~/calib_data/mindvision_calib.bag \
    --cols 10 --rows 6 --grid 60

# 2.3 basalt 标定
source ~/.basalt/env
basalt_calibrate --dataset-path ~/calib_data/mindvision_calib.bag \
    --dataset-type bag \
    --aprilgrid ~/calib_data/mindvision_aprilgrid.json \
    --result-path ~/calib_results/mindvision/ \
    --cam-types ds --no-gui
```

详细步骤与参数说明见 [迈德威视海康相机标定使用文档.md](./迈德威视海康相机标定使用文档.md)。

### 3. 相机-IMU 联合标定

见 [IMU联合标定使用文档.md](./IMU联合标定使用文档.md)。

### 4. 标定结果验证（PnP 测距）

用卷尺实测距离，对比 PnP 解算距离，验证内参是否正确：

```bash
python3 verify_pnp_distance.py --distances 1.0 1.5 2.0 3.0
```

或使用 C++ 工具（自动检测装甲板，依赖主工程 Vision2027 编译）：

```bash
# 在 Vision2027 主工程中编译
cmake --build build --target armor_pnp_distance_debug -j$(nproc)
./build/armor_pnp_distance_debug --camera-backend mindvision --truth-m 2.00
```

详细说明见 [装甲板PnP测距校验使用文档.md](./装甲板PnP测距校验使用文档.md)。

---

## 标定板规格（本项目）

| 参数 | 值 | 说明 |
|---|---|---|
| `tagCols` | 10 | 横向 tag 数 |
| `tagRows` | 6 | 纵向 tag 数 |
| 总 tag 数 | 60 | = 10 × 6 |
| `tagSize` | 0.0137 m | 单个 tag 外沿边长（13.7 mm） |
| `tagSpacing` | 0.32 | 间隙 / tagSize（比值） |

`aprilgrid.json` 示例：

```json
{
  "tagCols": 10,
  "tagRows": 6,
  "tagSize": 0.0137,
  "tagSpacing": 0.32
}
```

> ⚠️ `tagSpacing` 是**比值**，不是间隙绝对长度。填错会导致内参严重失真。

---

## 关于 `armor_pnp_distance_debug.cpp`

该 C++ 工具依赖主工程 `Vision2027` 的 `tup_vision_core` / `tup_vision_yolox_detector`（含 YOLOX 装甲板检测器与 PnP 求解器），**无法在本仓库独立编译**。源码保留在此仅供参考；如需使用，请在主工程中编译，具体见主工程 `CMakeLists.txt` 中的 `armor_pnp_distance_debug` 目标。

---

## 许可证

本仓库工具代码为项目内部使用。`apriltag2_bridge.cpp` 封装的 ethz_apriltag2 检测器版权归原作者所有，遵循其原始许可证。
