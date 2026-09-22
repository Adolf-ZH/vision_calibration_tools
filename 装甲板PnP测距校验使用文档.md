# 装甲板 PnP 测距校验使用文档

用「已知距离的装甲板」验证相机内参标定是否正确的操作手册。

对应工具：`相机标定/armor_pnp_distance_debug.cpp`（编译产物 `build/armor_pnp_distance_debug`）

---

## 1. 这份文档解决什么问题

相机内参（`fx / fy / cx / cy / 畸变`）标定完之后，怎么知道它是对的？

标定工具（basalt / OpenCV）给出的**重投影误差**只能说明「内参和这批数据自洽」，不能说明「内参符合这台相机的物理真相」——如果标定板几何配错、数据姿态太单一，重投影误差可以很小但内参完全错（本项目就真实发生过：`tagCols` 填错导致 `fx` 差了一个数量级）。

**唯一可靠的验证方式是引入一个外部真值**：把装甲板放在相机前一个**用卷尺量出来的距离**处，让程序跑「装甲板识别 + PnP」，读出它解算的距离，和卷尺读数比对。

这个工具就是干这件事的：**迈德威视相机 → ArmorDetector → ArmorPnpSolver → 打印距离**。它不需要串口、云台、IMU，台架上就能跑。

---

## 2. 原理：为什么这样能验证焦距

PnP 解出的距离服从：

```
z = fx_used × W_true / w_px
```

- `z` —— PnP 解出的相机到装甲板距离
- `fx_used` —— 配置里填的焦距（像素）
- `W_true` —— 装甲板真实宽度（米），本项目 = `autoaim.yaml` 的 `small_armor_width_m` = 0.135
- `w_px` —— 装甲板在图像里的像素宽度

**推论一**：如果配置的 `fx` 偏大 k 倍，解出的距离也偏大 k 倍。所以拿卷尺一量就能发现。

**推论二（更重要）**：把上式反解出焦距：

```
fx_true = z_meas × w_px / W_true
```

把 `z = fx_used × W_true / w_px` 代进去，`fx_used` 会被约掉：

```
implied_fx = fx_used × truth / z = truth × w_px / W_true
```

也就是说，工具输出的 **`implied_fx` 是一个独立于「你配置了什么 fx」的直接测量结果**——它就是「图像里装甲板的像素宽度 + 卷尺距离」反推出来的真实焦距。

于是判据极其简单：

> **`implied_fx` 应该等于你配置里的 `fx`。**

两者一致 → 标定正确。两者差多少倍，说明配置的 `fx` 就错多少倍。

---

## 3. 前置条件

| 项 | 要求 |
|---|---|
| 构建 | 项目已用 OpenVINO 编译，且 `TUP_VISION_BUILD_DEBUG_TOOLS=ON`（默认开） |
| 相机 | 迈德威视 / 海康 / USB 相机，能出图 |
| 装甲板 | 一块灯条完整的装甲板，`0.135 m × 0.056 m`（小装甲板） |
| 卷尺 | 量相机到装甲板的距离 |
| 内参配置 | 一份相机 yaml，里面的 `intrinsics.camera_matrix` / `distortion_coefficients` 是待验证的值 |

---

## 4. 编译

```bash
cd /home/adolf/桌面/Vision2027

cmake -S . -B build
cmake --build build --target armor_pnp_distance_debug -j$(nproc)
```

产物：`build/armor_pnp_distance_debug`

确认编译成功：

```bash
./build/armor_pnp_distance_debug --help
```

---

## 5. 命令行参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--source camera\|video` | `camera` | 输入源 |
| `--video <path>` | — | `--source video` 时的视频文件路径 |
| `--camera-backend auto\|mindvision\|hikrobot\|video` | `auto` | 相机后端；`auto` 按 hikrobot → mindvision → video 顺序尝试 |
| `--camera-config <path>` | `src/config/camera/<backend>.yaml` | 相机配置，**内参就是从这里读的** |
| `--detector-config <path>` | `src/modules/autoaim/detector/nn/tup_yolox/tup_yolox.conf` | 神经网络配置文件 |
| `--model <path>` | 用 conf 里的 | 覆盖模型路径 |
| `--device CPU\|GPU` | 用 conf 里的 | 推理设备 |
| `--width W --height H` | `0`（用相机配置尺寸） | **验证时务必保持 0**，见 §10 |
| `--cv-refine` / `--no-cv-refine` | `--cv-refine` | 是否做 CV 角点细化 |
| `--armor small\|big` | `small` | 装甲板尺寸档位，对应 `0.135×0.056` / `0.230×0.056` |
| `--truth-m D` | `0`（不给） | 卷尺实测的相机→装甲板距离（米）。**给了才输出 `ratio` 和 `implied_fx`** |
| `--summary-every N` | `30` | 每 N 帧输出一次窗口统计 |
| `--frames N` | `0`（不限） | 处理多少帧后退出 |
| `--display` / `--no-display` | `--display` | 是否开预览窗口；`q`/`Esc` 退出 |
| `--help` / `-h` | — | 打印帮助 |

---

## 6. 操作步骤

### 6.1 准备内参配置

工具从 `--camera-config` 指向的 yaml 读内参。两种做法：

**做法 A：直接改工程配置**（验证通过后就长期用这个）

编辑 `src/config/camera/mindvision.yaml`，把 `intrinsics` 段换成待验证的值：

```yaml
intrinsics:
  width: 1280
  height: 960
  camera_matrix:
    rows: 3
    cols: 3
    data: [3085.2872, 0.0, 640.0, 0.0, 2896.8446, 480.0, 0.0, 0.0, 1.0]
  distortion_coefficients:
    rows: 1
    cols: 5
    data: [-0.089927, 0.504720, -0.000338, 0.000830, 0.0]
```

**做法 B：不动工程配置，生成一份临时副本**

```bash
cd /home/adolf/桌面/Vision2027
python3 -c "
import yaml, pathlib
p = pathlib.Path('src/config/camera/mindvision.yaml')
d = yaml.safe_load(p.read_text())
it = d['intrinsics']
it['camera_matrix']['data'] = [3085.2872, 0.0, 640.0, 0.0, 2896.8446, 480.0, 0.0, 0.0, 1.0]
it['distortion_coefficients']['data'] = [-0.089927, 0.504720, -0.000338, 0.000830, 0.0]
pathlib.Path('/tmp/mv_new_full.yaml').write_text(
    yaml.safe_dump(d, allow_unicode=True, sort_keys=False))
print('written /tmp/mv_new_full.yaml')
"
```

启动时工具会打印一次生效的配置，**先核对内参这几行再往下做**：

```
[info] [armor_pnp_distance_debug] armor_pnp_distance_debug ready
[info] [armor_pnp_distance_debug]   source: camera:mindvision config=/tmp/mv_new_full.yaml
[info] [armor_pnp_distance_debug]   backend: mindvision
[info] [armor_pnp_distance_debug]   resolution: 1280x960
[info] [armor_pnp_distance_debug]   intrinsics fx/fy/cx/cy: 3085.29 / 2896.84 / 640 / 480
[info] [armor_pnp_distance_debug]   distortion: -0.089927, 0.504720, -0.000338, 0.000830, 0.000000
[info] [armor_pnp_distance_debug]   armor: small width/height: 0.135 / 0.056
[info] [armor_pnp_distance_debug]   detector_config: src/modules/autoaim/detector/nn/tup_yolox/tup_yolox.conf
[info] [armor_pnp_distance_debug]   model: src/modules/autoaim/detector/nn/tup_yolox/model/opt-0527-001.xml
[info] [armor_pnp_distance_debug]   device: CPU
[info] [armor_pnp_distance_debug]   cv_refine: true
[info] [armor_pnp_distance_debug]   display: true
[info] [armor_pnp_distance_debug]   truth_m: 2
```

（`[info] [armor_pnp_distance_debug]` 是日志前缀，实际每行都有；这里保留以便你对照真实输出。）

如果 `fx` 打印的还是旧的占位值（`4280.16`），说明 `--camera-config` 没指对。`resolution` 必须是 `1280x960`——标定就是在这个尺寸下做的。

### 6.2 摆位

1. 相机固定不动（三脚架/夹具），避免抖动。
2. 装甲板正对相机，**尽量别歪**（倾斜会引入额外误差）。
3. 装甲板在画面里**占得越大越准**（但别出框）。
4. 用卷尺量**相机到装甲板**的距离，记下来。

> 参考点（从镜头前端量还是从相机外壳量）不用纠结——它带来的固定偏差只影响绝对距离，不影响 §8 的斜率判据。但**每次量法要一致**。

### 6.3 运行

```bash
cd /home/adolf/桌面/Vision2027

./build/armor_pnp_distance_debug \
  --camera-backend mindvision \
  --camera-config /tmp/mv_new_full.yaml \
  --truth-m 2.00
```

会弹出一个预览窗口，画出识别到的装甲板 4 个角点，并叠加 `z=`、`reproj=`、`fx=`、`ratio=`。

**第一件事：确认框住的是你摆的那块板。** 如果框歪了或者框到别的东西上，后面的数字都没有意义。

按 `q` 或 `Esc` 退出。

> 台架没有显示器时加 `--no-display`，但此时必须用 `--frames N` 限定帧数，否则会一直跑下去（用 `Ctrl+C` 停）。

### 6.4 记录

把退出前最后几行 `[summary]` 记下来，就是这一档距离的测量结果。

按 `q`/`Esc` 退出后，还会多打印一张 `[region]` 画面分格表（见 §7），记录装甲板摆在画面不同位置时的测距差异。

---

## 7. 输出解读

### 7.1 逐帧行

```
[0123] armors=1 z=2.0431 |t|=2.0455 reproj=0.42px truth=2.000 ratio=1.022 implied_fx=3020.1
```

| 字段 | 含义 |
|---|---|
| `[0123]` | 帧序号 |
| `armors=1` | 这一帧检测到几个装甲板 |
| `z` | PnP 解出的深度（米），= `tvec[2]`，**这就是要跟卷尺比的值** |
| `\|t\|` | 平移向量的模（米），正对时约等于 `z` |
| `reproj` | 该装甲板的重投影误差（像素）。**大于 1~2px 说明角点质量差，这一帧别信** |
| `truth` | `--truth-m` 给的值 |
| `ratio` | `z / truth`。**目标 1.00** |
| `implied_fx` | 反推的真实焦距，**目标 = 配置的 fx** |

一帧里有多个装甲板时，工具选**重投影误差最小**的那个作为代表。

### 7.2 汇总行

```
[summary] frames=30 hit=30 z_median=2.0421 z_mean=2.0435 z_std=0.0031 implied_fx_median=3020.4 ratio_median=1.021
```

| 字段 | 含义 |
|---|---|
| `frames` | 本窗口的帧数 |
| `hit` | 其中成功解出装甲板的帧数。**`hit` 太低说明识别不稳，结果不可信** |
| `z_median` | 命中帧 `z` 的中位数，**这就是 PnP 测出的相机→装甲板距离，也是该档距离的最终测量值** |
| `z_mean` / `z_std` | 均值和标准差。`z_std` 大说明抖动大 |
| `implied_fx_median` | 反推焦距的中位数（有 `--truth-m` 时才输出） |
| `ratio_median` | `z_median / truth`，**这就是「PnP 距离 vs 卷尺距离」的比值，目标 1.000** |

### 7.3 判据

| 观察 | 结论 |
|---|---|
| `implied_fx_median` ≈ 配置的 `fx`，`ratio` ≈ 1.00 | ✅ 内参正确 |
| `implied_fx_median` 明显偏离（差 10% 以上） | ❌ 内参有问题，`implied_fx_median` 就是它应该被改成多少 |
| `z_std` 很大 / `hit` 很低 / `reproj` 很大 | ⚠️ 测量本身不可信，先解决识别或拍摄问题 |
| `ratio` 约 0.72 而 `implied_fx` 约 3085 | 配置里填的 `fx`（4280）偏大 1.39 倍，`implied_fx` 才是对的 |

### 7.4 单点对比的局限（重要）

**一个距离上 `ratio ≠ 1`，不能直接判定标定错了。** 因为 PnP 的距离服从

```
z = fx_used × W_assumed / w_px
```

单点测量只能给出 `z` 和 `truth` 的一个比值。而任何**整体等比**的误差（例如焦距偏大、装甲板尺寸常数偏大）都会被 PnP 等比例吸收进 `z`，在单点上表现完全一样，分不开。

还要特别注意：**这类比例误差在 `reproj` 里是完全隐形的。** 因为 `fx` 整体缩放时 PnP 会等比例改变 `z` 来补偿，重投影误差恒为 0。所以 **`reproj` 很小不能证明 `fx` 是对的**，`z` 与 `truth` 一致才能。

**所以要换 2~3 个距离各跑一遍，看 `ratio_median` 随距离怎么变：**

| 观察 | 含义 |
|---|---|
| 各档 `ratio_median` 基本恒定（例如都是 1.08） | 纯比例误差，与距离无关 |
| `ratio_median` 随距离单调向 1.000 靠拢 | 存在固定偏移（典型是卷尺基准点，量的不是「光心 → 板面」），距离越远影响越小 |

多点数据还可以进一步做线性拟合：把 `z_median` 对 `truth` 拟合一条直线，**斜率 = 比例因子，截距 = 固定偏移**，两个误差来源一次性分开。

### 7.5 画面分格输出（`[region]`）

程序退出时还会打印一份**按画面位置分格**的统计，用来检查视野**上/下/左/右/中心**（以及四角）的测距误差是否一致：

```
[region] 3x3 breakdown over 1280x960 (rows top->bottom, cols left->right), frames=300
[region] top-left     hit=12  z_median=2.0503 pos=(213,160) ratio_median=1.025 implied_fx_median=3010.2
[region] top          hit=41  z_median=2.0450 pos=(640,158) ratio_median=1.023 implied_fx_median=3018.7
[region] top-right    hit=9   z_median=2.0621 pos=(1067,161) ratio_median=1.031 implied_fx_median=2992.4
[region] left         hit=38  z_median=2.0448 pos=(210,480) ratio_median=1.022 implied_fx_median=3019.1
[region] center       hit=96  z_median=2.0421 pos=(639,478) ratio_median=1.021 implied_fx_median=3020.4
[region] right        hit=35  z_median=2.0469 pos=(1069,481) ratio_median=1.023 implied_fx_median=3016.0
[region] bottom-left  hit=11  z_median=2.0555 pos=(214,800) ratio_median=1.028 implied_fx_median=3003.8
[region] bottom       hit=44  z_median=2.0466 pos=(641,802) ratio_median=1.023 implied_fx_median=3015.5
[region] bottom-right hit=14  z_median=2.0640 pos=(1066,799) ratio_median=1.032 implied_fx_median=2989.6
```

网格固定为 **3×3**，把画面横竖各三等分；每个命中帧按**代表装甲板检测框的中心像素**落入某一格。

| 字段 | 含义 |
|---|---|
| `top-left` … `bottom-right` | 区域名，行序自上而下、列序自左而右 |
| `hit` | 落在该格的命中帧数。**`hit=0` 的格没数据，不能解读** |
| `z_median` | 该格的 PnP 距离中位数（米） |
| `pos=(x,y)` | 该格命中帧检测框中心的平均像素坐标，用来确认装甲板实际落在哪儿 |
| `ratio_median` | `z_median / truth`（有 `--truth-m` 时才输出） |
| `implied_fx_median` | 该格反推焦距中位数（有 `--truth-m` 时才输出） |

**怎么用**：把装甲板固定在某一距离**不动**，依次把它摆到画面**中心 / 上 / 下 / 左 / 右**（有精力再加四个角），每个位置各跑十几秒，最后一次性看 `[region]` 表。

| 观察 | 含义 |
|---|---|
| 各格 `z_median`（或 `implied_fx_median`）基本一致 | ✅ 畸变模型在整个视野内一致，离轴处也准 |
| 边角格系统性偏离中心格（通常表现为边角 `z_median` 偏大、`implied_fx_median` 偏小） | ❌ 畸变模型在全视野上不准，或标定时标定板没覆盖到边缘 |
| 只有 `hit` 很低 / `z_std` 大 | ⚠️ 该位置识别不稳或模糊，先解决拍摄问题再看数 |

> **注意**：分格统计只比较**同一档距离**下的各格数据。装甲板移动位置时**别改卷尺距离**，否则各格 `z_median` 的差异里会混进距离变化，读不出来。

---

## 8. 推荐的完整验证流程

### 8.1 多距离测斜率

单点测量只能看「差多少」，多点测量还能看出「是否成比例」。

分别在几档距离各跑十几秒，记录 `z_median`：

| 卷尺实测 `truth` | `z_median` | `ratio` | `implied_fx_median` |
|---|---|---|---|
| 1.00 m | | | |
| 2.00 m | | | |
| 3.00 m | | | |
| 4.00 m | | | |

- **`ratio` 应稳定在 1.00 附近**；若随距离单调漂移，说明有系统性问题（畸变模型不合适、装甲板尺寸配错等）。
- **`implied_fx_median` 在各档上应基本一致**。这一条比斜率更有说服力——因为它不依赖「距离从哪儿量起」。

### 8.2 对照实验（强烈建议做一次）

用一个**故意错的** `fx` 跑一遍，确认这个测试真的有分辨能力：

```bash
# 不加 --camera-config，用工程里那份占位内参（fx=4280.16）
./build/armor_pnp_distance_debug --camera-backend mindvision --truth-m 2.00
```

预期结果：
- `ratio` ≈ 0.72（因为 4280 偏大 1.39 倍）
- **`implied_fx_median` 仍然 ≈ 3085**

如果 `implied_fx` 在两份配置下都给出一致的值，说明这个测量确实独立于配置、且新标定是对的。**这个对照比单跑一遍可信得多。**

---

## 9. 常见问题

**Q：一帧都检测不到装甲板（`hit=0`）**
- 检查曝光：装甲板灯条过曝或太暗都检不出，可改相机配置里的曝光参数
- 检查距离/大小：太远时装甲板像素太少，NN 检不到；建议 1~4 m
- 检查装甲板是否完整入画
- 试试 `--no-cv-refine` 排除 CV 细化阶段的问题

**Q：检测到了但框歪了**
- 确认 `--armor` 档位对不对（小/大装甲板尺寸不同）
- 确认预览里框的是装甲板的灯条外接矩形

**Q：`z` 抖动很大（`z_std` 大）**
- 相机或装甲板没固定好
- 装甲板太斜
- 距离太远导致像素太少

**Q：`reproj` 很大（> 2px）**
- 角点检测质量差，这一帧的数据别用
- 检查是否运动模糊、曝光是否合适

**Q：`--display` 窗口打不开**
- 用 `--no-display --frames 200` 纯读数字

**Q：`implied_fx` 每次都不一样**
- 装甲板尺寸（`W_true`）填错了——`implied_fx` 与 `W_true` 成反比
- 检查 `autoaim.yaml` 的 `small_armor_width_m` 是否和实物一致

---

## 10. 重要注意事项

### 10.1 不要传 `--width / --height`

这两个参数只改输出尺寸，**不会同步缩放 `camera_matrix`**。标定是在 1280×960 下做的，传了别的尺寸内参就失效，距离会整体偏。**验证时保持默认 0。**

### 10.2 与运行时的一致性

这个工具刻意走和运行时完全相同的路径：

- 相机取流：`tup_vision::camera::loadCameraConfig` + `CameraFactory`，和主程序同一套代码
- 尺寸处理：用相机配置里的 `preprocess` 设置，和运行时一致
- 识别：`ArmorDetector`（NN + CV refine），和运行时同一个类
- PnP：`ArmorPnpSolver`，和运行时同一个类，`cv::solvePnPGeneric` + OpenCV 5 参数畸变

所以它测出来的结论可以直接代表运行时的表现。

### 10.3 装甲板尺寸的来源

| 参数 | 值 | 来源 |
|---|---|---|
| 小装甲板宽 | `0.135 m` | `src/config/autoaim.yaml` → `aimer.fire_controller.small_armor_width_m` |
| 装甲板高 | `0.056 m` | `src/config/autoaim.yaml` → `aimer.fire_controller.small_armor_height_m` |
| 大装甲板宽 | `0.230 m` | `armor_pnp_solver.h` 的 `big_armor_width_m` |

工具内置的默认值与上表一致。注意 `implied_fx` 与装甲板宽度成反比，**这个值错了会直接导致误判**，务必和实物核对。

---

## 11. 备选方案

`相机标定/verify_pnp_distance.py` 是同一件事的 Python 版本，区别是**装甲板角点靠手工点击**而不是跑识别。

什么时候用它：想**把标定误差和识别误差分开**。识别器的角点有它自己的系统偏差（灯条边缘、CV 细化的偏好），手工点角点虽然精度低一点，但没有检测器偏差，能更纯粹地反映内参质量。

用法见脚本头部的注释。

---

## 12. 相关文件

| 文件 | 作用 |
|---|---|
| `相机标定/armor_pnp_distance_debug.cpp` | 本工具的源码 |
| `CMakeLists.txt`（`armor_pnp_distance_debug` 目标，约 489 行） | 编译规则；移动源码时要同步改这里的路径 |
| `相机标定/verify_pnp_distance.py` | 备选的 Python 版（手工点角点） |
| `src/config/camera/mindvision.yaml` | 迈德威视相机内参配置 |
| `src/config/autoaim.yaml` | 装甲板尺寸配置 |
| `src/modules/autoaim/pnp/armor_pnp_solver.cpp` | PnP 解算实现 |
| `相机标定/迈德威视海康相机标定使用文档.md` | 内参标定流程 |
