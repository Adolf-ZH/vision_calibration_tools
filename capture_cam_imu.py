#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机 + 独立主控 IMU 同步采集 → basalt IMU 联合标定 bag
=====================================================
同时驱动一台相机(海康/迈德威视)和串口 IMU, 用同一系统时钟(system-monotonic ns)
给两者打时间戳, 写入一个 rosbag, 供 basalt_calibrate_imu 使用。

话题:
  /cam0/image_raw      sensor_msgs/Image (mono8 灰度)
  /imu/data_raw        sensor_msgs/Imu   (linear_acceleration + angular_velocity)

IMU 串口帧协议(默认 text 模式, 每行 6 个浮点, 逗号分隔):
    ax,ay,az,gx,gy,gz         单位: 加速度 m/s^2, 角速度 rad/s
例:
    0.02,-0.10,9.81,0.001,0.002,-0.003

用法:
  # 海康 + 串口 IMU
  python3 capture_cam_imu.py --camera hikvision \
      --out ~/calib_data/imu_calib.bag \
      --port /dev/ttyUSB0 --baud 115200 \
      --exposure 30 --duration 90

  # 迈德威视 + 串口 IMU
  python3 capture_cam_imu.py --camera mindvision \
      --out ~/calib_data/imu_calib.bag \
      --port /dev/ttyUSB0 --baud 115200 \
      --exposure 30 --duration 90

配套建议:
  1. 相机/IMU 刚体固定, 采集前先跑 basalt_calibrate 单相机得到 calibration.json
  2. 采集时载具做大幅度六自由度运动(绕 X/Y/Z 各轴转+平移), 持续 60~120s
  3. 标定板始终完整、清晰、占画面足够大

分辨率:
  写入 bag 的图像默认缩放到 1280x960, 与运行时 mindvision.yaml 的
  preprocess(crop 关闭 + resize 1280x960) 完全一致, 因此 basalt 解出的内参
  可以直接填进配置使用, 无需再做尺寸换算。用 --out-size 可改。

IMU 时间戳:
  串口数据成批到达, 由独立线程高频读取并按 --imu-rate 在批内回填时间戳,
  避免整批样本共用同一时刻(否则 basalt 的 IMU 因子失效、外参发散)。
"""
import argparse
import ctypes
import os
import queue
import select
import struct
import sys
import threading
import time

import cv2
import numpy as np

from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

import serial  # 需: pip install pyserial

# basalt 需要的时间基准: NTP/Unix → ROS time
try:
    import rosbags.typesys as _  # noqa
    from rosbags.rosbag1.writer import Writer  # noqa
except Exception:
    pass

STORE = get_typestore(Stores.ROS1_NOETIC)
IMG_CLASS = STORE.types["sensor_msgs/msg/Image"]
IMU_CLASS = STORE.types["sensor_msgs/msg/Imu"]
Time_ = STORE.types["builtin_interfaces/msg/Time"]
Hdr_ = STORE.types["std_msgs/msg/Header"]

# ---------- 加载相机采集类(复用单相机标定脚本里的相机封装) ----------
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from capture_hikvision import HikVisionCam          # noqa: E402
from capture_mindvision_sdk import MindVisionCam    # noqa: E402


VEC3 = STORE.types["geometry_msgs/msg/Vector3"]
QUAT = STORE.types["geometry_msgs/msg/Quaternion"]

_NEG_COV = np.array([-1.0] + [0.0] * 8, dtype=np.float64)   # 未提供姿态
_ZERO9 = np.zeros(9, dtype=np.float64)                       # 未提供协方差


def mono_ts():
    """统一单调时钟(ns), 相机与 IMU 共用同一基准。"""
    return time.monotonic_ns()


def build_image_msg(ts_ns, frame_id, gray):
    h, w = gray.shape
    return IMG_CLASS(
        header=Hdr_(seq=0, stamp=Time_(sec=int(ts_ns // 1_000_000_000),
                                       nanosec=int(ts_ns % 1_000_000_000)),
                    frame_id=frame_id),
        height=h, width=w, encoding="mono8", is_bigendian=0, step=w,
        data=gray.reshape(-1),
    )


def build_imu_msg(ts_ns, ax, ay, az, gx, gy, gz, frame_id="imu"):
    """构造 sensor_msgs/Imu。未提供姿态 → orientation 置恒等 + 协方差 -1。"""
    return IMU_CLASS(
        header=Hdr_(seq=0, stamp=Time_(sec=int(ts_ns // 1_000_000_000),
                                       nanosec=int(ts_ns % 1_000_000_000)),
                    frame_id=frame_id),
        orientation=QUAT(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=_NEG_COV,
        angular_velocity=VEC3(x=gx, y=gy, z=gz),
        angular_velocity_covariance=_ZERO9,
        linear_acceleration=VEC3(x=ax, y=ay, z=az),
        linear_acceleration_covariance=_ZERO9,
    )


def parse_imu_cboard(buf):
    """解析大疆 C 板二进制帧(帧头 A5 00 90, 64字节/帧)。
    帧布局: A5 00 90 | quat(w,x,y,z) float4 | gyro(x,y,z) float3 | acc(x,y,z) float3 | 19零 | crc16
    返回 (最新样本, 剩余缓冲) 或 (None, buf)。单位已为 SI: acc m/s², gyro rad/s。"""
    HEAD = b"\xa5\x00\x90"
    FRAME = 64
    idx = buf.find(HEAD)
    if idx < 0:
        # 没找到帧头, 只保留末尾 len-1, 防止帧头跨块
        return None, buf[-len(HEAD) + 1:]
    if len(buf) - idx < FRAME:
        return None, buf[idx:]  # 等整帧
    if idx > 0:
        buf = buf[idx:]  # 丢弃帧头前的垃圾
    frame = buf[:FRAME]
    vals = struct.unpack("<10f", frame[3:43])
    gyro = tuple(vals[4:7])     # (gx,gy,gz) rad/s
    acc = tuple(vals[7:10])     # (ax,ay,az) m/s^2
    return acc + gyro, buf[FRAME:]


def parse_imu_line(line, mode):
    """按模式解析串口一行 → (ax,ay,az,gx,gy,gz) 或 None。"""
    line = line.strip()
    if not line:
        return None
    if mode == "text":
        try:
            vals = [float(t) for t in line.replace(",", " ").split()]
        except ValueError:
            return None
    elif mode == "nmea":
        # 期望以自定义前缀跳过头, 例如 PIMU,ax,ay,az,gx,gy,gz
        if line.startswith("PIMU,"):
            try:
                vals = [float(t) for t in line.split(",")[1:7]]
            except (ValueError, IndexError):
                return None
        else:
            return None
    else:
        return None
    if len(vals) < 6:
        return None
    return tuple(vals[:6])


def open_camera(camera, exposure, index):
    if camera == "hikvision":
        cam = HikVisionCam(index=index, exposure_ms=exposure)
    else:
        cam = MindVisionCam(exposure_ms=exposure)
    print(f"[1/3] 打开相机({camera}) ...")
    cam.open()
    return cam


# ===== ethz_apriltag2 实时检测叠加 (--display, 与 basalt 相同的检测器) =====
_apriltag2_lib = None
_apriltag2_det = None


def _load_apriltag2():
    global _apriltag2_lib, _apriltag2_det
    if _apriltag2_lib is None:
        so = os.path.join(HERE, "libapriltag2.so")
        _apriltag2_lib = ctypes.CDLL(so)
        _apriltag2_lib.apriltag2_create.restype = ctypes.c_void_p
        _apriltag2_lib.apriltag2_destroy.argtypes = [ctypes.c_void_p]
        _apriltag2_lib.apriltag2_detect.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int, ctypes.c_int, ctypes.c_int,
            ctypes.POINTER(ctypes.c_int),
            ctypes.POINTER(ctypes.c_float),
            ctypes.c_int,
        ]
        _apriltag2_lib.apriltag2_detect.restype = ctypes.c_int
        _apriltag2_det = _apriltag2_lib.apriltag2_create()
    return _apriltag2_lib


def ethz_detect(gray, num_tags=36, max_tags=128):
    """返回 (ids, corners), corners 每 tag 8 个 float:
    [blx,bly, brx,bry, trx,try, tlx,tly] (与 basalt 角点顺序一致)。"""
    lib = _load_apriltag2()
    gray = np.ascontiguousarray(gray)
    h, w = gray.shape
    ptr = gray.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    ids = (ctypes.c_int * max_tags)()
    corn = (ctypes.c_float * (max_tags * 8))()
    n = lib.apriltag2_detect(_apriltag2_det, ptr, w, h, num_tags,
                             ids, corn, max_tags)
    if n <= 0:
        return [], []
    return list(ids[:n]), list(corn[:n * 8])


def draw_ethz_overlay(overlay, ids, corners, rows, cols,
                      tag_size=1.0, tag_spacing=0.3, color=(0, 255, 0)):
    """画检测到的 tag 角点+边框; 提供 rows/cols 且检测≥4 时用单应性外推整板。"""
    n = len(ids)
    if n == 0:
        return 0
    tag_pts = []
    for i in range(n):
        pts = np.array([
            [corners[i * 8 + 0], corners[i * 8 + 1]],
            [corners[i * 8 + 2], corners[i * 8 + 3]],
            [corners[i * 8 + 4], corners[i * 8 + 5]],
            [corners[i * 8 + 6], corners[i * 8 + 7]],
        ], np.float32)
        tag_pts.append(pts)
        p = pts.astype(np.int32)
        cv2.polylines(overlay, [p], True, color, 2)
        for q in p:
            cv2.circle(overlay, tuple(q), 5, color, 2)
    if n < 4 or not rows or not cols:
        return n
    p2, p3 = [], []
    max_id = rows * cols
    for i, tid in enumerate(ids):
        if tid < 0 or tid >= max_id:
            continue
        gx, gy = tid % cols, tid // cols
        ox, oy = gx * tag_size * (1.0 + tag_spacing), gy * tag_size * (1.0 + tag_spacing)
        pts = tag_pts[i]
        p2.extend([pts[0], pts[1], pts[2], pts[3]])
        p3.extend([(ox, oy), (ox + tag_size, oy),
                   (ox + tag_size, oy + tag_size), (ox, oy + tag_size)])
    if len(p2) < 8:
        return n
    H, inliers = cv2.findHomography(np.array(p3, np.float64),
                                    np.array(p2, np.float64), cv2.RANSAC)
    if H is None or inliers is None or int(inliers.sum()) < 8:
        return n
    all3 = []
    for gy in range(rows):
        for gx in range(cols):
            ox, oy = gx * tag_size * (1.0 + tag_spacing), gy * tag_size * (1.0 + tag_spacing)
            all3.append((ox, oy))
            all3.append((ox + tag_size, oy))
            all3.append((ox + tag_size, oy + tag_size))
            all3.append((ox, oy + tag_size))
    if not all3:
        return n
    all2 = cv2.perspectiveTransform(
        np.array(all3, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)
    for p in all2:
        cv2.circle(overlay, tuple(p.astype(int)), 4, color, 1)
    idx = 0
    for _gy in range(rows):
        for _gx in range(cols):
            sq = all2[idx:idx + 4].astype(np.int32)
            cv2.polylines(overlay, [sq], True, color, 1)
            idx += 4
    return n


def resize_like_runtime(gray, out_size):
    """把图像缩放到运行时的输出尺寸。

    与 src/hw_io/camera/image_utils.h 的 resizeFrameForOutput 保持一致:
    当前 mindvision.yaml 里 crop.enabled=false、resize=1280x960, 即整幅原图
    (1280x1024) 直接 cv::resize 到 1280x960。标定 bag 里的图像必须经过
    同一变换, basalt 解出的内参才能直接写进配置使用, 无需再换算。
    """
    if out_size is None:
        return gray
    if (gray.shape[1], gray.shape[0]) == out_size:
        return gray
    return cv2.resize(gray, out_size, interpolation=cv2.INTER_LINEAR)


class ImuReader(threading.Thread):
    """后台线程: 高频轮询串口, 按采样率回填每个样本的时间戳。

    串口数据是成批到达的: 主线程被相机取图(cam.grab() 阻塞)挡住几十毫秒,
    这段时间串口缓冲区会攒下几十帧 IMU。若在"读取时刻"给整批打同一个时间戳,
    IMU 时间轴会被压扁(实测 23 个样本挤进 0.2ms, 而真实跨度 116ms),
    basalt 的 IMU 因子随之完全失效, 外参平移会发散成天文数字。

    这里把读串口放进独立线程(轮询间隔 ~0.5ms), 每次只拿到 1~2 个样本,
    再在批内按 1/rate 回填时间戳, 并用全局 _next_ts 保证跨批次单调递增,
    避免相邻批次时间戳重叠导致间隔不均(实测 9.8% 间隔 <1ms)。
    """

    def __init__(self, ser, imu_fmt, rate_hz, out_queue):
        super().__init__(daemon=True)
        self._ser = ser
        self._fmt = imu_fmt
        self._period_ns = int(round(1e9 / rate_hz))
        self._queue = out_queue
        # 注意: 不能叫 _stop, 会覆盖 threading.Thread 内部的 _stop() 导致 join 报错
        self._stop_event = threading.Event()
        self._buf_cb = b""
        self._buf = ""
        self.count = 0
        self._next_ts = None  # 下一个样本应分配的时间戳, 保证全局单调递增

    def stop(self):
        self._stop_event.set()

    def _read_batch(self):
        """读一次串口, 返回本次解析出的样本列表(按时间先后排列)。"""
        available = self._ser.in_waiting if hasattr(self._ser, "in_waiting") else 0
        if available <= 0:
            return []
        data = self._ser.read(available)
        if not data:
            return []

        if self._fmt == "cboard":
            self._buf_cb += data
            samples = []
            while True:
                parsed, self._buf_cb = parse_imu_cboard(self._buf_cb)
                if parsed is None:
                    break
                samples.append(parsed)
            return samples

        self._buf += data.decode(errors="replace")
        lines = self._buf.split("\n")
        # 最后一段可能是不完整行, 留到下次
        self._buf = lines.pop() if lines else ""
        samples = []
        for ln in lines:
            parsed = parse_imu_line(ln, self._fmt)
            if parsed is not None:
                samples.append(parsed)
        return samples

    def run(self):
        while not self._stop_event.is_set():
            samples = self._read_batch()
            if not samples:
                time.sleep(0.0005)
                continue
            now = mono_ts()
            n = len(samples)
            # 末样本对齐读取时刻 now, 批次内按 period 均匀回填
            first_ts = now - (n - 1) * self._period_ns
            # 跨批次保证单调递增: 不早于上一批分配的下一个时间戳
            if self._next_ts is not None:
                first_ts = max(first_ts, self._next_ts)
            for i, sample in enumerate(samples):
                ts = first_ts + i * self._period_ns
                self._queue.put((ts, sample))
            self._next_ts = first_ts + n * self._period_ns
            self.count += n


def parse_size(text):
    """解析 WxH 尺寸, 如 1280x960。"""
    try:
        width, height = text.lower().split("x")
        width, height = int(width), int(height)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"尺寸格式应为 WxH, 例如 1280x960 (收到: {text})")
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError(f"尺寸必须为正数 (收到: {text})")
    return width, height


def main():
    ap = argparse.ArgumentParser(
        description="相机 + 串口 IMU 同步采集 → basalt calib bag")
    ap.add_argument("--out", required=True, help="输出 .bag 路径")
    ap.add_argument("--camera", required=True,
                    choices=["hikvision", "mindvision"])
    ap.add_argument("--index", type=int, default=0,
                    help="海康设备序号(仅海康)")
    ap.add_argument("--exposure", type=float, default=None,
                    help="曝光 ms")
    ap.add_argument("--port", required=True, help="IMU 串口 如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--imu-fmt", default="text",
                    choices=["text", "nmea", "cboard"],
                    help="IMU 串口协议: text=ax,ay,az,gx,gy,gz 文本行; "
                         "nmea=PIMU,...; cboard=大疆C板二进制帧(A5 00 90)")
    ap.add_argument("--imu-rate", type=float, default=200.0,
                    help="IMU 采样率 Hz, 用于批内回填时间戳(大疆C板默认200)")
    ap.add_argument("--out-size", type=parse_size, default=(1280, 960),
                    help="写入 bag 的图像尺寸 WxH, 默认 1280x960 "
                         "(与运行时 mindvision.yaml 的 preprocess.resize 一致)")
    ap.add_argument("--duration", type=float, default=90,
                    help="采集时长秒")
    ap.add_argument("--fps", type=float, default=5,
                    help="图像写入帧率上限(默认5fps, 避免bag过大)")
    ap.add_argument("--cam-topic", default="/cam0/image_raw")
    ap.add_argument("--imu-topic", default="/imu/data_raw")
    ap.add_argument("--display", action="store_true",
                    help="实时预览窗口(含 AprilTag 检测叠加), q/Esc 或 Ctrl+C 结束")
    ap.add_argument("--grid", type=int, default=36,
                    help="标定板 tag 总数(=cols*rows), 用于过滤误检, 默认 36")
    ap.add_argument("--cols", type=int, default=None,
                    help="标定板列数(配合 --rows 画整板轮廓)")
    ap.add_argument("--rows", type=int, default=None,
                    help="标定板行数")
    args = ap.parse_args()

    # 1) 先开串口(快速失败, 避免占用相机后再报错泄漏句柄)
    print("[1/3] 打开串口 IMU ...")
    ports = None
    try:
        from serial.tools import list_ports
        ports = [p.device for p in list_ports.comports()]
    except Exception:
        ports = None
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as e:
        hint = ", ".join(ports) if ports else "(没有检测到任何串口设备)"
        raise SystemExit(
            f"无法打开串口 {args.port}: {e}\n当前检测到的串口: {hint}\n"
            f"请把 --port 换成你的主控 IMU 实际串口(如 /dev/ttyUSB0 /dev/ttyACM0)。")
    print(f"  串口: {args.port} @ {args.baud}, 格式: {args.imu_fmt}")

    # 2) 打开相机; 相机/串口/bag 全程保证关闭, 异常也释放句柄
    cam = None
    writer = None
    conn_img = conn_imu = None
    imu_queue = None
    imu_reader = None
    n_img = 0
    n_imu = 0
    try:
        cam = open_camera(args.camera, args.exposure, args.index)

        out_path = os.path.expanduser(args.out)
        if os.path.exists(out_path):
            print(f"  删除旧 bag {out_path}")
            os.remove(out_path)
        writer = Writer(out_path)
        writer.open()
        conn_img = writer.add_connection(
            args.cam_topic, IMG_CLASS.__msgtype__,
            typestore=STORE)
        conn_imu = writer.add_connection(
            args.imu_topic, IMU_CLASS.__msgtype__,
            typestore=STORE)

        print("[3/3] 开始采集(自动, 相机+IMU 同步入 bag) ...")
        print(f"  图像尺寸: {args.out_size[0]}x{args.out_size[1]}"
              f" (对齐运行时 preprocess.resize)")
        print(f"  IMU: {args.imu_fmt} @ {args.imu_rate:.0f}Hz (批内回填时间戳)")
        print("  按 Ctrl+C 提前结束; 期间让载具做大幅度六自由度运动!")

        # IMU 放独立线程高频读取: 主线程会被 cam.grab() 阻塞几十毫秒,
        # 若在同一循环里读串口, 整批样本会共用同一个时间戳导致 IMU 时间轴被压扁。
        imu_queue = queue.Queue()
        imu_reader = ImuReader(ser, args.imu_fmt, args.imu_rate, imu_queue)
        imu_reader.start()

        t0 = time.monotonic()
        next_img_t = 0.0
        while True:
            # --- 写入 IMU 线程产出的样本 ---
            while True:
                try:
                    ts, sample = imu_queue.get_nowait()
                except queue.Empty:
                    break
                msg = build_imu_msg(ts, *sample)
                writer.write(conn_imu, ts, STORE.serialize_ros1(
                    msg, IMU_CLASS.__msgtype__))
                n_imu += 1

            # --- 读相机(按 fps 限速) ---
            now = time.monotonic()
            if now >= next_img_t:
                got = cam.grab()
                if got is not None:
                    bgr = got[0]
                    ts = mono_ts()
                    # 按通道数判断: 1通道已是灰度, 3通道BGR才转灰度
                    if bgr.ndim == 2 or bgr.shape[2] == 1:
                        gray = np.squeeze(bgr)
                    else:
                        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    # 与运行时 resizeFrameForOutput 走同一变换, 内参可直接使用
                    gray = resize_like_runtime(gray, args.out_size)
                    writer.write(conn_img, ts, STORE.serialize_ros1(
                        build_image_msg(ts, args.cam_topic, gray),
                        IMG_CLASS.__msgtype__))
                    n_img += 1
                    if args.display:
                        ids, corners = ethz_detect(gray, num_tags=args.grid)
                        overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                        draw_ethz_overlay(overlay, ids, corners,
                                          args.rows, args.cols)
                        show = overlay if overlay.shape[1] <= 900 else \
                            cv2.resize(overlay, (900, int(
                                900.0 * overlay.shape[0] / overlay.shape[1])))
                        cv2.imshow("capture_cam_imu", show)
                        k = cv2.waitKey(1) & 0xFF
                        if k in (ord("q"), 27):
                            print("  检测到 q/Esc, 停止采集")
                            break
                    if n_img % 5 == 1:
                        print(f"  img={n_img} imu={n_imu} @ {time.monotonic()-t0:.0f}s")
                # 无论是否取到帧都推进限速, 避免满速空转
                next_img_t = now + 1.0 / max(args.fps, 0.1)

            if time.monotonic() - t0 >= args.duration:
                print("  达到时长, 正常结束")
                break
    except KeyboardInterrupt:
        print("\n  收到 Ctrl+C, 停止采集")
    except Exception as e:
        print(f"\n错误: {e}")
        raise
    finally:
        if imu_reader is not None:
            imu_reader.stop()
            imu_reader.join(timeout=1.0)
        if writer is not None:
            # 把 IMU 线程残留的样本补写完再关 bag, 避免丢尾部数据
            if imu_queue is not None and conn_imu is not None:
                while True:
                    try:
                        ts, sample = imu_queue.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        msg = build_imu_msg(ts, *sample)
                        writer.write(conn_imu, ts, STORE.serialize_ros1(
                            msg, IMU_CLASS.__msgtype__))
                        n_imu += 1
                    except Exception:
                        break
            try:
                writer.close()
            except Exception:
                pass
        if cam is not None:
            try:
                cam.close()
            except Exception:
                pass
        try:
            ser.close()
        except Exception:
            pass
        if args.display:
            cv2.destroyAllWindows()
        if _apriltag2_det is not None and _apriltag2_lib is not None:
            try:
                _apriltag2_lib.apriltag2_destroy(_apriltag2_det)
            except Exception:
                pass

    print(f"\n完成: 图像 {n_img} 帧, IMU {n_imu} 条 -> {out_path}")
    if n_img < 30 or n_imu < 3000:
        print("警告: 采集量偏少。建议 ≥30 图像帧 且 IMU 采样充足。")


if __name__ == "__main__":
    main()