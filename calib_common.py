#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""相机标定采集公共模块。

被 capture_hikvision.py / capture_mindvision_sdk.py / capture_mindvision.py /
extract_calib_frames.py 共用, 提供:

  1. rosbag 打包 (sensor_msgs/Image, mono8)
  2. 与运行时 resizeFrameForOutput 完全一致的尺寸对齐
  3. ethz_apriltag2 检测 + 整块标定板轮廓叠加 (与 basalt 标定用的检测器相同)
  4. 终端单键读取 (cbreak, 不用抢占预览窗口焦点)
  5. 采集主循环: 预览常开, 按 R 开始/停止录制, 按 q/Esc 退出

板面坐标系与 basalt AprilGrid 一致:
    tag_id = cols * gy + gx
    每个 tag 的 4 个角点顺序 = [左下, 右下, 右上, 左上] (与检测器输出一致)
    角点在板面上的位置 = [(0,0), (s,0), (s,s), (0,s)], s = tag_size
"""

import argparse
import ctypes
import os
import select
import sys
import termios
import time
import tty

import cv2
import numpy as np

from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

STORE = get_typestore(Stores.ROS1_NOETIC)
IMG_CLASS = STORE.types["sensor_msgs/msg/Image"]
Time_ = STORE.types["builtin_interfaces/msg/Time"]
Hdr_ = STORE.types["std_msgs/msg/Header"]

# 本项目实际标定板规格 (AprilGrid 11x8, 单格 83mm, 间隙 = 边长 30%)
DEFAULT_COLS = 11
DEFAULT_ROWS = 8
DEFAULT_GRID = DEFAULT_COLS * DEFAULT_ROWS          # 88
DEFAULT_OUT_SIZE = (1280, 960)                      # 与运行时 preprocess.resize 一致

# 叠加用的板面几何, 只用比值, 取任意单位
TAG_SIZE = 1.0
TAG_SPACING = 0.3

DEFAULT_TOPIC = "/cam0/image_raw"


# --------------------------------------------------------------------------
# 消息打包 / 尺寸对齐
# --------------------------------------------------------------------------

def build_image_msg(ts_ns, frame_id, gray):
    """把灰度帧封装成 sensor_msgs/Image (mono8)。"""
    h, w = gray.shape
    return IMG_CLASS(
        header=Hdr_(seq=0, stamp=Time_(sec=int(ts_ns // 1_000_000_000),
                                       nanosec=int(ts_ns % 1_000_000_000)),
                    frame_id=frame_id),
        height=h, width=w, encoding="mono8", is_bigendian=0, step=w,
        data=np.ascontiguousarray(gray).reshape(-1),
    )


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


def to_gray(image):
    """BGR / 灰度 -> 单通道灰度。"""
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def resize_like_runtime(image, out_size):
    """把图像缩放到运行时的输出尺寸。

    与 src/hw_io/camera/image_utils.h 的 resizeFrameForOutput 保持一致:
    当前配置 crop.enabled=false、resize=1280x960, 即整幅原图直接 cv::resize。
    标定 bag 里的图像必须经过同一变换, basalt 解出的内参才能直接写进配置。
    """
    if out_size is None:
        return image
    out_size = tuple(out_size)
    if (image.shape[1], image.shape[0]) == out_size:
        return image
    return cv2.resize(image, out_size, interpolation=cv2.INTER_LINEAR)


# --------------------------------------------------------------------------
# ethz_apriltag2 检测 (与 basalt 完全相同的检测器)
# --------------------------------------------------------------------------

_apriltag2_lib = None


def _load_apriltag2():
    global _apriltag2_lib
    if _apriltag2_lib is not None:
        return _apriltag2_lib
    so = os.path.join(os.path.dirname(os.path.abspath(__file__)), "libapriltag2.so")
    if not os.path.exists(so):
        raise SystemExit(
            f"缺少检测库 {so}\n"
            "它由 apriltag2_bridge.cpp 编译得到, 用于实时叠加与离线抽帧。")
    lib = ctypes.CDLL(so)
    lib.apriltag2_create.restype = ctypes.c_void_p
    lib.apriltag2_destroy.argtypes = [ctypes.c_void_p]
    lib.apriltag2_detect.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int, ctypes.c_int, ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_float),
        ctypes.c_int,
    ]
    lib.apriltag2_detect.restype = ctypes.c_int
    _apriltag2_lib = lib
    return lib


def ethz_detect(gray, num_tags=DEFAULT_GRID, max_tags=256):
    """检测灰度图中的 AprilTag。

    返回 (ids, corners)。corners 每 tag 8 个 float, 角点顺序
    [blx,bly, brx,bry, trx,try, tlx,tly] (与 basalt 角点顺序一致)。
    """
    lib = _load_apriltag2()
    gray = np.ascontiguousarray(gray)
    h, w = gray.shape
    ptr = gray.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
    det = lib.apriltag2_create()
    ids = (ctypes.c_int * max_tags)()
    corn = (ctypes.c_float * (max_tags * 8))()
    n = lib.apriltag2_detect(det, ptr, w, h, num_tags, ids, corn, max_tags)
    lib.apriltag2_destroy(det)
    if n <= 0:
        return [], []
    return list(ids[:n]), list(corn[:n * 8])


# --------------------------------------------------------------------------
# 板面几何
# --------------------------------------------------------------------------

def board_homography(ids, corners, rows, cols,
                     tag_size=TAG_SIZE, tag_spacing=TAG_SPACING):
    """由检测到的 tag 反解「板面坐标 -> 图像」的单应矩阵 H。

    返回 (H, inliers); 点太少或拟合失败返回 (None, 0)。
    """
    pitch = tag_size * (1.0 + tag_spacing)
    max_id = rows * cols
    p2, p3 = [], []
    for i, tid in enumerate(ids):
        tid = int(tid)
        if tid < 0 or tid >= max_id:
            continue
        gx, gy = tid % cols, tid // cols
        ox, oy = gx * pitch, gy * pitch
        c = corners[i * 8:i * 8 + 8]
        p2.extend([(c[0], c[1]), (c[2], c[3]), (c[4], c[5]), (c[6], c[7])])
        p3.extend([(ox, oy), (ox + tag_size, oy),
                   (ox + tag_size, oy + tag_size), (ox, oy + tag_size)])
    if len(p2) < 8:
        return None, 0
    H, inl = cv2.findHomography(np.array(p3, np.float64),
                                np.array(p2, np.float64), cv2.RANSAC)
    if H is None or inl is None:
        return None, 0
    return H, int(inl.sum())


def board_outline(H, rows, cols, tag_size=TAG_SIZE, tag_spacing=TAG_SPACING):
    """整块标定板的四角在图像中的位置, 顺序 [左下, 右下, 右上, 左上]。"""
    pitch = tag_size * (1.0 + tag_spacing)
    wb = (cols - 1) * pitch + tag_size
    hb = (rows - 1) * pitch + tag_size
    corners_3d = np.array([[0.0, 0.0], [wb, 0.0], [wb, hb], [0.0, hb]], np.float64)
    return cv2.perspectiveTransform(corners_3d.reshape(-1, 1, 2), H).reshape(-1, 2)


def draw_ethz_overlay(overlay, ids, corners, rows, cols,
                      tag_size=TAG_SIZE, tag_spacing=TAG_SPACING,
                      color=(0, 255, 0), faint=(0, 200, 0)):
    """画检测到的 tag(方框 + 角点); tag 足够多时用单应性外推整块板网格。

    外推网格只在「检测到足够多 tag」时才画, 且外推点会被裁剪到图像范围内,
    避免板子部分出框时单应外推把网格画到画面外造成误导。
    """
    n = len(ids)
    if n == 0:
        return 0
    h, w = overlay.shape[:2]
    for i in range(n):
        pts = np.array([
            [corners[i * 8 + 0], corners[i * 8 + 1]],
            [corners[i * 8 + 2], corners[i * 8 + 3]],
            [corners[i * 8 + 4], corners[i * 8 + 5]],
            [corners[i * 8 + 6], corners[i * 8 + 7]],
        ], np.int32)
        cv2.polylines(overlay, [pts], True, color, 2)
        for q in pts:
            cv2.circle(overlay, (int(q[0]), int(q[1])), 5, color, 2)
    grid = rows * cols
    # 外推网格门槛: 至少检测到 1/3 的 tag 且不少于 12 个, 否则单应矩阵不可靠
    min_for_grid = max(12, grid // 3)
    if n < min_for_grid:
        return n
    H, inl = board_homography(ids, corners, rows, cols, tag_size, tag_spacing)
    if H is None or inl < max(8, n // 2):
        return n
    pitch = tag_size * (1.0 + tag_spacing)
    all3 = []
    for gy in range(rows):
        for gx in range(cols):
            ox, oy = gx * pitch, gy * pitch
            for dx, dy in [(0, 0), (tag_size, 0),
                           (tag_size, tag_size), (0, tag_size)]:
                all3.append((ox + dx, oy + dy))
    all2 = cv2.perspectiveTransform(
        np.array(all3, np.float64).reshape(-1, 1, 2), H).reshape(-1, 2)
    # 裁剪外推点到图像范围内 (留 2px 边距), 超出的标记为 NaN
    margin = 2
    in_bounds = ((all2[:, 0] >= -margin) & (all2[:, 0] <= w - 1 + margin)
                 & (all2[:, 1] >= -margin) & (all2[:, 1] <= h - 1 + margin))
    for i, p in enumerate(all2):
        if in_bounds[i]:
            cv2.circle(overlay, (int(np.clip(p[0], 0, w - 1)),
                                 int(np.clip(p[1], 0, h - 1))), 4, faint, 1)
    # 画网格框时只画四条顶点都在范围内的格子
    for i in range(rows * cols):
        cell = all2[i * 4:i * 4 + 4]
        if np.all(in_bounds[i * 4:i * 4 + 4]):
            cv2.polylines(overlay, [cell.astype(np.int32)], True, faint, 1)
    return n


# --------------------------------------------------------------------------
# 终端单键读取
# --------------------------------------------------------------------------

class RawTerminal:
    """把终端切到 cbreak 以便单键读取; 非 tty 时静默降级(只靠预览窗按键)。"""

    def __init__(self):
        self.fd = None
        self.old = None
        try:
            fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
            self.fd = fd
        except Exception:
            self.fd = None
            self.old = None

    def read(self):
        """非阻塞读终端按键, 返回一列单字节。"""
        if self.fd is None:
            return []
        out = []
        while select.select([self.fd], [], [], 0)[0]:
            try:
                ch = os.read(self.fd, 1)
            except (OSError, ValueError):
                break
            if not ch:
                break
            out.append(ch)
        return out

    def close(self):
        if self.fd is not None and self.old is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            except Exception:
                pass
            self.fd = None


# --------------------------------------------------------------------------
# 采集主循环 (预览常开 + 按 R 录制)
# --------------------------------------------------------------------------

def add_board_args(parser):
    """给采集脚本挂上板子规格参数(默认值 = 本项目实际板子 11x8=88)。"""
    parser.add_argument("--cols", type=int, default=DEFAULT_COLS,
                        help=f"标定板横向 tag 数 tagCols, 默认 {DEFAULT_COLS}")
    parser.add_argument("--rows", type=int, default=DEFAULT_ROWS,
                        help=f"标定板纵向 tag 数 tagRows, 默认 {DEFAULT_ROWS}")
    parser.add_argument("--grid", type=int, default=DEFAULT_GRID,
                        help=f"标定板 tag 总数 tagCols*tagRows, 默认 {DEFAULT_GRID}")
    parser.add_argument("--out-size", type=parse_size, default=DEFAULT_OUT_SIZE,
                        help="写入 bag 的图像尺寸 WxH, 默认 1280x960 "
                             "(与运行时 preprocess.resize 一致)")


def check_board_args(args):
    if args.grid != args.cols * args.rows:
        raise SystemExit(
            f"--grid({args.grid}) 必须等于 --cols({args.cols}) x --rows({args.rows}) "
            f"= {args.cols * args.rows}。请按实际板子填写。")


def run_capture_session(cam, *, out_path, topic=DEFAULT_TOPIC,
                        out_size=DEFAULT_OUT_SIZE, cols=DEFAULT_COLS,
                        rows=DEFAULT_ROWS, grid=None, frame_id="0",
                        window="Calibration Capture",
                        tag_size=TAG_SIZE, tag_spacing=TAG_SPACING):
    """预览 + 录制主循环。

    - 预览常开, 带整板轮廓叠加 (所见即所存);
    - 按 R 开始录制 / 再按 R 停止录制 (可反复, 累积到同一个 bag);
    - 按 q / Esc 退出, Ctrl+C 亦会正常关闭 bag;
    - 录制时把**每一帧**连续写入 bag (原始录制, 之后再离线抽帧)。

    返回 (saved_frames, recorded_seconds)。
    """
    grid = grid or (cols * rows)
    out_path = os.path.abspath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        print(f"    删除旧的 {out_path}")
        os.remove(out_path)

    writer = Writer(out_path)
    writer.open()
    conn = writer.add_connection(topic, "sensor_msgs/msg/Image", typestore=STORE)

    term = RawTerminal()
    print("[3/3] 预览已开启: 按 [R] 开始/停止录制, 按 [q/Esc] 退出")
    print(f"      原始录制产物 -> {out_path}")
    print(f"      板子规格: cols={cols} rows={rows} grid={grid} "
          f"out-size={out_size[0]}x{out_size[1]}")
    if term.fd is None:
        print("      注意: 终端不是 tty, R/q 只能在预览窗口里按")

    recording = False
    saved = 0
    rec_frames = 0
    rec_bytes = 0
    rec_start = 0.0
    rec_total = 0.0
    grab_fail = 0
    last_n = -1
    win = window

    try:
        while True:
            res = cam.grab()
            if res is None:
                grab_fail += 1
                if grab_fail % 200 == 0:
                    print(f"  取帧超时/丢帧 x{grab_fail} ...")
                continue
            grab_fail = 0
            img, ts_ns = res
            gray = resize_like_runtime(to_gray(img), out_size)
            overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

            ids, corn = ethz_detect(gray, num_tags=grid)
            n = len(ids)
            if n:
                draw_ethz_overlay(overlay, ids, corn, rows, cols, tag_size, tag_spacing)
                if n != last_n:
                    last_n = n
                    print(f"  [detect] tags={n}/{grid}")

            # 状态叠加先画, 保证预览里看到的就是正在写进 bag 的帧
            h, w = gray.shape
            if recording:
                cv2.rectangle(overlay, (0, 0), (w - 1, h - 1), (0, 0, 255), 6)
                txt = (f"REC {time.time() - rec_start:5.1f}s  frames={rec_frames}  "
                       f"tags={n}/{grid}")
                col = (0, 0, 255)
            else:
                txt = f"READY  press R to record  |  saved={saved}  tags={n}/{grid}"
                col = (0, 255, 0) if n >= 8 else (0, 200, 255)
            cv2.putText(overlay, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
            if n == 0:
                cv2.putText(overlay, "NO TAGS: move board closer / increase exposure",
                            (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            key = -1
            try:
                cv2.imshow(win, overlay)
                key = cv2.waitKey(1) & 0xFF
            except Exception:
                pass
            keys = term.read()

            quit_now = key in (ord("q"), 27) or any(c in (b"q", b"\x1b") for c in keys)
            toggle = key in (ord("r"), ord("R")) or any(c in (b"r", b"R") for c in keys)

            if toggle:
                recording = not recording
                if recording:
                    rec_start = time.time()
                    rec_frames = 0
                    rec_bytes = 0
                    print(f"  [REC] 开始录制 -> {out_path}")
                else:
                    rec_total += time.time() - rec_start
                    print(f"  [REC] 停止录制: 本次 {rec_frames} 帧 / "
                          f"{rec_bytes / 1e6:.1f} MB, 累计 {saved} 帧")
            if quit_now:
                print("收到退出指令。")
                break

            if recording:
                msg = build_image_msg(ts_ns, frame_id, gray)
                data = STORE.serialize_ros1(msg, "sensor_msgs/msg/Image")
                writer.write(conn, ts_ns, data)
                saved += 1
                rec_frames += 1
                rec_bytes += len(data)
                if rec_frames % 30 == 0:
                    print(f"  [REC] {rec_frames} 帧  {time.time() - rec_start:.1f}s  "
                          f"{rec_bytes / 1e6:.0f} MB")
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C, 正在保存并退出 ...")
    finally:
        if recording:
            rec_total += time.time() - rec_start
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        term.close()
        try:
            writer.close()
        except Exception:
            pass

    print(f"\n完成。原始录制 {saved} 帧 / {rec_total:.1f}s -> {out_path}")
    if saved == 0:
        print("提示: 一帧都没录到。重跑并按 [R] 开始录制。")
    else:
        print("下一步: 用 extract_calib_frames.py 从该 bag 自动抽帧, 生成标定用的 bag。")
    return saved, rec_total
