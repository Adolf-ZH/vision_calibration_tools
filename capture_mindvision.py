#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""UVC 相机 —— AprilGrid 标定【录制】脚本 (OpenCV VideoCapture 版)

适用:
    走标准 UVC 协议、能在 /dev/videoN 里看到的相机(例如笔记本内置摄像头、
    USB 免驱摄像头)。工业相机(迈德威视/海康)不是 UVC 设备, 请用
    capture_mindvision_sdk.py / capture_hikvision.py。

功能:
    1. 用 OpenCV 打开 UVC 相机采集图像
    2. 预览常开(带整板轮廓叠加), 按 [R] 开始录制 / 再按 [R] 停止录制,
       录制期间把每一帧连续写入 rosbag(topic /cam0/image_raw, mono8)
    3. 产物是"原始录制 bag", 再用 extract_calib_frames.py 离线自动抽帧,
       生成最终标定 bag 交给 basalt_calibrate (--dataset-type bag)

用法:
    python3 capture_mindvision.py \
        --out ~/calib_data/uvc_raw.bag \
        --width 1280 --height 1024 --device 0

操作:
    R        开始 / 停止录制(可反复)
    q / Esc  退出(自动关闭 bag)
"""

import argparse
import time

import cv2

import calib_common as cc


class UvcCam:
    """OpenCV VideoCapture 适配器: grab() 返回 (BGR 帧, 时间戳ns)。"""

    def __init__(self, cap):
        self.cap = cap

    def grab(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame, time.time_ns()

    def close(self):
        try:
            self.cap.release()
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description="UVC 相机录制 + rosbag 打包")
    parser.add_argument("--device", type=int, default=0, help="/dev/videoN 编号 (默认 0)")
    parser.add_argument("--width", type=int, default=1280, help="采集宽度 (默认 1280)")
    parser.add_argument("--height", type=int, default=1024, help="采集高度 (默认 1024)")
    parser.add_argument("--fps", type=float, default=30.0, help="采集帧率 (默认 30)")
    parser.add_argument("--out", required=True, help="输出原始录制 .bag 路径")
    parser.add_argument("--topic", default=cc.DEFAULT_TOPIC, help="图像话题")
    cc.add_board_args(parser)
    args = parser.parse_args()
    cc.check_board_args(args)

    print(f"[1/3] 打开相机 /dev/video{args.device} @ {args.width}x{args.height} ...")
    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头 /dev/video{args.device}，请检查设备编号。")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"    实际分辨率: {actual_w}x{actual_h} "
          f"-> 写入 {args.out_size[0]}x{args.out_size[1]} (对齐运行时 preprocess.resize)")

    print("[2/3] 创建 rosbag ...")
    try:
        cc.run_capture_session(
            UvcCam(cap),
            out_path=args.out, topic=args.topic, out_size=args.out_size,
            cols=args.cols, rows=args.rows, grid=args.grid,
            window="UVC Capture")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
