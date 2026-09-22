#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""迈德威视 (MindVision) 工业相机 —— AprilGrid 标定【录制】脚本(SDK 版)

背景:
    迈德威视工业相机(如 SUA133GC)不是 UVC 设备, 不会出现在 /dev/videoN,
    OpenCV 抓不到。本脚本通过厂商 SDK (libMVSDK.so) 采图, 并用 ctypes 调用。

功能:
    1. 用 MindVision SDK 打开相机并连续采图
    2. 预览常开(带整板轮廓叠加), 按 [R] 开始录制 / 再按 [R] 停止录制,
       录制期间把**每一帧**连续写入 rosbag(topic /cam0/image_raw, mono8)
    3. 产物是"原始录制 bag", 再用 extract_calib_frames.py 离线自动抽帧,
       生成最终标定 bag 交给 basalt_calibrate (--dataset-type bag)

用法:
    python3 capture_mindvision_sdk.py \
        --out ~/calib_data/mindvision_raw.bag \
        --exposure 30        # 曝光(毫秒, 可选, 默认自动)

操作:
    R        开始 / 停止录制(可反复)
    q / Esc  退出(自动关闭 bag)

分辨率:
    写入 bag 的图像默认缩放到 1280x960, 与运行时 mindvision.yaml 的
    preprocess(crop 关闭 + resize 1280x960) 完全一致, 因此 basalt 解出的内参
    可以直接填进配置使用, 无需再做尺寸换算。用 --out-size 可改。
"""

import argparse
import ctypes
import ctypes.util
import os
import time

import numpy as np

import calib_common as cc

CAMERA_MEDIA_TYPE_BGR8 = 0x02150000  # CAMERA_MEDIA_TYPE_BGR8


# ---------- MindVision SDK 结构体 ----------
class tSdkCameraDevInfo(ctypes.Structure):
    _fields_ = [
        ("acProductSeries", ctypes.c_char * 32),
        ("acProductName", ctypes.c_char * 32),
        ("acFriendlyName", ctypes.c_char * 32),
        ("acLinkName", ctypes.c_char * 32),
        ("acDriverVersion", ctypes.c_char * 32),
        ("acSensorType", ctypes.c_char * 32),
        ("acPortType", ctypes.c_char * 32),
        ("acSn", ctypes.c_char * 32),
        ("uInstance", ctypes.c_uint),
    ]


class tSdkFrameHead(ctypes.Structure):
    _fields_ = [
        ("uiMediaType", ctypes.c_uint),
        ("uBytes", ctypes.c_uint),
        ("iWidth", ctypes.c_int),
        ("iHeight", ctypes.c_int),
        ("iWidthZoomSw", ctypes.c_int),
        ("iHeightZoomSw", ctypes.c_int),
        ("bIsTrigger", ctypes.c_int),
        ("uiTimeStamp", ctypes.c_uint),
        ("uiExpTime", ctypes.c_uint),
        ("fAnalogGain", ctypes.c_float),
        ("iGamma", ctypes.c_int),
        ("iContrast", ctypes.c_int),
        ("iSaturation", ctypes.c_int),
        ("fRgain", ctypes.c_float),
        ("fGgain", ctypes.c_float),
        ("fBgain", ctypes.c_float),
    ]


def load_sdk():
    candidates = [
        os.environ.get("MVSDK_LIB", ""),
        "/usr/local/lib/libMVSDK.so",
        "/usr/lib/libMVSDK.so",
        "/home/adolf/桌面/Visionlearning/sp_vision_25/io/mindvision/lib/amd64/libMVSDK.so",
        ctypes.util.find_library("MVSDK") or "",
    ]
    for p in candidates:
        if p and os.path.exists(p):
            try:
                return ctypes.CDLL(p)
            except OSError:
                continue
    raise RuntimeError("找不到 libMVSDK.so，请设置 MVSDK_LIB 环境变量指向 SDK 路径")


SDK = load_sdk()
BYTE_P = ctypes.POINTER(ctypes.c_ubyte)


def setup(fn, *argtypes, restype=ctypes.c_int):
    f = getattr(SDK, fn)
    f.argtypes = argtypes
    f.restype = restype
    return f


CAMERA_SDK_INIT = setup("CameraSdkInit", ctypes.c_int)
CAMERA_ENUM = setup("CameraEnumerateDevice",
                    ctypes.POINTER(tSdkCameraDevInfo), ctypes.POINTER(ctypes.c_int))
CAMERA_INIT = setup("CameraInit",
                    ctypes.POINTER(tSdkCameraDevInfo), ctypes.c_int, ctypes.c_int,
                    ctypes.POINTER(ctypes.c_int))
CAMERA_UNINIT = setup("CameraUnInit", ctypes.c_int)
CAMERA_GET_BUF = setup("CameraGetImageBuffer",
                       ctypes.c_int, ctypes.POINTER(tSdkFrameHead), ctypes.POINTER(BYTE_P),
                       ctypes.c_uint)
CAMERA_PROC = setup("CameraImageProcess", ctypes.c_int, BYTE_P, BYTE_P,
                    ctypes.POINTER(tSdkFrameHead))
CAMERA_RELEASE = setup("CameraReleaseImageBuffer", ctypes.c_int, BYTE_P)
CAMERA_PLAY = setup("CameraPlay", ctypes.c_int)
CAMERA_SET_AE = setup("CameraSetAeState", ctypes.c_int, ctypes.c_int)
CAMERA_SET_EXP = setup("CameraSetExposureTime", ctypes.c_int, ctypes.c_double)
CAMERA_SET_ISP = setup("CameraSetIspOutFormat", ctypes.c_int, ctypes.c_uint)
CAMERA_SET_TRIG = setup("CameraSetTriggerMode", ctypes.c_int, ctypes.c_int)


class MindVisionCam:
    """MindVision 相机封装, 用法对齐官方 BasicDemo。"""

    def __init__(self, exposure_ms=None):
        self.handle = ctypes.c_int(-1)
        self.frame_head = tSdkFrameHead()
        self.out_buf = None
        self.exposure_ms = exposure_ms
        self.width = 0
        self.height = 0

    def open(self):
        if CAMERA_SDK_INIT(1) != 0:
            raise RuntimeError("CameraSdkInit 失败")
        dev = tSdkCameraDevInfo()
        num = ctypes.c_int(1)
        if CAMERA_ENUM(ctypes.byref(dev), ctypes.byref(num)) != 0 or num.value == 0:
            raise RuntimeError("未找到相机。请确认迈德威视相机已连接且 SDK 驱动已安装。")
        print(f"  相机: {dev.acProductName.decode(errors='ignore')} "
              f"SN={dev.acSn.decode(errors='ignore')}")
        if CAMERA_INIT(ctypes.byref(dev), -1, -1, ctypes.byref(self.handle)) != 0:
            raise RuntimeError("CameraInit 失败")
        h = self.handle.value
        CAMERA_SET_AE(h, 0)                      # 关闭自动曝光
        if self.exposure_ms is not None:
            CAMERA_SET_EXP(h, self.exposure_ms * 1000.0)  # 单位微秒
        CAMERA_SET_ISP(h, CAMERA_MEDIA_TYPE_BGR8)
        CAMERA_SET_TRIG(h, 0)                    # 连续采集
        CAMERA_PLAY(h)
        return True

    def grab(self):
        """取一帧, 返回 BGR numpy 数组和曝光(us); 失败(超时)返回 None。"""
        raw = BYTE_P()
        if CAMERA_GET_BUF(self.handle.value, ctypes.byref(self.frame_head),
                          ctypes.byref(raw), 100) != 0:
            return None
        fb = self.frame_head
        if self.out_buf is None or fb.iWidth * fb.iHeight * 3 != len(self.out_buf):
            self.out_buf = (ctypes.c_ubyte * (fb.iWidth * fb.iHeight * 3))()
            self.width, self.height = fb.iWidth, fb.iHeight
        if CAMERA_PROC(self.handle.value, ctypes.cast(raw, BYTE_P),
                       ctypes.cast(self.out_buf, BYTE_P),
                       ctypes.byref(self.frame_head)) != 0:
            CAMERA_RELEASE(self.handle.value, ctypes.cast(raw, BYTE_P))
            return None
        CAMERA_RELEASE(self.handle.value, ctypes.cast(raw, BYTE_P))
        w, h = self.frame_head.iWidth, self.frame_head.iHeight
        bgr = np.frombuffer(self.out_buf, dtype=np.uint8, count=w * h * 3)
        return bgr.reshape(h, w, 3).copy(), self.frame_head.uiExpTime

    def close(self):
        if self.handle.value >= 0:
            CAMERA_UNINIT(self.handle.value)
            self.handle.value = -1


class _TimestampedCam:
    """把相机 grab() 适配成 (图像, 墙钟时间戳ns) —— bag 时间戳统一用 time.time_ns()。"""

    def __init__(self, cam):
        self._cam = cam

    def grab(self):
        res = self._cam.grab()
        if res is None:
            return None
        return res[0], time.time_ns()

    def close(self):
        self._cam.close()


def main():
    parser = argparse.ArgumentParser(description="迈德威视(SDK)录制 + rosbag 打包")
    parser.add_argument("--out", required=True, help="输出原始录制 .bag 路径")
    parser.add_argument("--topic", default=cc.DEFAULT_TOPIC, help="图像话题")
    parser.add_argument("--exposure", type=float, default=None,
                        help="曝光(毫秒), 缺省为关闭自动曝光并保持默认")
    cc.add_board_args(parser)
    args = parser.parse_args()
    cc.check_board_args(args)

    print("[1/3] 打开迈德威视相机(SDK) ...")
    cam = MindVisionCam(args.exposure)
    try:
        cam.open()
    except RuntimeError as e:
        raise SystemExit(f"相机打开失败: {e}")
    print(f"    分辨率: {cam.width or '?'}x{cam.height or '?'} (BGR8) "
          f"-> 写入 {args.out_size[0]}x{args.out_size[1]} (对齐运行时 preprocess.resize)")

    print("[2/3] 创建 rosbag ...")
    try:
        cc.run_capture_session(
            _TimestampedCam(cam),
            out_path=args.out, topic=args.topic, out_size=args.out_size,
            cols=args.cols, rows=args.rows, grid=args.grid,
            window="MindVision Capture")
    finally:
        try:
            cam.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
