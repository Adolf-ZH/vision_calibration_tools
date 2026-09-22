#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""海康威视 (Hikvision) 工业相机 —— AprilGrid 标定【录制】脚本(MVS SDK 版)

背景:
    海康工业相机需要用厂商 MVS SDK (/opt/MVS)。相机不会出现在 /dev/videoN。
    本脚本复用官方 Python 绑定(MvCameraControl_class), 轮询取帧(MV_CC_GetImageBuffer),
    转灰度后写入 rosbag, 产物再经 extract_calib_frames.py 抽帧, 交给
    basalt_calibrate (--dataset-type bag) 标定。

依赖:
    1. MVS SDK 已安装于 /opt/MVS, 含 lib/64/libMvCameraControl.so 与 Samples/.../MvImport
    2. python3 -m pip install --user opencv-python rosbags

用法:
    python3 capture_hikvision.py \
        --out ~/calib_data/hikvision_raw.bag \
        --exposure 30     # 曝光(毫秒, 可选)
        --index 0         # 枚举到的相机序号(多相机时用)

操作:
    R        开始 / 停止录制(可反复)
    q / Esc  退出(自动关闭 bag)

可在命令行前设置:
    MVCAM_COMMON_RUNENV=/opt/MVS/lib   (SDK 库目录, 脚本已自动检测 /opt/MVS)
"""

import argparse
import ctypes
import os
import sys
import time

import numpy as np

import calib_common as cc

# ---------- 定位 MVS SDK 并加载绑定 ----------
MVS_LIB_DIRS = [os.environ.get("MVCAM_COMMON_RUNENV", ""), "/opt/MVS/lib"]
MVS_SAMPLE_DIRS = [
    "/opt/MVS/Samples/64/Python/MvImport",
    "/opt/MVS/Samples/2020/Python/MvImport",
]
MVS_LIB = next((d for d in MVS_LIB_DIRS if d and os.path.isdir(d)), "")
if MVS_LIB:
    os.environ.setdefault("MVCAM_COMMON_RUNENV", MVS_LIB)
MVIMPORT = next((d for d in MVS_SAMPLE_DIRS if os.path.isdir(d)), None)
if MVIMPORT is None:
    raise SystemExit("找不到 MVS MvImport Python 绑定目录"
                     "(期望 /opt/MVS/Samples/64/Python/MvImport)")
sys.path.insert(0, MVIMPORT)

from MvCameraControl_class import (  # noqa: E402
    MvCamera,
    MV_CC_DEVICE_INFO_LIST,
    MV_CC_DEVICE_INFO,
    MV_GIGE_DEVICE,
    MV_USB_DEVICE,
    MV_GENTL_CAMERALINK_DEVICE,
    MV_GENTL_CXP_DEVICE,
    MV_GENTL_XOF_DEVICE,
    MV_GENTL_GIGE_DEVICE,
)
from MvErrorDefine_const import MV_OK  # noqa: E402
from CameraParams_header import (  # noqa: E402
    MV_FRAME_OUT,
    MV_TRIGGER_MODE_OFF,
    MV_EXPOSURE_AUTO_MODE_OFF,
)
from PixelType_header import PixelType_Gvsp_BGR8_Packed  # noqa: E402


def _decode(ct_arr):
    b = bytes(ct_arr)
    return b.split(b"\x00")[0].decode("utf-8", errors="replace")


class HikVisionCam:
    """海康相机封装: 枚举 / 打开 / 设置 / 轮询取帧。"""

    def __init__(self, index=0, exposure_ms=None, layer_mask=None):
        self.dev_index = index
        self.exposure_ms = exposure_ms
        self.layer_mask = (layer_mask if layer_mask is not None
                           else MV_GIGE_DEVICE | MV_USB_DEVICE
                           | MV_GENTL_CAMERALINK_DEVICE | MV_GENTL_CXP_DEVICE
                           | MV_GENTL_XOF_DEVICE | MV_GENTL_GIGE_DEVICE)
        self.cam = MvCamera()
        self.frame = None

    def open(self):
        if MvCamera.MV_CC_Initialize() != MV_OK:
            raise RuntimeError("MV_CC_Initialize 失败")
        dev_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(self.layer_mask, dev_list)
        if ret != MV_OK or dev_list.nDeviceNum == 0:
            raise RuntimeError("未找到相机。请确认海康相机已连接且 MVS 驱动已安装。")
        if self.dev_index >= dev_list.nDeviceNum:
            raise RuntimeError(f"index {self.dev_index} 越界, 共 {dev_list.nDeviceNum} 台")
        info = ctypes.cast(dev_list.pDeviceInfo[self.dev_index],
                           ctypes.POINTER(MV_CC_DEVICE_INFO)).contents
        lt = info.nTLayerType
        if lt in (MV_GIGE_DEVICE, MV_GENTL_GIGE_DEVICE):
            name = (_decode(info.SpecialInfo.stGigEInfo.chUserDefinedName)
                    or _decode(info.SpecialInfo.stGigEInfo.chModelName))
            print("GigE: {} IP={}".format(name, ".".join(map(str, [
                (info.SpecialInfo.stGigEInfo.nCurrentIp >> 24) & 0xff,
                (info.SpecialInfo.stGigEInfo.nCurrentIp >> 16) & 0xff,
                (info.SpecialInfo.stGigEInfo.nCurrentIp >> 8) & 0xff,
                info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff]))))
        elif lt == MV_USB_DEVICE:
            name = (_decode(info.SpecialInfo.stUsb3VInfo.chUserDefinedName)
                    or _decode(info.SpecialInfo.stUsb3VInfo.chModelName))
            sn = _decode(info.SpecialInfo.stUsb3VInfo.chSerialNumber)
            print(f"USB3V: {name} SN={sn}")
        else:
            print(f"设备类型 {lt}")
        if self.cam.MV_CC_CreateHandle(info) != MV_OK:
            raise RuntimeError("MV_CC_CreateHandle 失败")
        if self.cam.MV_CC_OpenDevice() != MV_OK:
            raise RuntimeError("MV_CC_OpenDevice 失败")
        # 连续采集
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        # 曝光
        if self.exposure_ms is not None:
            self.cam.MV_CC_SetEnumValue("ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF)
            self.cam.MV_CC_SetFloatValue("ExposureTime", self.exposure_ms * 1000.0)  # 微秒
        # 输出 BGR8
        self.cam.MV_CC_SetEnumValue("PixelFormat", PixelType_Gvsp_BGR8_Packed)
        if self.cam.MV_CC_StartGrabbing() != MV_OK:
            raise RuntimeError("MV_CC_StartGrabbing 失败")
        return True

    def grab(self):
        """取一帧; 返回 (图像[numpy]，相机时间戳ns) 或 None。
        图像可能为 3 通道 BGR 或 1 通道灰度，由 nFrameLen 自动判断。"""
        st = MV_FRAME_OUT()
        if self.cam.MV_CC_GetImageBuffer(st, 100) != MV_OK:
            return None
        info = st.stFrameInfo
        try:
            raw = ctypes.string_at(st.pBufAddr, info.nFrameLen)
            w, h = int(info.nWidth), int(info.nHeight)
            ts_ns = (int(info.nDevTimeStampHigh) << 32
                     | int(info.nDevTimeStampLow)) * 100  # 100ns -> ns
            ch = info.nFrameLen // (w * h)  # 1=灰度, 3=BGR
            img = np.frombuffer(raw, np.uint8, count=w * h * ch).reshape(h, w, ch).copy()
            return img, ts_ns
        finally:
            self.cam.MV_CC_FreeImageBuffer(st)

    def close(self):
        try:
            self.cam.MV_CC_StopGrabbing()
        except Exception:
            pass
        try:
            self.cam.MV_CC_CloseDevice()
        except Exception:
            pass
        try:
            self.cam.MV_CC_DestroyHandle()
        except Exception:
            pass
        try:
            MvCamera.MV_CC_Finalize()
        except Exception:
            pass


class _TimestampedCam:
    """bag 时间戳统一用 time.time_ns(), 不混用相机内部时钟。"""

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
    parser = argparse.ArgumentParser(description="海康威视(MVS SDK)录制 + rosbag 打包")
    parser.add_argument("--out", required=True, help="输出原始录制 .bag 路径")
    parser.add_argument("--topic", default=cc.DEFAULT_TOPIC, help="图像话题")
    parser.add_argument("--exposure", type=float, default=None,
                        help="曝光(毫秒), 缺省关闭自动曝光用相机默认")
    parser.add_argument("--index", type=int, default=0, help="相机序号, 多台时选择")
    cc.add_board_args(parser)
    args = parser.parse_args()
    cc.check_board_args(args)

    print("[1/3] 打开海康相机(MVS SDK) ...")
    cam = HikVisionCam(index=args.index, exposure_ms=args.exposure)
    try:
        cam.open()
    except RuntimeError as e:
        raise SystemExit(f"相机打开失败: {e}")

    print("[2/3] 创建 rosbag ...")
    try:
        cc.run_capture_session(
            _TimestampedCam(cam),
            out_path=args.out, topic=args.topic, out_size=args.out_size,
            cols=args.cols, rows=args.rows, grid=args.grid,
            window="HikVision Capture")
    finally:
        try:
            cam.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
