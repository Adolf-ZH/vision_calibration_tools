#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从「原始录制 bag」离线自动抽帧, 生成 basalt 标定用的 bag。

流程:
    原始录制 bag (连续录, 帧多且含废帧)
      -> 逐帧用 ethz_apriltag2 检测 (与 basalt_calibrate 用的检测器完全相同)
      -> 质量过滤: tag 数 >= --min-tags, 拉普拉斯清晰度 >= --min-sharp
      -> 贪心最远点采样: 位置 / 尺度 / 倾斜 / 旋转 差异最大的帧优先,
         保证选出的帧姿态尽量分散(内参标定最需要姿态多样性)
      -> 按时间顺序把选中的帧原样转发进标定 bag (mono8, 尺寸不变)

用法:
    python3 extract_calib_frames.py \
        --in ~/calib_data/mindvision_raw.bag \
        --out ~/calib_data/mindvision_calib.bag

    # 常用调参
    --min-tags 20        # 至少要检测到多少个 tag 才算合格帧(默认 20)
    --min-sharp 20       # 拉普拉斯方差下限, 滤掉运动模糊(0 = 不滤)
    --max-frames 40      # 最多输出多少帧(默认 40)
    --min-distance 0.04  # 姿态差异下限, 低于它就不再选(调大 = 帧更少更分散)

抽完后建议先看脚本打印的覆盖度报告, 再跑 basalt_calibrate。
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

from rosbags.rosbag1 import Reader, Writer

import calib_common as cc

# 特征向量的各项权重(位置/尺度/倾斜/旋转), 决定"姿态差异"怎么算
W_POS = 1.0
W_SCALE = 2.0
W_TILT = 1.5
W_ANGLE = 0.5


def read_image(gray_msg):
    """把 sensor_msgs/Image 转成灰度 numpy 数组。"""
    arr = np.asarray(gray_msg.data, dtype=np.uint8)
    h, w = int(gray_msg.height), int(gray_msg.width)
    enc = gray_msg.encoding
    if enc in ("mono8", "8UC1"):
        return arr.reshape(h, w)
    if enc in ("bgr8", "8UC3"):
        return cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_BGR2GRAY)
    if enc in ("rgb8", "8UC3"):
        return cv2.cvtColor(arr.reshape(h, w, 3), cv2.COLOR_RGB2GRAY)
    raise SystemExit(f"不支持的图像编码: {enc} (只支持 mono8 / bgr8 / rgb8)")


def frame_feature(gray, cols, rows, grid, min_tags, min_sharp):
    """检测 + 质量过滤 + 提取姿态特征; 不合格返回 None。"""
    ids, corn = cc.ethz_detect(gray, num_tags=grid)
    n = len(ids)
    if n < min_tags:
        return None

    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    if sharp < min_sharp:
        return None

    H, inl = cc.board_homography(ids, corn, rows, cols)
    if H is None or inl < 8:
        return None

    h, w = gray.shape
    quad = cc.board_outline(H, rows, cols)          # [左下, 右下, 右上, 左上]
    cx, cy = quad.mean(axis=0)
    area = abs(cv2.contourArea(quad.astype(np.float32)))
    scale = float(np.sqrt(max(area, 1.0)) / np.hypot(w, h))

    top = float(np.linalg.norm(quad[3] - quad[2]))
    bot = float(np.linalg.norm(quad[0] - quad[1]))
    left = float(np.linalg.norm(quad[0] - quad[3]))
    right = float(np.linalg.norm(quad[1] - quad[2]))
    t1 = (top - bot) / max(top + bot, 1e-6)         # 上下透视差 -> 俯仰倾斜
    t2 = (left - right) / max(left + right, 1e-6)   # 左右透视差 -> 偏航倾斜
    ang = float(np.arctan2(quad[1][1] - quad[0][1], quad[1][0] - quad[0][0]))

    return {"tags": n, "sharp": sharp, "pos": (float(cx / w), float(cy / h)),
            "scale": scale, "tilt": (t1, t2), "ang": ang}


def feature_vec(f):
    """把姿态特征拼成用于"差异度"的向量(各项按权重缩放)。"""
    px, py = f["pos"]
    return np.array([
        W_POS * px, W_POS * py,
        W_SCALE * f["scale"],
        W_TILT * f["tilt"][0], W_TILT * f["tilt"][1],
        W_ANGLE * np.sin(f["ang"]), W_ANGLE * np.cos(f["ang"]),
    ], np.float64)


def select_diverse(feats, max_frames, min_distance):
    """贪心最远点采样: 每轮选出"离已选集合最远"的帧, 直到够数或差异太小。"""
    V = np.asarray([feature_vec(f) for f in feats], np.float64)
    n = len(V)
    if n == 0:
        return []
    center = V.mean(axis=0)
    first = int(np.argmin(np.linalg.norm(V - center, axis=1)))
    sel = [first]
    best = np.linalg.norm(V - V[first], axis=1)
    best[first] = -1.0
    while len(sel) < min(max_frames, n):
        i = int(np.argmax(best))
        if best[i] < min_distance:
            break
        sel.append(i)
        best = np.minimum(best, np.linalg.norm(V - V[i], axis=1))
        best[np.asarray(sel)] = -1.0
    return sorted(sel)


def coverage_report(feats, sel):
    """打印位置 3x3 覆盖 + 尺度范围, 便于判断姿态是否够分散。"""
    grid = np.zeros((3, 3), int)
    for i in sel:
        px, py = feats[i]["pos"]
        gx = min(2, max(0, int(px * 3)))
        gy = min(2, max(0, int(py * 3)))
        grid[gy, gx] += 1
    print("\n位置覆盖(3x3, 画面九宫格, 左上角为第一个):")
    for r in range(3):
        print("    " + "  ".join(f"{grid[r, c]:3d}" for c in range(3)))
    scales = [feats[i]["scale"] for i in sel]
    if scales:
        print(f"尺度范围: {min(scales):.3f} ~ {max(scales):.3f} (越小=板子越远)")
    tilts = [max(abs(feats[i]["tilt"][0]), abs(feats[i]["tilt"][1])) for i in sel]
    if tilts:
        print(f"最大倾斜量: {max(tilts):.3f} (0=正对, 越大=越斜)")


def main():
    parser = argparse.ArgumentParser(description="从原始录制 bag 自动抽帧生成标定 bag")
    parser.add_argument("--in", dest="in_path", required=True,
                        help="输入: 原始录制 bag (capture_*.py 的产物)")
    parser.add_argument("--out", dest="out_path", required=True,
                        help="输出: 标定用 bag (交给 basalt_calibrate)")
    parser.add_argument("--topic", default=cc.DEFAULT_TOPIC, help="图像话题")
    parser.add_argument("--min-tags", type=int, default=20,
                        help="合格帧至少要检测到的 tag 数, 默认 20")
    parser.add_argument("--min-sharp", type=float, default=20.0,
                        help="拉普拉斯方差下限(滤运动模糊), 默认 20, 0 = 不滤")
    parser.add_argument("--max-frames", type=int, default=40,
                        help="最多输出多少帧, 默认 40")
    parser.add_argument("--min-distance", type=float, default=0.04,
                        help="姿态差异下限, 低于它停止挑选, 默认 0.04")
    cc.add_board_args(parser)
    args = parser.parse_args()
    cc.check_board_args(args)

    if not os.path.exists(args.in_path):
        raise SystemExit(f"输入 bag 不存在: {args.in_path}")

    # ---------- 第一遍: 逐帧检测 + 质量过滤, 只留特征(不占内存) ----------
    print(f"[1/3] 扫描原始录制: {args.in_path}")
    t0 = time.time()
    feats = []
    total = 0
    enc_seen = None
    r = Reader(args.in_path)
    r.open()
    try:
        for conn, ts, data in r.messages():
            if conn.topic != args.topic:
                continue
            total += 1
            img = cc.STORE.deserialize_ros1(data, "sensor_msgs/msg/Image")
            enc_seen = enc_seen or img.encoding
            gray = read_image(img)
            f = frame_feature(gray, args.cols, args.rows, args.grid,
                              args.min_tags, args.min_sharp)
            if f is not None:
                f["ts"] = ts
                feats.append(f)
            if total % 200 == 0:
                print(f"    已扫描 {total} 帧, 合格 {len(feats)} 帧 ...")
    finally:
        r.close()

    if total == 0:
        raise SystemExit(f"bag 里没有话题 {args.topic} 的图像。检查 --topic 或重新录制。")
    print(f"    共 {total} 帧 (编码 {enc_seen}), 合格 {len(feats)} 帧 "
          f"(耗时 {time.time() - t0:.1f}s)")
    if len(feats) < 10:
        raise SystemExit(
            "合格帧太少。请确认: 板子完整入镜、对焦清晰、曝光合适, "
            "或放宽 --min-tags / --min-sharp 后重试。")

    # ---------- 挑选 ----------
    print("[2/3] 挑选姿态最分散的帧 ...")
    sel = select_diverse(feats, args.max_frames, args.min_distance)
    print(f"    选中 {len(sel)} 帧 / 合格 {len(feats)} 帧")
    for k, i in enumerate(sel, 1):
        f = feats[i]
        print(f"    #{k:02d} t={f['ts'] // 1_000_000}ms tags={f['tags']:3d} "
              f"sharp={f['sharp']:7.1f} pos=({f['pos'][0]:.2f},{f['pos'][1]:.2f}) "
              f"scale={f['scale']:.3f} tilt=({f['tilt'][0]:+.2f},{f['tilt'][1]:+.2f})")
    coverage_report(feats, sel)
    if len(sel) < 30:
        print(f"\n提示: 只选到 {len(sel)} 帧, basalt 建议 30 帧以上。"
              "可减小 --min-distance, 或录更久、覆盖更多姿态。")

    # ---------- 第二遍: 原样转发选中帧 ----------
    print(f"\n[3/3] 写出标定 bag: {args.out_path}")
    keep = {feats[i]["ts"] for i in sel}
    out_path = os.path.abspath(args.out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        print(f"    删除旧的 {out_path}")
        os.remove(out_path)

    r2 = Reader(args.in_path)
    r2.open()
    writer = Writer(out_path)
    writer.open()
    conn_out = writer.add_connection(args.topic, "sensor_msgs/msg/Image",
                                     typestore=cc.STORE)
    written = 0
    try:
        for conn, ts, data in r2.messages():
            if conn.topic != args.topic or ts not in keep:
                continue
            writer.write(conn_out, ts, data)     # 原样转发, 不重新编码
            written += 1
    finally:
        writer.close()
        r2.close()

    print(f"\n完成。{written} 帧 -> {out_path}")
    if written != len(sel):
        print(f"警告: 期望 {len(sel)} 帧, 实际写出 {written} 帧(时间戳可能重复)。")
    print("下一步: basalt_calibrate --dataset-path <该 bag> --dataset-type bag ...")


if __name__ == "__main__":
    sys.exit(main())
