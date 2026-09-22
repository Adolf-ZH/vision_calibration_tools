#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""装甲板 PnP 测距校验 —— 用卷尺实测距离验证相机内参是否正确。

原理:
    PnP 的尺度完全由「已知的装甲板真实尺寸」和「焦距」共同决定:
        z_recovered = fx_used * W_true / W_pixels
    因此若使用的 fx 偏大 k 倍, 解出的距离也会偏大 k 倍。

    在多个实测距离上各拍一张, 拟合:
        z_recovered = k * z_measured + b
    k ≈ 1.00  =>  内参正确。
    「从镜头前端量还是从靶面量」这类固定偏差只影响 b, 不影响 k,
    所以不必纠结参考点从哪里算起。

用法:
    python3 verify_pnp_distance.py --width 0.135 --height 0.056 \
        --distances 1.0 1.5 2.0 3.0

    # 想先试别的一组内参而不改工程配置:
    cp src/config/camera/mindvision.yaml /tmp/test.yaml
    nano /tmp/test.yaml
    python3 verify_pnp_distance.py ... --yaml /tmp/test.yaml

操作:
    SPACE    冻结当前帧, 然后依次点击装甲板的 4 个角:
             左上 -> 右上 -> 右下 -> 左下 (以图像里看到的方位为准)
    ENTER    确认本次拍摄, 记录下来
    r        重拍当前这次
    q / Esc  退出并输出拟合结果

要点:
    - 点「两条灯条外接矩形」的 4 个角, 并且用同一把尺子、同一种量法量
      --width / --height, 保持一致比定义严格更重要。
    - 装甲板要尽量正对相机(不要歪太多), 画面里占得越大越准。
    - 距离建议覆盖 1m ~ 4m, 至少 3 个点。
"""

import argparse
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import calib_common as cc
from capture_mindvision_sdk import MindVisionCam

DEFAULT_YAML = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "src", "config", "camera", "mindvision.yaml")


def load_intrinsics(path):
    """从运行时 yaml 读 camera_matrix / distortion_coefficients。"""
    with open(path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    it = doc["intrinsics"]
    K = np.array(it["camera_matrix"]["data"], np.float64).reshape(3, 3)
    D = np.array(it["distortion_coefficients"]["data"], np.float64).reshape(1, -1)
    return K, D, (int(it["width"]), int(it["height"]))


def refine_corners(gray, pts):
    """把手工点击的角点细化到亚像素。"""
    p = np.array(pts, np.float32).reshape(-1, 1, 2)
    cv2.cornerSubPix(
        gray, p, (7, 7), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    return p.reshape(-1, 2)


def main():
    ap = argparse.ArgumentParser(description="装甲板 PnP 测距校验")
    ap.add_argument("--width", type=float, required=True,
                    help="装甲板灯条外接矩形的实测宽度(米), 如 0.135")
    ap.add_argument("--height", type=float, required=True,
                    help="装甲板灯条外接矩形的实测高度(米), 如 0.056")
    ap.add_argument("--distances", type=float, nargs="*", default=None,
                    help="各次的实测距离(米), 顺序与拍摄顺序一致; 不填则在退出时输入")
    ap.add_argument("--yaml", default=DEFAULT_YAML, help="内参 yaml 路径")
    ap.add_argument("--exposure", type=float, default=None, help="曝光(毫秒)")
    ap.add_argument("--out-size", type=cc.parse_size, default=cc.DEFAULT_OUT_SIZE,
                    help="与运行时一致的输出尺寸, 默认 1280x960")
    args = ap.parse_args()

    yaml_path = os.path.abspath(args.yaml)
    if not os.path.exists(yaml_path):
        raise SystemExit(f"内参文件不存在: {yaml_path}")
    K, D, cfg_size = load_intrinsics(yaml_path)
    print(f"内参: {yaml_path}")
    print(f"  fx={K[0, 0]:.2f} fy={K[1, 1]:.2f} cx={K[0, 2]:.2f} cy={K[1, 2]:.2f}")
    print(f"  k1={D[0, 0]:.6f} k2={D[0, 1]:.6f} p1={D[0, 2]:.6f} "
          f"p2={D[0, 3]:.6f} k3={D[0, 4]:.6f}")
    if cfg_size != tuple(args.out_size):
        print(f"  警告: yaml 标定尺寸 {cfg_size} 与 --out-size {tuple(args.out_size)} "
              "不一致, 内参会失效。")

    half_w, half_h = args.width / 2.0, args.height / 2.0
    objp = np.array([[-half_w, half_h, 0.0], [half_w, half_h, 0.0],
                     [half_w, -half_h, 0.0], [-half_w, -half_h, 0.0]], np.float64)

    cam = MindVisionCam(args.exposure)
    try:
        cam.open()
    except RuntimeError as e:
        raise SystemExit(f"相机打开失败: {e}")

    win = "PnP Distance Check"
    cv2.namedWindow(win)
    state = {"frozen": None, "clicks": [], "result": None}

    def on_mouse(event, x, y, flags, param):
        if (event == cv2.EVENT_LBUTTONDOWN and state["frozen"] is not None
                and len(state["clicks"]) < 4):
            state["clicks"].append((x, y))

    cv2.setMouseCallback(win, on_mouse)
    shots = []

    def solve():
        gray = state["frozen"]
        if len(state["clicks"]) < 4:
            return None
        pts = refine_corners(gray, state["clicks"])
        ok, rvec, tvec = cv2.solvePnP(objp, pts, K, D)
        if not ok:
            return None
        proj, _ = cv2.projectPoints(objp, rvec, tvec, K, D)
        err = float(np.linalg.norm(proj.reshape(-1, 2) - pts, axis=1).mean())
        return {"pts": pts, "t": tvec.ravel(), "r": rvec.ravel(),
                "z": float(np.linalg.norm(tvec)), "err": err}

    print(f"\n装甲板 {args.width*1000:.1f} x {args.height*1000:.1f} mm")
    print("SPACE 冻结 -> 依次点 左上/右上/右下/左下 -> ENTER 确认, r 重拍, q 退出\n")

    try:
        while True:
            if state["frozen"] is None:
                got = cam.grab()
                if got is None:
                    continue
                gray = cc.to_gray(got[0])
                gray = cc.resize_like_runtime(gray, args.out_size)
                gray = np.ascontiguousarray(gray)
                vis = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv2.putText(vis, f"READY  shots={len(shots)}  "
                                 "SPACE=freeze  q=quit",
                            (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2)
                if shots:
                    cv2.putText(vis, f"last z={shots[-1]['z']:.3f} m",
                                (12, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 255), 2)
                cv2.imshow(win, vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                if key == ord(' '):
                    state["frozen"] = gray
                    state["clicks"] = []
                    state["result"] = None
            else:
                vis = cv2.cvtColor(state["frozen"], cv2.COLOR_GRAY2BGR)
                n = len(state["clicks"])
                for i, p in enumerate(state["clicks"]):
                    cv2.circle(vis, (int(p[0]), int(p[1])), 6, (0, 0, 255), -1)
                    cv2.putText(vis, "TL TR BR BL".split()[i],
                                (int(p[0]) + 8, int(p[1]) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                if n < 4:
                    cv2.putText(vis, f"click {n+1}/4: "
                                     f"{'TL TR BR BL'.split()[n]}",
                                (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                (0, 255, 0), 2)
                else:
                    state["result"] = solve()
                    r = state["result"]
                    if r is None:
                        cv2.putText(vis, "solvePnP failed - press r",
                                    (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    (0, 0, 255), 2)
                    else:
                        for p in r["pts"]:
                            cv2.circle(vis, (int(p[0]), int(p[1])), 4,
                                       (0, 255, 255), -1)
                        cv2.putText(vis, f"z={r['z']:.3f} m  reproj={r['err']:.2f} px",
                                    (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                                    (0, 255, 255), 2)
                        cv2.putText(vis, "ENTER=accept   r=redo",
                                    (12, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                    (0, 255, 0), 2)
                cv2.imshow(win, vis)
                key = cv2.waitKey(20) & 0xFF
                if key in (ord('q'), 27):
                    break
                if key == ord('r'):
                    state["frozen"] = None
                    state["clicks"] = []
                if key in (13, 10, ord(' ')) and state["result"] is not None:
                    r = state["result"]
                    shots.append(r)
                    print(f"[#{len(shots)}] z_rec={r['z']:.4f} m  "
                          f"t=({r['t'][0]:+.3f},{r['t'][1]:+.3f},{r['t'][2]:+.3f})  "
                          f"reproj={r['err']:.2f} px")
                    state["frozen"] = None
                    state["clicks"] = []
                    state["result"] = None
    finally:
        cv2.destroyAllWindows()
        cam.close()

    if not shots:
        raise SystemExit("没有有效拍摄。")

    print(f"\n==== 结果 ({len(shots)} 次拍摄) ====")
    for i, s in enumerate(shots, 1):
        print(f"  #{i}  z_rec={s['z']:.4f} m   reproj={s['err']:.2f} px")

    dists = args.distances
    if dists is None:
        try:
            txt = input("\n请依次输入各次实测距离(米), 空格或逗号分隔: ")
            dists = [float(v) for v in txt.replace(",", " ").split()]
        except Exception:
            dists = []
    if len(dists) < len(shots):
        print(f"提示: 只提供了 {len(dists)} 个实测距离, 前 {len(dists)} 次参与拟合。")
        shots = shots[:len(dists)]

    if len(shots) >= 2 and len(dists) == len(shots):
        z_rec = np.array([s["z"] for s in shots])
        z_meas = np.array(dists, np.float64)
        k, b = np.polyfit(z_meas, z_rec, 1)
        print(f"\n  #  z_meas(m)  z_rec(m)   ratio")
        for i, (zm, zr) in enumerate(zip(z_meas, z_rec), 1):
            print(f"  {i}  {zm:9.3f}  {zr:8.3f}  {zr/zm:7.3f}")
        print(f"\n拟合: z_rec = {k:.4f} * z_meas + {b:+.4f}  (m)")
        dev = abs(k - 1.0) * 100
        if dev < 3.0:
            print(f"判定: 斜率 k={k:.4f}, 偏离 1.00 仅 {dev:.1f}%  ->  内参正确")
        elif dev < 8.0:
            print(f"判定: 斜率 k={k:.4f}, 偏离 1.00 有 {dev:.1f}%  ->  基本可用, "
                  "但建议再录一批数据重标定")
        else:
            print(f"判定: 斜率 k={k:.4f}, 偏离 1.00 达 {dev:.1f}%  ->  内参有问题")
        print(f"参考: 若 fx 错了 m 倍, 斜率就是 m。"
              f"本次配置 fx={K[0,0]:.1f}, 若真实 fx 为 1200 则斜率会是 "
              f"{K[0,0]/1200:.2f}")
    else:
        print("\n实测距离不足 2 个, 无法拟合斜率。请把上面的 z_rec 与卷尺读数逐一对照。")


if __name__ == "__main__":
    main()
