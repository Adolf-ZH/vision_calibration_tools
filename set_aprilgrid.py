#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 basalt 用的 aprilgrid JSON（标定板规格）。

用法：
    换标定板时，只改下面【板子参数】那一块，然后运行：

        python3 set_aprilgrid.py

    脚本会把同一份规格同时写入两份 JSON：
        ~/calib_data/mindvision_aprilgrid.json   （迈德威视）
        ~/calib_data/hikrobot_aprilgrid.json     （海康）

    两份 JSON 内容相同（同一块板子）。如果两台相机以后用不同的板子，
    把 OUTPUTS 里对应的一行删掉，自己另存一份即可。

注意：
    * 本脚本只写 JSON。采集脚本的 --cols / --rows / --grid 要自己按
      打印出来的提示填（或直接用 calib_common.py 的默认值）。
    * tagSize 填错只会让整体等比缩放；tagSpacing 填错会真正污染内参。
"""

import json
import os

# ==========================================================================
#  板子参数 —— 换板子时只改这里
# ==========================================================================
#
#  怎么量：
#    1. tagCols / tagRows：数板子上横向、纵向各有几个 tag（黑白方块）
#    2. tagSize：用尺子量【单个 tag 的外沿边长】（含黑边框），换算成米
#       —— 不是整块板子的边长，也不含 tag 之间的白边
#    3. tagSpacing：量相邻两个 tag 之间的白边间隙 G，再除以 tagSize
#       —— 是【比值】不是长度！比如间隙 4.4mm、边长 13.7mm → 4.4/13.7 ≈ 0.32
#
TAG_COLS = 10        # 横向 tag 个数（tagCols）
TAG_ROWS = 6         # 纵向 tag 个数（tagRows）
TAG_SIZE = 0.0137    # 单个 tag 外沿边长，单位：米（含黑边框）
TAG_SPACING = 0.32   # 相邻 tag 间隙 ÷ tagSize（比值）
# ==========================================================================

# 输出路径（一般不用改）
OUTPUTS = [
    os.path.expanduser("~/calib_data/mindvision_aprilgrid.json"),
    os.path.expanduser("~/calib_data/hikrobot_aprilgrid.json"),
]


def main() -> None:
    if TAG_COLS <= 0 or TAG_ROWS <= 0:
        raise SystemExit("tagCols / tagRows 必须是正整数，请检查【板子参数】")
    if TAG_SIZE <= 0:
        raise SystemExit("tagSize 必须是正数（单位：米），请检查【板子参数】")
    if TAG_SPACING <= 0:
        raise SystemExit("tagSpacing 必须是正数（比值），请检查【板子参数】")

    data = {
        "tagCols": TAG_COLS,
        "tagRows": TAG_ROWS,
        "tagSize": TAG_SIZE,
        "tagSpacing": TAG_SPACING,
    }
    text = json.dumps(data, indent=2) + "\n"

    for path in OUTPUTS:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"已写入 {path}")

    total = TAG_COLS * TAG_ROWS
    print()
    print(f"  板子规格  : {TAG_COLS} x {TAG_ROWS} = {total} 个 tag")
    print(f"  单 tag 边长: {TAG_SIZE * 1000:.1f} mm")
    print(f"  间隙/边长  : {TAG_SPACING}")
    print(f"  整板外沿   : 宽 {(TAG_COLS * TAG_SIZE + (TAG_COLS - 1) * TAG_SIZE * TAG_SPACING) * 1000:.0f} mm"
          f" x 高 {(TAG_ROWS * TAG_SIZE + (TAG_ROWS - 1) * TAG_SIZE * TAG_SPACING) * 1000:.0f} mm")
    print()
    print(f"  采集时对应 : --cols {TAG_COLS} --rows {TAG_ROWS} --grid {total}")


if __name__ == "__main__":
    main()
