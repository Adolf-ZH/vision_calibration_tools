// apriltag2_bridge.cpp
// 将 basalt 使用的 ethz_apriltag2 检测器封装为 C 接口, 供 Python ctypes 调用。
// 检测参数与 basalt 的 ApriltagDetector 完全一致:
//   - tagCodes36h11
//   - blackTagBorder = 2
//   - minBorderDistance = 4.0 (角点离图像边界小于该值的 tag 剔除)
//   - good != 1 的检测剔除
//   - id >= num_tags 的检测剔除
//   - 按 tag id 排序

#include <algorithm>
#include <cstdint>
#include <vector>

#include "opencv2/opencv.hpp"

#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"

using namespace AprilTags;

extern "C" {

// 创建检测器(36h11, blackBorder=2, 与 basalt 一致)
void* apriltag2_create() {
  return new TagDetector(tagCodes36h11, 2);
}

void apriltag2_destroy(void* det) {
  delete static_cast<TagDetector*>(det);
}

// 检测灰度图(gray: w*h uint8 连续内存)。
// num_tags: 网格 tag 总数, 用于过滤 id 越界的误检测(6x6 -> 36)。
// out_ids: 长度为 max_tags 的 int 数组。
// out_corners: 长度为 max_tags*8 的 float 数组, 每个 tag 4 个角点(x,y),
//              顺序为 p[0]左下, p[1]右下, p[2]右上, p[3]左上(与 basalt 一致)。
// max_tags: 输出数组容量上限。
// 返回实际检测到的 tag 数量。
int apriltag2_detect(void* det, const uint8_t* gray, int w, int h,
                     int num_tags, int* out_ids, float* out_corners,
                     int max_tags) {
  TagDetector* td = static_cast<TagDetector*>(det);
  // 注意: cv::Mat 引用的是外部内存, 不拷贝, extractTags 内部不会修改。
  cv::Mat image(h, w, CV_8UC1, const_cast<uint8_t*>(gray));
  std::vector<TagDetection> detections = td->extractTags(image);

  const double min_border = 4.0;

  std::vector<TagDetection> filtered;
  filtered.reserve(detections.size());
  for (const auto& d : detections) {
    if (d.good != 1) continue;
    if (d.id >= num_tags) continue;

    bool remove = false;
    for (int j = 0; j < 4; j++) {
      if (d.p[j].first < min_border || d.p[j].first > (float)(w - 1) - min_border ||
          d.p[j].second < min_border || d.p[j].second > (float)(h - 1) - min_border) {
        remove = true;
        break;
      }
    }
    if (remove) continue;
    filtered.push_back(d);
  }

  // 按 id 排序(与 basalt 一致)
  std::sort(filtered.begin(), filtered.end(), TagDetection::sortByIdCompare);

  int n = static_cast<int>(filtered.size());
  if (n > max_tags) n = max_tags;

  for (int i = 0; i < n; i++) {
    out_ids[i] = filtered[i].id;
    for (int j = 0; j < 4; j++) {
      out_corners[i * 8 + j * 2]     = filtered[i].p[j].first;
      out_corners[i * 8 + j * 2 + 1] = filtered[i].p[j].second;
    }
  }
  return n;
}

}  // extern "C"