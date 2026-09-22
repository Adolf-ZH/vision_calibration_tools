// tagdiag.cpp - 诊断 ethz_apriltag2 在 frame0 上的原始检测(过滤前)
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>
#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"
#include "apriltags/Tag25h9.h"

using namespace AprilTags;

int main(int argc, char** argv) {
  const char* path = argc > 1 ? argv[1] : "/tmp/frame0.npy";
  // 读 .npy (uint8 2D) 简单方式: 用 python 存成 pgm? 这里直接用 imread png
  if (argc > 1) path = argv[1];
  cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
  if (img.empty()) { printf("cannot read %s\n", path); return 1; }
  printf("image %dx%d\n", img.cols, img.rows);

  // 36h11
  TagDetector det36(tagCodes36h11, 2);
  auto dets36 = det36.extractTags(img);
  printf("extractTags(36h11) raw = %zu\n", dets36.size());
  for (auto& d : dets36) {
    printf("  id=%d good=%d ", d.id, d.good);
    for (int j = 0; j < 4; j++) printf("(%.0f,%.0f) ", d.p[j].first, d.p[j].second);
    printf("\n");
  }

  // 25h9 (另一种常见家族, 交叉验证)
  TagDetector det25(tagCodes25h9, 2);
  auto dets25 = det25.extractTags(img);
  printf("extractTags(25h9) raw = %zu\n", dets25.size());
  for (auto& d : dets25) {
    printf("  id=%d good=%d ", d.id, d.good);
    for (int j = 0; j < 4; j++) printf("(%.0f,%.0f) ", d.p[j].first, d.p[j].second);
    printf("\n");
  }
  return 0;
}
