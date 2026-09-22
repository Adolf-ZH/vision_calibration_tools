// taggen_diag.cpp - 生成 36h11 tag 图像并检测, 验证检测器基准; 再测 frame0
// 位布局: 检测器 (TagDetector.cc) 按 iy=dim-1..0, ix=0..dim-1 顺序读取,
//         MSB = 左下角 cell. cell(iy,ix) 对应 code bit = dim^2-1-(dim-1-iy)*dim-ix.
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>
#include "apriltags/TagDetector.h"
#include "apriltags/Tag36h11.h"
#include "apriltags/TagFamily.h"

using namespace AprilTags;

int main(int argc, char** argv) {
  // 1) 生成 id=0 的 36h11 tag
  TagFamily fam(tagCodes36h11, 2);
  int dim = fam.dimension;   // 6
  int bb  = fam.blackBorder; // 2
  int dd  = 2*bb + dim;      // 10
  unsigned long long code = fam.codes[0];
  printf("dim=%d blackBorder=%d dd=%d code0=0x%llx bits=%d\n",
         dim, bb, dd, code, fam.bits);

  int s = 30;  // 每 cell 像素
  cv::Mat tag = cv::Mat::zeros(dd*s, dd*s, CV_8UC1);  // 全黑(含黑边框)
  for (int iy = 0; iy < dim; iy++) {        // iy: 数据区行, 0=顶
    for (int ix = 0; ix < dim; ix++) {
      int bit_idx = dim*dim - 1 - (dim-1-iy)*dim - ix;
      int bit = (int)((code >> bit_idx) & 1ULL);
      if (bit) {
        cv::rectangle(tag,
                      cv::Rect((bb+ix)*s, (bb+iy)*s, s, s),
                      cv::Scalar(255), -1);
      }
    }
  }
  // 白底画布, 放大
  int pad = 150;
  cv::Mat canvas = cv::Mat::ones(dd*s + 2*pad, dd*s + 2*pad, CV_8UC1) * 255;
  tag.copyTo(canvas(cv::Rect(pad, pad, dd*s, dd*s)));
  cv::Mat big;
  cv::resize(canvas, big, cv::Size(1200, 1200), 0, 0, cv::INTER_NEAREST);
  cv::imwrite("/tmp/gen_tag.png", big);

  TagDetector det(tagCodes36h11, 2);
  auto d0 = det.extractTags(big);
  printf("合成36h11 tag: extractTags = %zu\n", d0.size());
  for (auto& d : d0)
    printf("  id=%d good=%d hd=%d rot=%d\n", d.id, d.good, d.hammingDistance, d.rotation);

  // 2) frame0 (可选参数)
  if (argc > 1) {
    cv::Mat img = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if (!img.empty()) {
      auto d1 = det.extractTags(img);
      printf("frame0: extractTags = %zu\n", d1.size());
      for (auto& d : d1)
        printf("  id=%d good=%d hd=%d\n", d.id, d.good, d.hammingDistance);
    } else {
      printf("无法读取 %s\n", argv[1]);
    }
  }
  return 0;
}
