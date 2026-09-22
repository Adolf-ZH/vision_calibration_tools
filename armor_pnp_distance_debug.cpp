#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "hw_io/camera/camera_config.h"
#include "hw_io/camera/camera_factory.h"
#include "modules/autoaim/detector/armor_detector.h"
#include "modules/autoaim/pnp/armor_pnp_solver.h"
#include "tool/logger/logger.h"

namespace {

constexpr const char* kWindowName = "armor_pnp_distance_debug";
constexpr const char* kDefaultDetectorConfigPath =
    "src/modules/autoaim/detector/nn/tup_yolox/tup_yolox.conf";
// 装甲板尺寸默认值与 src/config/autoaim.yaml 中的 small_armor_width_m /
// small_armor_height_m 保持一致；big 宽度沿用 PnP 求解器的默认值。
constexpr double kDefaultSmallArmorWidthM = 0.135;
constexpr double kDefaultBigArmorWidthM = 0.230;
constexpr double kDefaultArmorHeightM = 0.056;

struct DebugArgs {
    std::string source = "camera";
    std::string video_path;
    std::string camera_backend = "auto";
    std::string camera_config_path;
    std::string detector_config_path = kDefaultDetectorConfigPath;
    std::string model_path;
    std::string device;
    int width = 0;
    int height = 0;
    int max_frames = 0;
    int summary_every = 30;
    bool display = true;
    bool cv_refine = true;
    bool armor_big = false;
    // 卷尺实测的相机到装甲板距离（米）；> 0 时额外输出比值与反推焦距。
    double truth_m = 0.0;
};

void printUsage(const char* argv0) {
    TUP_LOG_INFO_STREAM << "Usage: " << argv0 << " [options]\n"
                        << "\nSource options:\n"
                        << "  --source camera|video        (default camera)\n"
                        << "  video:  --video path\n"
                        << "  camera: [--camera-backend auto|mindvision|hikrobot|video] [--camera-config path]\n"
                        << "\nDetector / PnP options:\n"
                        << "  --detector-config path       (default " << kDefaultDetectorConfigPath << ")\n"
                        << "  --model path   --device CPU|GPU\n"
                        << "  --width W --height H         0 = use camera config output size\n"
                        << "  --cv-refine|--no-cv-refine   (default --cv-refine)\n"
                        << "  --armor small|big            (default small)\n"
                        << "\nMeasurement options:\n"
                        << "  --truth-m D                  tape-measured camera->armor distance (m), optional\n"
                        << "  --summary-every N            print a window summary every N frames (default 30)\n"
                        << "  --frames N                   0 = run until exit\n"
                        << "  --display|--no-display       (default --display)  q/Esc: quit\n"
                        << "  --help|-h\n";
}

DebugArgs parseArgs(int argc, char** argv) {
    DebugArgs args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto requireValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(std::string("missing value for ") + name);
            }
            return argv[++i];
        };

        if (key == "--source") {
            args.source = requireValue("--source");
        } else if (key == "--video") {
            args.video_path = requireValue("--video");
        } else if (key == "--camera-backend") {
            args.camera_backend = requireValue("--camera-backend");
        } else if (key == "--camera-config") {
            args.camera_config_path = requireValue("--camera-config");
        } else if (key == "--detector-config") {
            args.detector_config_path = requireValue("--detector-config");
        } else if (key == "--model") {
            args.model_path = requireValue("--model");
        } else if (key == "--device") {
            args.device = requireValue("--device");
        } else if (key == "--width") {
            args.width = std::stoi(requireValue("--width"));
        } else if (key == "--height") {
            args.height = std::stoi(requireValue("--height"));
        } else if (key == "--frames") {
            args.max_frames = std::stoi(requireValue("--frames"));
        } else if (key == "--summary-every") {
            args.summary_every = std::stoi(requireValue("--summary-every"));
        } else if (key == "--truth-m") {
            args.truth_m = std::stod(requireValue("--truth-m"));
        } else if (key == "--armor") {
            const std::string value = requireValue("--armor");
            if (value == "small") {
                args.armor_big = false;
            } else if (value == "big") {
                args.armor_big = true;
            } else {
                throw std::runtime_error("--armor must be small or big");
            }
        } else if (key == "--no-display") {
            args.display = false;
        } else if (key == "--display") {
            args.display = true;
        } else if (key == "--no-cv-refine") {
            args.cv_refine = false;
        } else if (key == "--cv-refine") {
            args.cv_refine = true;
        } else if (key == "--help" || key == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown argument: " + key);
        }
    }

    if (args.source != "video" && args.source != "camera") {
        throw std::runtime_error("--source must be video or camera");
    }
    if (args.source == "video" && args.video_path.empty()) {
        throw std::runtime_error("--source video requires --video path");
    }
    if (args.max_frames < 0) {
        throw std::runtime_error("--frames must be >= 0");
    }
    if (args.summary_every <= 0) {
        throw std::runtime_error("--summary-every must be > 0");
    }
    if (args.truth_m < 0.0) {
        throw std::runtime_error("--truth-m must be >= 0");
    }
    return args;
}

void applySizeOverride(tup_vision::camera::CameraConfig& config, const DebugArgs& args) {
    if (args.width <= 0 && args.height <= 0) {
        return;
    }
    const int width = args.width > 0 ? args.width : config.output_size.width;
    const int height = args.height > 0 ? args.height : config.output_size.height;
    config.output_size = cv::Size(width, height);
    config.transform.resize.size = config.output_size;
    config.intrinsics.image_size = config.output_size;
}

class FrameSource {
public:
    virtual ~FrameSource() = default;
    virtual bool next(tup_vision::CameraFrame& frame) = 0;
    virtual std::string description() const = 0;
};

class CameraFrameSource final : public FrameSource {
public:
    CameraFrameSource(std::unique_ptr<tup_vision::camera::CameraBase> camera,
                      std::string description)
        : camera_(std::move(camera)),
          description_(std::move(description)) {
        if (!camera_->isOpened() && !camera_->open()) {
            throw std::runtime_error("failed to open " + description_);
        }
    }

    ~CameraFrameSource() override {
        camera_->close();
    }

    bool next(tup_vision::CameraFrame& frame) override {
        return camera_->grab(frame);
    }

    std::string description() const override {
        return description_;
    }

private:
    std::unique_ptr<tup_vision::camera::CameraBase> camera_;
    std::string description_;
};

// 已解析的取流配置与对应的帧源。保留 config 是为了后续读取标定内参。
struct PreparedSource {
    tup_vision::camera::CameraConfig config;
    std::unique_ptr<FrameSource> source;
    std::string description;
};

PreparedSource prepareVideoSource(const DebugArgs& args) {
    PreparedSource prepared;
    prepared.config = tup_vision::camera::loadCameraConfig(
        tup_vision::camera::defaultCameraConfigPath("video"), "video");
    prepared.config.source = args.video_path;
    applySizeOverride(prepared.config, args);
    prepared.description = "video:" + args.video_path;
    prepared.source = std::make_unique<CameraFrameSource>(
        tup_vision::camera::createCamera(prepared.config), prepared.description);
    return prepared;
}

PreparedSource prepareCameraSource(const DebugArgs& args) {
    if (args.camera_config_path.empty() &&
        tup_vision::camera::isAutoCameraBackend(args.camera_backend)) {
        std::vector<std::string> candidates;
#if TUP_VISION_HAS_HIKROBOT
        candidates.emplace_back("hikrobot");
#endif
#if TUP_VISION_HAS_MINDVISION
        candidates.emplace_back("mindvision");
#endif
        candidates.emplace_back("video");
        for (const auto& backend : candidates) {
            const std::string config_path =
                tup_vision::camera::defaultCameraConfigPath(backend);
            try {
                PreparedSource prepared;
                prepared.config = tup_vision::camera::loadCameraConfig(config_path, backend);
                applySizeOverride(prepared.config, args);
                prepared.description = "camera:" + backend + " config=" + config_path;
                prepared.source = std::make_unique<CameraFrameSource>(
                    tup_vision::camera::createCamera(prepared.config), prepared.description);
                return prepared;
            } catch (const std::exception& e) {
                TUP_LOG_WARN("auto camera skipped {}: {}", backend, e.what());
            }
        }
        throw std::runtime_error("no camera backend could be opened automatically");
    }

    const std::string config_path = args.camera_config_path.empty()
        ? tup_vision::camera::defaultCameraConfigPath(args.camera_backend)
        : args.camera_config_path;
    const std::string backend_override =
        tup_vision::camera::isAutoCameraBackend(args.camera_backend)
        ? std::string{}
        : args.camera_backend;
    PreparedSource prepared;
    prepared.config = tup_vision::camera::loadCameraConfig(config_path, backend_override);
    applySizeOverride(prepared.config, args);
    prepared.description = "camera:" + prepared.config.backend_name + " config=" + config_path;
    prepared.source = std::make_unique<CameraFrameSource>(
        tup_vision::camera::createCamera(prepared.config), prepared.description);
    return prepared;
}

PreparedSource prepareSource(const DebugArgs& args) {
    if (args.source == "video") {
        return prepareVideoSource(args);
    }
    return prepareCameraSource(args);
}

cv::Mat drawFrame(
    const cv::Mat& image,
    const std::vector<tup_vision::ArmorDetection>& detections,
    const tup_vision::ArmorObservation* representative,
    const std::vector<std::string>& overlay_lines) {
    cv::Mat output = image.clone();
    for (const auto& detection : detections) {
        const bool is_representative =
            representative != nullptr &&
            detection.corners[0] == representative->detection.corners[0];
        const cv::Scalar color = is_representative ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        for (std::size_t corner = 0; corner < detection.corners.size(); ++corner) {
            cv::line(
                output,
                detection.corners[corner],
                detection.corners[(corner + 1) % detection.corners.size()],
                color,
                2);
            cv::circle(output, detection.corners[corner], 4, color, cv::FILLED);
        }
    }

    int y = 26;
    for (const auto& line : overlay_lines) {
        cv::putText(
            output,
            line,
            cv::Point(12, y),
            cv::FONT_HERSHEY_SIMPLEX,
            0.6,
            cv::Scalar(0, 255, 255),
            1,
            cv::LINE_AA);
        y += 24;
    }
    return output;
}

double medianOf(std::vector<double> values) {
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if (values.size() % 2 == 1) {
        return values[mid];
    }
    return 0.5 * (values[mid - 1] + values[mid]);
}

double meanOf(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (const double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

double stdOf(const std::vector<double>& values, const double mean) {
    if (values.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (const double value : values) {
        const double diff = value - mean;
        sum += diff * diff;
    }
    return std::sqrt(sum / static_cast<double>(values.size()));
}

struct FrameSample {
    bool hit = false;
    double z = 0.0;
    double implied_fx = 0.0;
};

void printSummary(const std::vector<FrameSample>& window, const double truth_m) {
    std::vector<double> z_values;
    std::vector<double> implied_fx_values;
    for (const auto& sample : window) {
        if (!sample.hit) {
            continue;
        }
        z_values.push_back(sample.z);
        implied_fx_values.push_back(sample.implied_fx);
    }

    const std::size_t frames = window.size();
    const std::size_t hit = z_values.size();
    if (hit == 0) {
        std::cout << "[summary] frames=" << frames << " hit=0" << std::endl;
        return;
    }

    const double z_mean = meanOf(z_values);
    const double z_median = medianOf(z_values);
    std::cout << "[summary] frames=" << frames
              << " hit=" << hit
              << std::fixed << std::setprecision(4)
              << " z_median=" << z_median
              << " z_mean=" << z_mean
              << " z_std=" << stdOf(z_values, z_mean);
    if (truth_m > 0.0) {
        std::cout << std::setprecision(1)
                  << " implied_fx_median=" << medianOf(implied_fx_values)
                  << std::setprecision(3)
                  << " ratio_median=" << (z_median / truth_m);
    }
    std::cout << std::endl;
}

// 画面分格统计: 把每帧代表装甲板的解算结果按其在画面中的位置归入 3x3 网格,
// 用于检查视野上/下/左/右/中心(及四角)处的测距误差是否一致。
constexpr int kRegionGrid = 3;
constexpr int kRegionCount = kRegionGrid * kRegionGrid;

struct RegionStats {
    std::vector<double> z_values;
    std::vector<double> implied_fx_values;
    double sum_x = 0.0;
    double sum_y = 0.0;
    std::size_t hits = 0;
};

int regionIndexOf(const cv::Point2f& center, const int width, const int height) {
    const int col = std::clamp(static_cast<int>(center.x * kRegionGrid / width), 0, kRegionGrid - 1);
    const int row = std::clamp(static_cast<int>(center.y * kRegionGrid / height), 0, kRegionGrid - 1);
    return row * kRegionGrid + col;
}

const char* regionNameOf(const int index) {
    static const char* kNames[kRegionCount] = {
        "top-left", "top", "top-right",
        "left", "center", "right",
        "bottom-left", "bottom", "bottom-right",
    };
    return kNames[index];
}

void printRegionSummary(const std::array<RegionStats, kRegionCount>& regions,
                        const double truth_m,
                        const int width,
                        const int height,
                        const std::uint64_t frames) {
    std::cout << "[region] " << kRegionGrid << "x" << kRegionGrid
              << " breakdown over " << width << "x" << height
              << " (rows top->bottom, cols left->right), frames=" << frames << std::endl;
    for (int index = 0; index < kRegionCount; ++index) {
        const RegionStats& stats = regions[static_cast<std::size_t>(index)];
        std::cout << "[region] " << std::left << std::setw(13) << regionNameOf(index) << std::right;
        if (stats.hits == 0) {
            std::cout << "hit=0" << std::endl;
            continue;
        }
        const double z_median = medianOf(stats.z_values);
        std::cout << "hit=" << stats.hits
                  << std::fixed << std::setprecision(4)
                  << " z_median=" << z_median
                  << std::setprecision(0)
                  << " pos=(" << (stats.sum_x / static_cast<double>(stats.hits)) << ","
                  << (stats.sum_y / static_cast<double>(stats.hits)) << ")";
        if (truth_m > 0.0) {
            std::cout << std::setprecision(3)
                      << " ratio_median=" << (z_median / truth_m)
                      << std::setprecision(1)
                      << " implied_fx_median=" << medianOf(stats.implied_fx_values);
        }
        std::cout << std::endl;
    }
}

int run(const DebugArgs& args) {
    auto detector_config =
        tup_vision::autoaim::loadNnDetectorConfig(args.detector_config_path);
    if (!args.model_path.empty()) {
        detector_config.model_path = args.model_path;
    }
    if (!args.device.empty()) {
        detector_config.device = args.device;
    }

    tup_vision::autoaim::ArmorDetectorConfig armor_config;
    armor_config.nn = detector_config;
    armor_config.cv_refiner.enabled = args.cv_refine;
    tup_vision::autoaim::ArmorDetector detector;
    detector.init(armor_config);

    PreparedSource prepared = prepareSource(args);
    const auto& intrinsics = prepared.config.intrinsics;

    tup_vision::autoaim::ArmorPnpSolverConfig pnp_config;
    pnp_config.intrinsics = intrinsics;
    pnp_config.small_armor_width_m = kDefaultSmallArmorWidthM;
    pnp_config.big_armor_width_m = kDefaultBigArmorWidthM;
    pnp_config.armor_height_m = kDefaultArmorHeightM;
    tup_vision::autoaim::ArmorPnpSolver solver(pnp_config);

    const double fx = intrinsics.camera_matrix(0, 0);
    const double fy = intrinsics.camera_matrix(1, 1);
    const double cx = intrinsics.camera_matrix(0, 2);
    const double cy = intrinsics.camera_matrix(1, 2);
    const double armor_width =
        args.armor_big ? pnp_config.big_armor_width_m : pnp_config.small_armor_width_m;

    TUP_LOG_INFO_STREAM << "armor_pnp_distance_debug ready\n"
                        << "  source: " << prepared.description << '\n'
                        << "  backend: " << prepared.config.backend_name << '\n'
                        << "  resolution: " << prepared.config.output_size.width << "x"
                        << prepared.config.output_size.height << '\n'
                        << "  intrinsics fx/fy/cx/cy: " << fx << " / " << fy << " / "
                        << cx << " / " << cy << '\n'
                        << "  distortion: " << [&]() {
                               std::string text;
                               for (std::size_t i = 0; i < intrinsics.distortion_coefficients.size(); ++i) {
                                   if (i > 0) {
                                       text += ", ";
                                   }
                                   text += std::to_string(intrinsics.distortion_coefficients[i]);
                               }
                               return text;
                           }() << '\n'
                        << "  armor: " << (args.armor_big ? "big" : "small")
                        << " width/height: " << armor_width << " / " << pnp_config.armor_height_m << '\n'
                        << "  detector_config: " << args.detector_config_path << '\n'
                        << "  model: " << detector_config.model_path << '\n'
                        << "  device: " << detector_config.device << '\n'
                        << "  cv_refine: " << (args.cv_refine ? "true" : "false") << '\n'
                        << "  display: " << (args.display ? "true" : "false") << '\n'
                        << "  truth_m: " << args.truth_m;

    std::vector<FrameSample> window;
    window.reserve(static_cast<std::size_t>(args.summary_every));

    std::array<RegionStats, kRegionCount> regions;

    std::uint64_t processed = 0;
    bool quit = false;
    const auto started = std::chrono::steady_clock::now();

    while (!quit && (args.max_frames == 0 || processed < args.max_frames)) {
        tup_vision::CameraFrame frame;
        if (!prepared.source->next(frame)) {
            TUP_LOG_INFO("input ended after {} frames", processed);
            break;
        }

        std::vector<tup_vision::ArmorDetection> detections;
        detector.detect(frame, detections);
        // 强制所有检测使用命令行指定的装甲板尺寸，覆盖求解器的类别推断。
        for (auto& detection : detections) {
            detection.armor_size =
                args.armor_big ? tup_vision::ArmorSize::BIG : tup_vision::ArmorSize::SMALL;
        }

        std::vector<tup_vision::ArmorObservation> observations;
        solver.solve(detections, observations);

        // 选一帧里重投影误差最小、且深度为正的装甲板作为代表。
        const tup_vision::ArmorObservation* representative = nullptr;
        for (const auto& observation : observations) {
            if (observation.tvec[2] <= 0.0) {
                continue;
            }
            if (representative == nullptr ||
                observation.reprojection_error < representative->reprojection_error) {
                representative = &observation;
            }
        }

        const std::uint64_t index = processed;
        ++processed;

        FrameSample sample;
        std::vector<std::string> overlay_lines;
        if (representative != nullptr) {
            const double z = representative->tvec[2];
            const double t_norm = cv::norm(representative->tvec);
            const double reproj = representative->reprojection_error;
            const double implied_fx = z > 0.0 ? fx * args.truth_m / z : 0.0;

            sample.hit = true;
            sample.z = z;
            sample.implied_fx = implied_fx;

            // 按代表装甲板的画面位置归入 3x3 网格。
            cv::Point2f center(0.0F, 0.0F);
            for (const auto& corner : representative->detection.corners) {
                center.x += corner.x;
                center.y += corner.y;
            }
            const float corner_count = static_cast<float>(representative->detection.corners.size());
            center.x /= corner_count;
            center.y /= corner_count;
            RegionStats& region = regions[static_cast<std::size_t>(
                regionIndexOf(center, frame.image.cols, frame.image.rows))];
            ++region.hits;
            region.sum_x += center.x;
            region.sum_y += center.y;
            region.z_values.push_back(z);
            region.implied_fx_values.push_back(implied_fx);

            std::cout << '[' << std::setfill('0') << std::setw(4) << index << ']'
                      << std::setfill(' ')
                      << " armors=" << detections.size()
                      << std::fixed << std::setprecision(4)
                      << " z=" << z
                      << " |t|=" << t_norm
                      << std::setprecision(2)
                      << " reproj=" << reproj << "px";
            if (args.truth_m > 0.0) {
                std::cout << std::setprecision(3)
                          << " truth=" << args.truth_m
                          << " ratio=" << (z / args.truth_m)
                          << std::setprecision(1)
                          << " implied_fx=" << implied_fx;
            }
            std::cout << std::endl;

            overlay_lines.push_back("z=" + std::to_string(z).substr(0, 6) + "m");
            overlay_lines.push_back("reproj=" + std::to_string(reproj).substr(0, 5) + "px");
            overlay_lines.push_back("fx=" + std::to_string(fx).substr(0, 7));
            if (args.truth_m > 0.0) {
                overlay_lines.push_back(
                    "truth=" + std::to_string(args.truth_m).substr(0, 5) +
                    " ratio=" + std::to_string(z / args.truth_m).substr(0, 5));
            }
        }

        window.push_back(sample);
        if (window.size() > static_cast<std::size_t>(args.summary_every)) {
            window.erase(window.begin());
        }
        if (processed % static_cast<std::uint64_t>(args.summary_every) == 0) {
            printSummary(window, args.truth_m);
        }

        if (args.display) {
            cv::imshow(
                kWindowName,
                drawFrame(frame.image, detections, representative, overlay_lines));
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                quit = true;
            }
        }
    }

    if (args.display) {
        cv::destroyWindow(kWindowName);
    }
    if (processed > 0) {
        printRegionSummary(regions, args.truth_m, prepared.config.output_size.width,
                           prepared.config.output_size.height, processed);
    }
    const double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    const double fps = elapsed_s > 0.0 ? static_cast<double>(processed) / elapsed_s : 0.0;
    TUP_LOG_INFO("processing complete: frames={}, elapsed_s={:.3f}, avg_fps={:.2f}",
                 processed, elapsed_s, fps);
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    tup_vision::tool::logger::initializeApplication("armor_pnp_distance_debug");
    try {
        const DebugArgs args = parseArgs(argc, argv);
        return run(args);
    } catch (const std::exception& e) {
        TUP_LOG_ERROR("armor_pnp_distance_debug error: {}", e.what());
        printUsage(argv[0]);
        return 1;
    }
}
