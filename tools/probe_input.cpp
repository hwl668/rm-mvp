#include "armor/io.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace armor;

int main(int argc, char** argv) {
  std::string cam_yaml = "config/camera.yaml";
  std::string source   = (argc > 1) ? argv[1] : "data/videos/demo.mp4"; // 或 "0"

  try {
    Camera cam = LoadCamera(cam_yaml);

    VideoReader vr;
    if (!vr.open(source)) {
      std::cerr << "[Probe] Open source failed: " << source << "\n";
      return 2;
    }

    cv::Mat frame;
    if (!vr.read(frame) || frame.empty()) {
      std::cerr << "[Probe] Read first frame failed.\n";
      return 3;
    }
    cv::Size sz = vr.size();
    AdaptIntrinsicsToFrame(cam, sz.width, sz.height);

    std::cout << "[Probe] Source: " << vr.source()
              << (vr.isCamera() ? " (camera)" : " (file)") << "\n";
    std::cout << "[Probe] Stream size = " << sz.width << " x " << sz.height
              << "  fps = " << vr.fps() << "\n";
    std::cout << "[Probe] K(adapted):\n" << cam.K << "\n";
    std::cout << "[Probe] dist:\n" << cam.dist << "\n";

    cv::putText(frame, "Probe OK - press any key", {30,40},
                cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,0}, 2);
    cv::imshow("probe_input", frame);
    cv::waitKey(0);
    return 0;

  } catch (const std::exception& e) {
    std::cerr << "[Probe] Exception: " << e.what() << "\n";
    return 1;
  }
}
