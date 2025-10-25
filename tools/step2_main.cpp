#include "armor/io.hpp"
#include "armor/color_selector.hpp"
#include "armor/lightbar_detector.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace armor;

static std::string usage() {
  return "Usage:\n  ./step2_main <source>\n"
         "  <source> = video path (e.g. data/videos/arm.avi) or camera index (e.g. 0)\n";
}

int main(int argc, char** argv) {
  try {
    // 1) 输入源
    std::string source = (argc > 1) ? argv[1] : "data/videos/arm.avi";
    if (argc <= 1) std::cout << "[step2] No source specified. Default: " << source << "\n" << usage();

    // 2) 加载相机参数 + YAML
    Camera cam = LoadCamera("config/camera.yaml");
    YAML::Node P  = YAML::LoadFile("config/params.yaml");

    // 3) 打开视频/相机
    VideoReader vr;
    if (!vr.open(source)) {
      std::cerr << "[step2] Open source failed: " << source << "\n";
      return 2;
    }

    cv::Mat frame;
    if (!vr.read(frame) || frame.empty()) {
      std::cerr << "[step2] Read first frame failed.\n";
      return 3;
    }
    cv::Size sz = vr.size();
    AdaptIntrinsicsToFrame(cam, sz.width, sz.height);

    // 4) 轨迹条（灰度通路参数）
    int use_otsu = 0;   // 0 固定阈值；1 Otsu
    int thresh   = 150; // 固定阈值
    int morph_k  = 3;   // 形态学核(奇数)
    cv::namedWindow("step2", 1);
    cv::createTrackbar("use_otsu", "step2", &use_otsu, 1);
    cv::createTrackbar("thresh",   "step2", &thresh,   255);
    cv::createTrackbar("morph_k",  "step2", &morph_k,  15);

  bool paused = false;
  bool exit_by_user = false;
  double t_prev = cv::getTickCount();
  double fps_meas = 0.0;

    for (;;) {
      if (!paused) {
        if (!vr.read(frame) || frame.empty()) {
          if (!vr.isCamera()) {
            if (!vr.reset()) {
              std::cerr << "[step2] Failed to rewind video source.\n";
              break;
            }
            if (!vr.read(frame) || frame.empty()) {
              std::cerr << "[step2] Failed to loop video source after reset.\n";
              break;
            }
            std::cout << "[step2] Looping video from start.\n";
          } else {
            std::cerr << "[step2] Camera frame grab failed.\n";
            break;
          }
        }
      }

      // ===== 灰度二值通路 =====
      cv::Mat bin_gray;
      BuildGrayBinary(frame, thresh, (use_otsu!=0), morph_k, bin_gray);
      auto cands_gray = ExtractLightBarCandidates(bin_gray, P);
      cv::Mat view_gray = frame.clone();
      DrawLBCandidates(view_gray, cands_gray);

      // ===== HSV 自动判色通路 =====
  auto cr = SelectColor(frame, P);
      auto cands_hsv = ExtractLightBarCandidates(cr.mask, P);
      cv::Mat view_hsv = frame.clone();
      DrawLBCandidates(view_hsv, cands_hsv);

      // ===== 拼图显示 =====
      cv::Mat mask_bgr; cv::cvtColor(cr.mask, mask_bgr, cv::COLOR_GRAY2BGR);
      cv::Mat left, right, view;
      cv::hconcat(view_gray, mask_bgr, left);     // 左：灰度候选 + 当前颜色mask
      cv::hconcat(view_hsv, frame,     right);    // 右：HSV候选 + 原图
    cv::hconcat(left, right, view);

      // FPS（测量）
      double t_now = cv::getTickCount();
      double dt = (t_now - t_prev) / cv::getTickFrequency();
      t_prev = t_now;
      double fps_inst = (dt > 1e-6) ? 1.0 / dt : 0.0;
      fps_meas = (fps_meas<=0.0) ? fps_inst : 0.9*fps_meas + 0.1*fps_inst;

      // 叠字
      char line1[256], line2[256];
      std::snprintf(line1, sizeof(line1),
                    "GRAY: use_otsu=%d thresh=%d morph_k=%d  cand=%zu",
                    use_otsu, thresh, morph_k, cands_gray.size());
      std::snprintf(line2, sizeof(line2),
                    "HSV:  color=%s  cand=%zu   FPS cap=%.1f meas=%.1f   %s",
                    (cr.color==TeamColor::RED? "RED" : (cr.color==TeamColor::BLUE? "BLUE":"UNK")),
                    cands_hsv.size(), vr.fps(), fps_meas, paused?"[PAUSE]":"");
      cv::putText(view, line1, {20,40},  cv::FONT_HERSHEY_SIMPLEX, 0.8, {0,255,0}, 2);
      cv::putText(view, line2, {20,80},  cv::FONT_HERSHEY_SIMPLEX, 0.8, {255,255,0}, 2);
      cv::putText(view, "Keys: ESC quit | P pause | R reload params.yaml", {20,120},
                  cv::FONT_HERSHEY_SIMPLEX, 0.7, {255,255,255}, 2);

      cv::imshow("step2", view);
      int k = cv::waitKey(1);
      if (k == 27) { exit_by_user = true; break; } // ESC
      else if (k=='p' || k=='P') paused = !paused;
      else if (k=='r' || k=='R') {
        try {
          P = YAML::LoadFile("config/params.yaml");
          std::cout << "[step2] params.yaml reloaded.\n";
        } catch (...) { std::cerr << "[step2] reload params failed.\n"; }
      }
    }

  cv::destroyWindow("step2");
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[step2] Exception: " << e.what() << "\n";
    return 1;
  }
}
