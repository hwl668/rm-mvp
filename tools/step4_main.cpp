#include "armor/io.hpp"
#include "armor/color_selector.hpp"
#include "armor/lightbar_detector.hpp"
#include "armor/pairing.hpp"
#include "armor/plate_corners.hpp"
#include "armor/pnp_solver.hpp"
#include "armor/visualizer.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <numeric>

using namespace armor;

namespace {

void DrawPoseAxes(cv::Mat& img,
                  const cv::Mat& K,
                  const cv::Mat& dist,
                  const Pose& pose,
                  double axis_len_m) {
  std::vector<cv::Point3f> axis3d = {
    {0.f, 0.f, 0.f},
    {(float)axis_len_m, 0.f, 0.f},
    {0.f, (float)axis_len_m, 0.f},
    {0.f, 0.f, (float)axis_len_m}
  };
  std::vector<cv::Point2f> axis2d;
  cv::projectPoints(axis3d, pose.rvec, pose.tvec, K, dist, axis2d);
  if (axis2d.size() != 4) return;

  cv::line(img, axis2d[0], axis2d[1], {0, 0, 255}, 2, cv::LINE_AA);
  cv::line(img, axis2d[0], axis2d[2], {0, 255, 0}, 2, cv::LINE_AA);
  cv::line(img, axis2d[0], axis2d[3], {255, 0, 0}, 2, cv::LINE_AA);

  cv::circle(img, axis2d[0], 6, {0, 255, 255}, -1, cv::LINE_AA);
  PutTextShadow(img, "O", axis2d[0] + cv::Point2f(6.f, -6.f), 0.6, {0, 255, 255});
}

} // namespace

int main(int argc, char** argv) try {
  std::string source = (argc > 1) ? argv[1] : "data/videos/arm.avi";

  Camera cam = LoadCamera("config/camera.yaml");
  YAML::Node P = YAML::LoadFile("config/params.yaml");

  const double armor_width  = P["armor"]["width"].as<double>(0.130);
  const double armor_height = P["armor"]["height"].as<double>(0.050);
  const double axis_len     = P["pnp"]["axis_len_m"].as<double>(0.08);
  const double reproj_thr   = P["pnp"]["reproj_thresh_px"].as<double>(3.0);

  VideoReader vr;
  if (!vr.open(source)) {
    std::cerr << "[step4] Failed to open source: " << source << "\n";
    return 2;
  }

  cv::Mat frame;
  if (!vr.read(frame) || frame.empty()) {
    std::cerr << "[step4] Failed to grab first frame.\n";
    return 3;
  }
  cv::Size sz = vr.size();
  AdaptIntrinsicsToFrame(cam, sz.width, sz.height);

  cv::namedWindow("Pose Viewer", cv::WINDOW_NORMAL);
  cv::resizeWindow("Pose Viewer", 1280, 960);

  bool paused = false;
  bool ended = false;

  cv::Mat current = frame.clone();

  cv::Mat last_processed_frame;
  bool should_exit = false;
  
  for (;;) {
    if (!paused && !ended) {
      cv::Mat next;
      if (!vr.read(next) || next.empty()) {
        ended = true;
        paused = true;
        std::cout << "[step4] Reached end of video.\n";
      } else {
        current = next;
      }
    }

    cv::Mat show = current.clone();

    // === Color + binary pipeline ===
    ColorResult cr = SelectColor(current, P);

    const YAML::Node gray_cfg = P["gray"];
    int gray_thresh   = gray_cfg ? gray_cfg["thresh"].as<int>(150) : 150;
    bool gray_use_otsu = gray_cfg ? gray_cfg["use_otsu"].as<bool>(false) : false;
    int gray_morph    = gray_cfg ? gray_cfg["morph_k"].as<int>(3) : 3;
    cv::Mat bin_gray;
    BuildGrayBinary(current, gray_thresh, gray_use_otsu, gray_morph, bin_gray);

    cv::Mat bin = bin_gray;
    if (cr.color != TeamColor::UNKNOWN) {
      cv::bitwise_and(bin_gray, cr.mask, bin);
      if (cv::countNonZero(bin) < 10)
        bin = cr.mask.clone();
    }
    int post_k = P["lightbar"]["post_morph"].as<int>(3);
    if (post_k > 1) {
      int kk = std::max(1, post_k | 1);
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kk, kk));
      cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
      cv::morphologyEx(bin, bin, cv::MORPH_OPEN,  kernel);
    }

    auto bars = ExtractLightBarCandidates(bin, P);
    if (bars.empty()) {
      YAML::Node P_relaxed = YAML::Clone(P);
      YAML::Node light = P_relaxed["lightbar"];
      double min_area  = light["min_area"].as<double>(20.0) * 0.5;
      double min_ratio = light["min_ratio"].as<double>(3.0) * 0.6;
      double max_ratio = light["max_ratio"].as<double>(25.0) * 1.5;
      double ang_upr   = light["angle_upright_deg"].as<double>(25.0) + 20.0;
      light["min_area"] = std::max(4.0, min_area);
      light["min_ratio"] = std::max(1.2, min_ratio);
      light["max_ratio"] = std::max(10.0, max_ratio);
      light["angle_upright_deg"] = std::min(75.0, ang_upr);
      bars = ExtractLightBarCandidates(bin, P_relaxed);
    }
    EnrichLightBarsWithEndpoints(bars);

    auto pairs = PairLights(bars, P);
    if (pairs.empty() && bars.size() >= 2) {
      std::vector<int> idx(bars.size());
      std::iota(idx.begin(), idx.end(), 0);
      std::sort(idx.begin(), idx.end(), [&](int a, int b) {
        return bars[a].area > bars[b].area;
      });
      int li = idx[0], ri = idx[1];
      if (bars[li].center.x > bars[ri].center.x) std::swap(li, ri);
      PairLB fallback; fallback.li = li; fallback.ri = ri;
      pairs.push_back(fallback);
    }

    for (const auto& b : bars) DrawLightBar(show, b);

    bool pose_ok = false;
    Pose pose;
    std::array<cv::Point2f, 4> corners;
    if (!pairs.empty()) {
      const LBBox& L = bars[pairs[0].li];
      const LBBox& R = bars[pairs[0].ri];
      bool corners_ok = EstimatePlateCorners(L, R, P, corners);
      if (corners_ok) {
        pose = SolveArmorPnP(corners, cam.K, cam.dist,
                             armor_width, armor_height, reproj_thr);
        pose_ok = pose.ok;
        if (pose_ok) {
          DrawPoseAxes(show, cam.K, cam.dist, pose, axis_len);
          auto toPoint = [](const cv::Point2f& p) {
            return cv::Point(cvRound(p.x), cvRound(p.y));
          };
          std::vector<cv::Point> outline;
          outline.reserve(4);
          for (const auto& pt : corners) outline.push_back(toPoint(pt));
          cv::polylines(show, std::vector<std::vector<cv::Point>>{outline}, true,
                        {0, 255, 0}, 2, cv::LINE_AA);
          for (size_t i = 0; i < corners.size(); ++i) {
            cv::Point pi = toPoint(corners[i]);
            cv::circle(show, pi, 4, {0, 255, 0}, -1, cv::LINE_AA);
            cv::putText(show, std::to_string((int)i), pi + cv::Point(3, -3),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1, cv::LINE_AA);
          }
        }
      }
    }

    cv::imshow("Pose Viewer", show);
    last_processed_frame = show.clone();
    
    // Save and exit after processing the last frame
    if (ended && !should_exit) {
      should_exit = true;
      cv::imwrite("/tmp/step4_output.jpg", last_processed_frame);
      std::cout << "[step4] Video ended. Saved last frame to /tmp/step4_output.jpg\n";
      cv::waitKey(1000);
      break;
    }
    
    int delay = (paused || ended) ? 30 : 1;
    int key = cv::waitKey(delay);
    if (key == 27) break;           // ESC
    if (key == 'p' || key == 'P') {
      if (!ended) paused = !paused;
    }
  }

  return 0;
} catch (const std::exception& e) {
  std::cerr << "[step4] Exception: " << e.what() << "\n";
  return 1;
}
