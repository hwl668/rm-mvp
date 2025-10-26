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
#include <array>
#include <algorithm>
#include <numeric>
#include <filesystem>
#include <deque>

using namespace armor;
namespace fs = std::filesystem;

// Structure to store detection state for temporal consistency
struct DetectionState {
  bool has_detection = false;
  PairLB best_pair;
  std::array<cv::Point2f, 4> corners;
  Pose pose;
  double score = 0.0;
};

static void DrawAxes(cv::Mat& img, const cv::Mat& K, const cv::Mat& dist,
         const cv::Vec3d& rvec, const cv::Vec3d& tvec, double axis_len_m)
{
  std::vector<cv::Point3f> axis3d = {
      {0,0,0}, { (float)axis_len_m,0,0 }, {0,(float)axis_len_m,0}, {0,0,(float)axis_len_m} };
  std::vector<cv::Point2f> axis2d;
  cv::projectPoints(axis3d, rvec, tvec, K, dist, axis2d);
  auto O=axis2d[0], X=axis2d[1], Y=axis2d[2], Z=axis2d[3];
  cv::line(img, O, X, {0,0,255}, 2, cv::LINE_AA);   // X 红
  cv::line(img, O, Y, {0,255,0}, 2, cv::LINE_AA);   // Y 绿
  cv::line(img, O, Z, {255,0,0}, 2, cv::LINE_AA);   // Z 蓝
}

static void BannerParams(const YAML::Node& P) {
  auto pr=[&](const char* k, const YAML::Node& n){
    if(n[k]) std::cout<<"  "<<k<<" = "<<n[k]<<"\n";
  };
  std::cout << "\n==== ARMOR VISION (build " << __DATE__ << " " << __TIME__ << ") ====\n";
  std::cout << "[armor.size(m)]\n";
  pr("width", P["armor"]); pr("height", P["armor"]);
  std::cout << "[pnp]\n";
  pr("reproj_thresh_px", P["pnp"]); pr("axis_len_m", P["pnp"]);
  std::cout << "====================================================\n";
}

int main(int argc, char** argv) try {
  // 1) 输入源与配置
  std::string source = (argc>1)? argv[1] : "data/videos/arm.avi";
  
  // Create output directory
  const std::string out_dir = "out";
  if (!fs::exists(out_dir)) {
    fs::create_directories(out_dir);
    std::cout << "[main] Created output directory: " << out_dir << "\n";
  }
  
  Camera cam = LoadCamera("config/camera.yaml");
  YAML::Node P = YAML::LoadFile("config/params.yaml");
  BannerParams(P);

  const double Wm = P["armor"]["width"].as<double>(0.130);   // 你的尺寸：宽 0.130 m
  const double Hm = P["armor"]["height"].as<double>(0.050);  // 高 0.050 m
  const double AX = P["pnp"]["axis_len_m"].as<double>(0.08); // 坐标轴长度
  const double RE = P["pnp"]["reproj_thresh_px"].as<double>(3.0);

  // 2) 打开视频/相机
  VideoReader vr;
  if(!vr.open(source)){ std::cerr<<"[main] open failed: "<<source<<"\n"; return 2; }
  cv::Mat frame;
  if(!vr.read(frame)||frame.empty()){ std::cerr<<"[main] first frame fail.\n"; return 3; }
  cv::Size sz = vr.size();
  AdaptIntrinsicsToFrame(cam, sz.width, sz.height);

  // Temporal smoothing
  std::deque<DetectionState> detection_history;
  const int max_history = 5;
  int frame_count = 0;

  bool paused=false;
  double fps_meas=0.0, t_prev=cv::getTickCount();

  for(;;){
    if(!paused){
      if(!vr.read(frame) || frame.empty()){
        if(!vr.isCamera()){
          if(!vr.reset()){
            std::cerr << "[main] Failed to rewind video source.\n";
            break;
          }
          if(!vr.read(frame) || frame.empty()){
            std::cerr << "[main] Failed to loop video source after reset.\n";
            break;
          }
          std::cout << "[main] Looping video from start.\n";
        } else {
          std::cerr << "[main] Camera frame grab failed.\n";
          break;
        }
      }
      frame_count++;
    }

    // ============ 颜色自动判定（HSV） ============
    ColorResult cr = SelectColor(frame, P);

    // 颜色掩膜 + 灰度约束
    const YAML::Node gray_cfg = P["gray"];
    int gray_thresh = gray_cfg ? gray_cfg["thresh"].as<int>(150) : 150;
    bool gray_use_otsu = gray_cfg ? gray_cfg["use_otsu"].as<bool>(false) : false;
    int gray_morph = gray_cfg ? gray_cfg["morph_k"].as<int>(3) : 3;
    cv::Mat bin_gray;
    BuildGrayBinary(frame, gray_thresh, gray_use_otsu, gray_morph, bin_gray);

    cv::Mat bin = bin_gray;
    if(cr.color != TeamColor::UNKNOWN){
      cv::bitwise_and(bin_gray, cr.mask, bin);
      if(cv::countNonZero(bin) < 10)
        bin = cr.mask.clone();
    }

    // Improved morphology to reduce reflection noise
    int post_k = P["lightbar"]["post_morph"].as<int>(3);
    if(post_k > 1){
      int kk = std::max(1, post_k | 1);
      cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kk, kk});
      cv::morphologyEx(bin, bin, cv::MORPH_OPEN, kernel);  // Open first to remove noise
      cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel); // Then close to fill gaps
    }

    // ============ 候选灯条 + 端点 ============
    auto bars = ExtractLightBarCandidates(bin, P);
    if(bars.empty() && !detection_history.empty()){
      YAML::Node P_relaxed = YAML::Clone(P);
      YAML::Node light = P_relaxed["lightbar"];
      double min_area  = light["min_area"].as<double>(8.0) * 0.8;
      double min_ratio = light["min_ratio"].as<double>(2.0) * 0.9;
      double max_ratio = light["max_ratio"].as<double>(10.0) * 1.2;
      double ang_upr   = light["angle_upright_deg"].as<double>(30.0) + 10.0;
      light["min_area"] = std::max(4.0, min_area);
      light["min_ratio"] = std::max(1.5, min_ratio);
      light["max_ratio"] = std::max(10.0, max_ratio);
      light["angle_upright_deg"] = std::min(45.0, ang_upr);
      bars = ExtractLightBarCandidates(bin, P_relaxed);
    }
    EnrichLightBarsWithEndpoints(bars);

    // ============ 配对 - ONLY ONE PAIR ============
    auto pairs = PairLights(bars, P);
    
    // Keep only the best pair
    if(pairs.size() > 1){
      pairs.resize(1);
    }
    
    if(pairs.empty() && bars.size() >= 2 && !detection_history.empty()){
      std::vector<int> idx(bars.size());
      std::iota(idx.begin(), idx.end(), 0);
      std::sort(idx.begin(), idx.end(), [&](int a, int b){ return bars[a].area > bars[b].area; });
      int li = idx[0], ri = idx[1];
      if(bars[li].center.x > bars[ri].center.x) std::swap(li, ri);
      PairLB fallback; fallback.li = li; fallback.ri = ri; fallback.score = 0.5;
      pairs.push_back(fallback);
    }

    // ============ 可视化底图 ============
    cv::Mat show = frame.clone();
    for(const auto& b: bars) DrawLightBar(show, b);
    if(!pairs.empty()) DrawBestPair(show, bars, pairs[0]);

    // Detection state for this frame
    DetectionState current_detection;
    current_detection.has_detection = false;

    // ============ 若有最佳配对：角点外推 + PnP ============
    bool have_pose=false; Pose pose; std::array<cv::Point2f,4> cimg;
    if(!pairs.empty()){
      const LBBox& L = bars[pairs[0].li];
      const LBBox& R = bars[pairs[0].ri];

      if(EstimatePlateCorners(L, R, P, cimg)){
        auto toPoint = [](const cv::Point2f& p){ return cv::Point(cvRound(p.x), cvRound(p.y)); };
        std::vector<cv::Point> outline;
        outline.reserve(4);
        for(const auto& pt : cimg) outline.push_back(toPoint(pt));
        cv::polylines(show, std::vector<std::vector<cv::Point>>{outline}, true, {0,255,0}, 2, cv::LINE_AA);
        for(int i=0;i<4;i++){
          cv::Point pi = toPoint(cimg[i]);
          cv::circle(show, pi, 5, {0,255,0}, -1, cv::LINE_AA);
          cv::putText(show, std::to_string(i), pi + cv::Point(3,-3), cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1, cv::LINE_AA);
        }

        // PnP
        pose = SolveArmorPnP(cimg, cam.K, cam.dist, Wm, Hm, RE);
        have_pose = pose.ok && pose.reproj_err <= RE;

        if(have_pose){
          // Store detection state
          current_detection.has_detection = true;
          current_detection.best_pair = pairs[0];
          current_detection.corners = cimg;
          current_detection.pose = pose;
          current_detection.score = pairs[0].score;

          DrawAxes(show, cam.K, cam.dist, pose.rvec, pose.tvec, AX);

          // 打印位姿
          char buf[256];
          double X=pose.tvec[0], Y=pose.tvec[1], Z=pose.tvec[2];
          std::snprintf(buf, sizeof(buf),
              "PnP OK | X=%.3f Y=%.3f Z=%.3f m  | Err=%.2f px  Inl=%d",
              X, Y, Z, pose.reproj_err, pose.inliers);
          PutTextShadow(show, buf, {20, 120}, 0.8, {0,255,0});
        } else {
          char buf[128];
          std::snprintf(buf, sizeof(buf), "PnP FAIL | Err=%.2f px  Inl=%d", pose.reproj_err, pose.inliers);
          PutTextShadow(show, buf, {20, 120}, 0.8, {0,200,255});
        }
      }
    }

    // Use history for temporal consistency if no detection
    if(!current_detection.has_detection && !detection_history.empty()){
      for(auto it = detection_history.rbegin(); it != detection_history.rend(); ++it){
        if(it->has_detection){
          current_detection = *it;
          DrawAxes(show, cam.K, cam.dist, current_detection.pose.rvec, 
                   current_detection.pose.tvec, AX);
          auto toPoint = [](const cv::Point2f& p){ return cv::Point(cvRound(p.x), cvRound(p.y)); };
          std::vector<cv::Point> outline;
          outline.reserve(4);
          for(const auto& pt : current_detection.corners) outline.push_back(toPoint(pt));
          cv::polylines(show, std::vector<std::vector<cv::Point>>{outline}, true,
                        {0, 200, 200}, 2, cv::LINE_AA);
          char buf[128];
          std::snprintf(buf, sizeof(buf), "Using previous detection (temporal smoothing)");
          PutTextShadow(show, buf, {20, 120}, 0.8, {0,200,200});
          break;
        }
      }
    }

    // Update history
    detection_history.push_back(current_detection);
    if(detection_history.size() > max_history){
      detection_history.pop_front();
    }

    // ============ 统计/FPS/提示 ============
    double t_now=cv::getTickCount();
    double dt=(t_now-t_prev)/cv::getTickFrequency(); t_prev=t_now;
    double fps_inst = (dt>1e-6)? 1.0/dt : 0.0; fps_meas = (fps_meas<=0)? fps_inst : 0.9*fps_meas+0.1*fps_inst;

    char line1[256], line2[256], line3[256];
    std::snprintf(line1, sizeof(line1),
      "BUILD %s %s | color=%s | bars=%zu pairs=1 (best only) | fps cap=%.1f meas=%.1f",
      __DATE__, __TIME__,
      (cr.color==TeamColor::RED?"RED":(cr.color==TeamColor::BLUE?"BLUE":"UNK")),
      bars.size(), vr.fps(), fps_meas);
    PutTextShadow(show, line1, {20,40}, 0.8);

    std::snprintf(line2, sizeof(line2),
      "Keys: ESC quit | P pause | R reload params | S save debug | frame=%d", frame_count);
    PutTextShadow(show, line2, {20,80}, 0.7);

    cv::imshow("armor_vision", show);
    int k=cv::waitKey(1);
    if(k==27) break;
    else if(k=='p'||k=='P') paused = !paused;
    else if(k=='r'||k=='R'){
      try{ P = YAML::LoadFile("config/params.yaml"); BannerParams(P); }
      catch(...){ std::cerr<<"[main] reload params failed.\n"; }
    }
    else if(k=='s'||k=='S'){
      std::string out_path = out_dir + "/frame_" + std::to_string(frame_count) + ".jpg";
      cv::imwrite(out_path, show);
      cv::imwrite(cv::format("%s/mask_%d.png", out_dir.c_str(), frame_count), cr.mask);
      std::cout<<"[main] saved debug images to " << out_dir << "/\n";
    }
  }

  return 0;
}
catch(const std::exception& e){
  std::cerr << "[main] Exception: " << e.what() << "\n"; return 1;
}
