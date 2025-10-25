#include "armor/io.hpp"
#include "armor/color_selector.hpp"
#include "armor/lightbar_detector.hpp"
#include "armor/pairing.hpp"
#include "armor/visualizer.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <numeric>

using namespace armor;

int main(int argc, char** argv) {
  std::string source = (argc>1)? argv[1] : "data/videos/arm.avi";
  Camera cam = LoadCamera("config/camera.yaml");
  YAML::Node P = YAML::LoadFile("config/params.yaml");

  VideoReader vr; if(!vr.open(source)){ std::cerr<<"open failed\n"; return 2; }
  cv::Mat frame; if(!vr.read(frame) || frame.empty()){ std::cerr<<"first frame fail\n"; return 3; }
  cv::Size sz = vr.size(); AdaptIntrinsicsToFrame(cam, sz.width, sz.height);

  bool paused=false;
  int frame_idx = 0;
  for(;;){
    if(!paused){
      if(!vr.read(frame) || frame.empty()){
        if(!vr.isCamera()){
          if(!vr.reset()){
            std::cerr << "[step3] Failed to rewind video source.\n";
            break;
          }
          if(!vr.read(frame) || frame.empty()){
            std::cerr << "[step3] Failed to loop video source after reset.\n";
            break;
          }
          std::cout << "[step3] Looping video from start.\n";
        }else{
          std::cerr << "[step3] Camera frame grab failed.\n";
          break;
        }
      }
    }

    frame_idx++;

    // 1) 自动判色，得到当前颜色掩膜
    auto cr = SelectColor(frame, P);

    // 1.5) 灰度二值作为结构约束
    int gray_thresh = P["gray"]["thresh"].as<int>(150);
    bool gray_use_otsu = P["gray"]["use_otsu"].as<bool>(false);
    int gray_morph = P["gray"]["morph_k"].as<int>(3);
    cv::Mat bin_gray;
    BuildGrayBinary(frame, gray_thresh, gray_use_otsu, gray_morph, bin_gray);

    cv::Mat bin = bin_gray;
    if(cr.color != TeamColor::UNKNOWN){
      cv::bitwise_and(bin_gray, cr.mask, bin);
      if(cv::countNonZero(bin) < 10){
        bin = cr.mask.clone();
      }
    }

    // 1.6) 额外形态学收缩/膨胀稳定轮廓
    if(!bin.empty()){
      int post_k = P["lightbar"]["post_morph"].as<int>(3);
      if(post_k > 1){
        int kk = std::max(1, post_k | 1);
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {kk, kk});
        cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(bin, bin, cv::MORPH_OPEN,  kernel);
      }
    }

    // 2) 提取候选 + 端点
    auto bars = ExtractLightBarCandidates(bin, P);
    if(bars.empty()){
      YAML::Node P_relaxed = YAML::Clone(P);
      auto light = P_relaxed["lightbar"];
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

    // 3) 配对
    auto pairs = PairLights(bars, P);
    if(pairs.empty() && bars.size() >= 2){
      // 兜底策略：挑选面积最大的两个灯条做临时配对，保证可视化
      std::vector<int> idx(bars.size());
      std::iota(idx.begin(), idx.end(), 0);
      std::sort(idx.begin(), idx.end(), [&](int a, int b){ return bars[a].area > bars[b].area; });
      int li = idx[0], ri = idx[1];
      if (bars[li].center.x > bars[ri].center.x) std::swap(li, ri);
      PairLB fallback; fallback.li = li; fallback.ri = ri; fallback.score = 0.0;
      pairs.push_back(fallback);
    }

    static int frame_idx = 0;
    frame_idx++;
    if(frame_idx % 120 == 0){
      std::cout << "[step3] frame=" << frame_idx
                << " bars=" << bars.size()
                << " pairs=" << pairs.size()
                << " scoreR=" << cr.score_red
                << " scoreB=" << cr.score_blue << "\n";
    }

    // 4) 可视化
    cv::Mat show = frame.clone();
    for (auto& b : bars) DrawLightBar(show, b);
    if (!pairs.empty()) DrawBestPair(show, bars, pairs[0]);

    char line[256];
    std::snprintf(line, sizeof(line), "color=%s  bars=%zu  pairs=%zu",
      (cr.color==TeamColor::RED? "RED" : (cr.color==TeamColor::BLUE? "BLUE":"UNK")),
      bars.size(), pairs.size());
    PutTextShadow(show, line, {20,40});

    PutTextShadow(show, "Keys: ESC quit | P pause | R reload params.yaml", {20,80}, 0.7);

    double scale = P["display"]["scale"].as<double>(1.5);
    if (scale > 1.01) {
      cv::resize(show, show, cv::Size(), scale, scale, cv::INTER_LINEAR);
    }

    cv::imshow("step3", show);
    int k=cv::waitKey(1);
    if(k==27) break;
    else if(k=='p'||k=='P') paused=!paused;
    else if(k=='r'||k=='R'){ try{ P=YAML::LoadFile("config/params.yaml"); std::cout<<"[step3] reloaded.\n"; }catch(...){} }
  }
  return 0;
}
