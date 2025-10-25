#include "armor/lightbar_detector.hpp"
#include <algorithm>
#include <cmath>

namespace armor {

// 本模块自带最小形态学与工具
static cv::Mat makeKernel(int k){
  int kk = std::max(1, k | 1);
  return cv::getStructuringElement(cv::MORPH_ELLIPSE, {kk, kk});
}
static void MorphOpenClose(cv::Mat& bin, int ksize){
  if (ksize<=0) return;
  cv::Mat k = makeKernel(ksize);
  cv::morphologyEx(bin, bin, cv::MORPH_OPEN,  k);
  cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k);
}

void BuildGrayBinary(const cv::Mat& bgr, int thresh, bool use_otsu, int morph_k, cv::Mat& bin){
  CV_Assert(!bgr.empty() && bgr.type()==CV_8UC3);
  cv::Mat gray; cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  if (use_otsu) cv::threshold(gray, bin, 0,   255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  else          cv::threshold(gray, bin, thresh, 255, cv::THRESH_BINARY);
  MorphOpenClose(bin, morph_k);
}

static inline double rad2deg(double r){ return r * 180.0 / CV_PI; }
static double longEdgeUprightDeg(const cv::RotatedRect& rr){
  cv::Point2f p[4]; rr.points(p);
  int idx=0; double best=-1;
  for (int i=0;i<4;i++){
    cv::Point2f a=p[i], b=p[(i+1)&3];
    double l2=(b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y);
    if (l2>best){best=l2; idx=i;}
  }
  cv::Point2f a=p[idx], b=p[(idx+1)&3], v=b-a;
  double vn = std::sqrt(v.x*v.x+v.y*v.y);
  if (vn<1e-6) return 90.0;
  double cosang = v.y / vn; // 与(0,1)夹角
  cosang = std::clamp(cosang, -1.0, 1.0);
  return rad2deg(std::acos(std::fabs(cosang))); // 0..90
}

std::vector<LBBox> ExtractLightBarCandidates(const cv::Mat& bin, const YAML::Node& P){
  CV_Assert(!bin.empty() && bin.type()==CV_8UC1);

  double min_area  = P["lightbar"]["min_area"] .as<double>(20);
  double min_ratio = P["lightbar"]["min_ratio"].as<double>(3.0);
  double max_ratio = P["lightbar"]["max_ratio"].as<double>(25.0);
  double ang_upr   = P["lightbar"]["angle_upright_deg"].as<double>(25.0);

  std::vector<std::vector<cv::Point>> cs;
  cv::findContours(bin, cs, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<LBBox> out; out.reserve(cs.size());
  for (auto& c : cs) {
    double a = std::fabs(cv::contourArea(c));
    if (a < min_area) continue;

    cv::RotatedRect rr = cv::minAreaRect(c);
    float w = rr.size.width, h = rr.size.height;
    if (w<1.f || h<1.f) continue;

    float L = std::max(w,h), S = std::min(w,h);
    double ratio = L / std::max(1.f, S);
    if (ratio < min_ratio || ratio > max_ratio) continue;

    double up = longEdgeUprightDeg(rr);
    if (up > ang_upr) continue;

    LBBox b; b.rrect=rr; b.area=a; b.ratio=ratio; b.upright_deg=up;
    out.push_back(b);
  }
  return out;
}

void DrawLBCandidates(cv::Mat& img, const std::vector<LBBox>& cands,
                      const cv::Scalar& color, int thickness){
  for (auto& b : cands) {
    cv::Point2f q[4]; b.rrect.points(q);
    for (int i=0;i<4;i++)
      cv::line(img, q[i], q[(i+1)&3], color, thickness, cv::LINE_AA);
    char txt[96];
    std::snprintf(txt, sizeof(txt), "r=%.1f ang=%.1f", b.ratio, b.upright_deg);
    cv::putText(img, txt, q[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_AA);
  }
}
static inline float len2(const cv::Point2f& a){ return a.x*a.x + a.y*a.y; }

void EnrichLightBarsWithEndpoints(std::vector<LBBox>& bars) {
  for (auto& b : bars) {
    // 1) 找长边方向
    cv::Point2f q[4]; b.rrect.points(q);
    int idx_long = 0; float best = -1.f;
    for (int i=0;i<4;i++){
      cv::Point2f d = q[(i+1)&3] - q[i];
      float L2 = len2(d);
      if (L2 > best) { best = L2; idx_long = i; }
    }
    cv::Point2f a = q[idx_long], c = q[(idx_long+1)&3];
    cv::Point2f v = c - a;              // 长边方向
    float L = std::sqrt(len2(v));       // 长边长度
    if (L < 1e-3f) { b.ep[0]=b.ep[1]=b.rrect.center; b.height=0; continue; }
    cv::Point2f vunit = v * (1.0f / L); // 单位方向

    // 2) 端点：中心 ± 长边一半
    cv::Point2f center = b.rrect.center;
    b.ep[0] = center - 0.5f * L * vunit;
    b.ep[1] = center + 0.5f * L * vunit;

    // 确保 ep[0] 在“更上方”，便于后续一致性
    if (b.ep[0].y > b.ep[1].y) std::swap(b.ep[0], b.ep[1]);

    // 3) 记录额外属性
    b.center = center;
    b.height = L;
    // 竖直度角（与竖直 (0,1) 的夹角）
  double cosang = vunit.y; // dot(vunit, (0,1))
  cosang = std::clamp(cosang, -1.0, 1.0);
    double up = rad2deg(std::acos(std::fabs(cosang))); // 0..90
    b.tilt_deg = static_cast<float>(up);
  }
}



} // namespace armor
