#include "armor/pairing.hpp"
#include <algorithm>
#include <cmath>

namespace armor {

static inline double clamp01(double x){ return std::max(0.0, std::min(1.0, x)); }

std::vector<PairLB> PairLights(const std::vector<LBBox>& B, const YAML::Node& P) {
  if (B.size() < 2) return {};

  // 参数
  const double h_diff_max   = P["pair"]["height_diff_max"].as<double>(0.30);   // 相对高度差
  const double len_diff_max = P["pair"]["length_diff_max"].as<double>(0.30);   // 相对长度差
  const double tilt_diff_max= P["pair"]["tilt_diff_max_deg"].as<double>(15.0); // 倾角差(度)
  const double k_max        = P["pair"]["slope_max"].as<double>(0.30);         // 中心连线斜率阈值
  const double gap_min      = P["pair"]["gap_min_ratio"].as<double>(0.6);      // 中心距/平均高
  const double gap_max      = P["pair"]["gap_max_ratio"].as<double>(3.0);

  // 权重（归一化后做 0~1 的奖励）
  const double w_h   = P["pair"]["w_height"].as<double>(0.25);
  const double w_len = P["pair"]["w_length"].as<double>(0.20);
  const double w_tlt = P["pair"]["w_tilt"].as<double>(0.20);
  const double w_k   = P["pair"]["w_slope"].as<double>(0.15);
  const double w_gap = P["pair"]["w_gap"].as<double>(0.20);

  std::vector<PairLB> out;

  for (int i = 0; i < (int)B.size(); ++i) {
    for (int j = i+1; j < (int)B.size(); ++j) {
      int L = (B[i].center.x <= B[j].center.x) ? i : j;
      int R = (L==i)? j : i;

      const auto& A = B[L];
      const auto& C = B[R];

      double hA = std::max(1.f, A.height);
      double hC = std::max(1.f, C.height);
      double hmean = 0.5*(hA+hC);
      double hdiff = std::abs(hA - hC) / hmean;

      // 长度用长边（与 height 同义）
      double ldiff = hdiff;

      double tdiff = std::abs(A.tilt_deg - C.tilt_deg);

      // 中心连线斜率
      double dx = std::max(1.f, std::abs(C.center.x - A.center.x));
      double dy = (C.center.y - A.center.y);
      double k  = std::abs(dy / dx);

      // 间距比
      double gap = std::sqrt(dx*dx + dy*dy) / hmean;

      // 硬过滤
      if (hdiff > h_diff_max) continue;
      if (ldiff > len_diff_max) continue;
      if (tdiff > tilt_diff_max) continue;
      if (k     > k_max) continue;
      if (gap < gap_min || gap > gap_max) continue;

      // 奖励（0~1）
      double s_h   = 1.0 - clamp01(hdiff / h_diff_max);
      double s_len = 1.0 - clamp01(ldiff / len_diff_max);
      double s_tlt = 1.0 - clamp01(tdiff / tilt_diff_max);
      double s_k   = 1.0 - clamp01(k / k_max);
      // gap 居中更好：以 [gap_min, gap_max] 的中点做高斯型（简化为三角）
      double g_mid = 0.5*(gap_min + gap_max);
      double g_rad = 0.5*(gap_max - gap_min);
      double s_gap = 1.0 - std::min(1.0, std::abs(gap - g_mid) / g_rad);

      double score = w_h*s_h + w_len*s_len + w_tlt*s_tlt + w_k*s_k + w_gap*s_gap;

      PairLB p; p.li = L; p.ri = R; p.score = score;
      out.push_back(p);
    }
  }

  std::sort(out.begin(), out.end(), [](const PairLB& a, const PairLB& b){ return a.score > b.score; });
  return out;
}

} // namespace armor
