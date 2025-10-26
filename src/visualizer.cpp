#include "armor/visualizer.hpp"

namespace armor {

void PutTextShadow(cv::Mat& img, const std::string& text, cv::Point org,
                   double scale, const cv::Scalar& fg) {
  cv::putText(img, text, org+cv::Point(1,1), cv::FONT_HERSHEY_SIMPLEX, scale, {0,0,0}, 3, cv::LINE_AA);
  cv::putText(img, text, org,                 cv::FONT_HERSHEY_SIMPLEX, scale, fg,       2, cv::LINE_AA);
}

void DrawLightBar(cv::Mat& img, const LBBox& b,
                  const cv::Scalar& box_color, const cv::Scalar& ep_color) {
  cv::Point2f q[4]; b.rrect.points(q);
  for (int i=0;i<4;i++)
    cv::line(img, q[i], q[(i+1)&3], box_color, 3, cv::LINE_AA);
  cv::circle(img, b.ep[0], 6, ep_color, -1, cv::LINE_AA);
  cv::circle(img, b.ep[1], 6, ep_color, -1, cv::LINE_AA);

  // Remove parameter display as per requirements
  // char txt[64];
  // std::snprintf(txt, sizeof(txt), "r=%.1f up=%.1f", b.ratio, b.upright_deg);
  // PutTextShadow(img, txt, q[1], 0.7, {0,255,255});
}

void DrawBestPair(cv::Mat& img, const std::vector<LBBox>& bars, const PairLB& p,
                  const cv::Scalar& color) {
  if (p.li<0 || p.ri<0) return;
  const auto& L = bars[p.li];
  const auto& R = bars[p.ri];
  cv::line(img, L.center, R.center, color, 4, cv::LINE_AA);
  // Remove parameter display as per requirements
  // char txt[64]; std::snprintf(txt, sizeof(txt), "best=%.2f", p.score);
  // PutTextShadow(img, txt, (L.center+R.center)*0.5 + cv::Point2f(0,-15), 0.9, color);
}

} // namespace armor
