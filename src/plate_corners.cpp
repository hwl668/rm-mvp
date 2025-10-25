#include "armor/plate_corners.hpp"
#include <algorithm>
#include <cmath>

namespace armor {

// 取两条“长边”段（各返回按 y 排序：top,bottom）
static inline float len2(const cv::Point2f& p){ return p.x*p.x + p.y*p.y; }

static void longEdges(const cv::RotatedRect& rr,
                      std::array<cv::Point2f,2>& e0, // [top,bottom]
                      std::array<cv::Point2f,2>& e1) // [top,bottom]
{
  cv::Point2f q[4]; rr.points(q);
  // 找到最长的边索引 i（i->i+1）
  int idx = 0; float best=-1.f;
  for(int i=0;i<4;i++){
    cv::Point2f d = q[(i+1)&3] - q[i];
    float L2 = len2(d);
    if(L2>best){ best=L2; idx=i; }
  }
  // 与其对边
  int idx2 = (idx+2)&3;

  cv::Point2f a=q[idx], b=q[(idx+1)&3];
  cv::Point2f c=q[idx2], d=q[(idx2+1)&3];

  // 每条边按 y 排序（top,bottom）
  if(a.y<=b.y){ e0 = {a,b}; } else { e0 = {b,a}; }
  if(c.y<=d.y){ e1 = {c,d}; } else { e1 = {d,c}; }
}

// 选择“内侧长边”（left→取 x 更大；right→取 x 更小）
static std::array<cv::Point2f,2> pickInnerEdge(const cv::RotatedRect& rr, bool is_left)
{
  std::array<cv::Point2f,2> E0, E1;
  longEdges(rr, E0, E1);
  auto midx = [](const std::array<cv::Point2f,2>& e){ return 0.5f*(e[0].x+e[1].x); };
  if(is_left){
    return (midx(E0) >= midx(E1)) ? E0 : E1; // 更靠右为内侧
  }else{
    return (midx(E0) <= midx(E1)) ? E0 : E1; // 更靠左为内侧
  }
}

bool EstimatePlateCorners(const LBBox& left_in, const LBBox& right_in,
                          const YAML::Node& P,
                          std::array<cv::Point2f,4>& corners_img)
{
  // 确保左右有序（容错）
  const LBBox *Lp=&left_in, *Rp=&right_in;
  if(Lp->center.x > Rp->center.x) std::swap(Lp, Rp);

  // 读取外推比例（可为 0）
  const double top_r    = P["plate"]["top_offset_ratio"]   ? P["plate"]["top_offset_ratio"].as<double>()   : 0.0;
  const double bottom_r = P["plate"]["bottom_offset_ratio"]? P["plate"]["bottom_offset_ratio"].as<double>() : 0.0;

  // 左右灯条的“内侧长边”（各为 top,bottom）
  auto Ledge = pickInnerEdge(Lp->rrect, /*is_left=*/true);
  auto Redge = pickInnerEdge(Rp->rrect, /*is_left=*/false);

  // 沿长边方向做“上下收缩”
  auto shrink = [&](std::array<cv::Point2f,2> e)->std::array<cv::Point2f,2>{
    cv::Point2f v = e[1] - e[0];
    float L = std::sqrt(len2(v)); if(L<1e-3f) return e;
    cv::Point2f u = v * (1.0f / L);
    e[0] = e[0] + u*(float)top_r*L;     // top 下移
    e[1] = e[1] - u*(float)bottom_r*L;  // bottom 上移
    return e;
  };
  Ledge = shrink(Ledge);
  Redge = shrink(Redge);

  // 角点顺序：LT, RT, RB, LB
  corners_img[0] = Ledge[0];
  corners_img[1] = Redge[0];
  corners_img[2] = Redge[1];
  corners_img[3] = Ledge[1];

  // 基本健康性检查（避免退化）
  auto d = [](const cv::Point2f& a, const cv::Point2f& b){
    return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
  };
  double hL = d(Ledge[0], Ledge[1]);
  double hR = d(Redge[0], Redge[1]);
  double w  = d( (Ledge[0]+Ledge[1])*0.5f, (Redge[0]+Redge[1])*0.5f );
  if(hL<4 || hR<4 || w<4) return false;

  return true;
}

} // namespace armor
