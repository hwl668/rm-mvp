#include "armor/pnp_solver.hpp"
#include <vector>
#include <numeric>

namespace armor {

static double meanReprojErr(const std::vector<cv::Point3f>& obj,
                            const std::vector<cv::Point2f>& img,
                            const cv::Vec3d& rvec, const cv::Vec3d& tvec,
                            const cv::Mat& K, const cv::Mat& dist)
{
  std::vector<cv::Point2f> reproj;
  cv::projectPoints(obj, rvec, tvec, K, dist, reproj);
  double sum = 0.0;
  for(size_t i=0;i<img.size();++i){
    cv::Point2f d = reproj[i] - img[i];
    sum += std::sqrt(d.x*d.x + d.y*d.y);
  }
  return sum / std::max<size_t>(1,img.size());
}

Pose SolveArmorPnP(const std::array<cv::Point2f,4>& img_corners,
                   const cv::Mat& K, const cv::Mat& dist,
                   double width_m, double height_m,
                   double reproj_thresh_px)
{
  Pose P;
  // 3D 模型：LT,RT,RB,LB（单位：米）
  std::vector<cv::Point3f> obj = {
    { (float)(-width_m/2), (float)(-height_m/2), 0.f },
    { (float)( width_m/2), (float)(-height_m/2), 0.f },
    { (float)( width_m/2), (float)( height_m/2), 0.f },
    { (float)(-width_m/2), (float)( height_m/2), 0.f }
  };
  std::vector<cv::Point2f> img(img_corners.begin(), img_corners.end());

  cv::Vec3d rvec, tvec;
  bool ok = false;
  int inl = 0;

  // 优先 IPPE（平面）—— OpenCV 4.5.4 支持
  try {
    ok = cv::solvePnP(obj, img, K, dist, rvec, tvec, false, cv::SOLVEPNP_IPPE);
    inl = ok ? (int)img.size() : 0;
  } catch(...) { ok = false; }

  // 失败则 RANSAC(AP3P) 兜底
  if(!ok){
    std::vector<int> inliers;
    ok = cv::solvePnPRansac(obj, img, K, dist, rvec, tvec,
                            false, 100, 8.0, 0.99, inliers, cv::SOLVEPNP_AP3P);
    inl = (int)inliers.size();
  }

  if(!ok){
    P.ok = false; P.inliers = inl; P.reproj_err = 1e9;
    return P;
  }

  double err = meanReprojErr(obj, img, rvec, tvec, K, dist);

  P.rvec = rvec; P.tvec = tvec;
  P.reproj_err = err;
  P.inliers = inl;
  P.ok = (err <= reproj_thresh_px);
  return P;
}

} // namespace armor
