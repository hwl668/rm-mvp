#pragma once
#include <opencv2/opencv.hpp>
#include <array>

namespace armor {

/**
 * @brief 位姿结果
 *  - rvec/tvec : 相机坐标系下目标位姿（米）
 *  - reproj_err: 重投影均值误差（像素）
 *  - inliers   : RANSAC 内点数（IPPE 时为点数）
 *  - ok        : 是否通过误差阈值
 */
struct Pose {
  cv::Vec3d rvec, tvec;
  double reproj_err = 1e9;
  int    inliers    = 0;
  bool   ok         = false;
};

/**
 * @brief 平面装甲板 PnP 求解（优先 IPPE，失败退 RANSAC(AP3P)）
 *
 * 输入：
 *  - img_corners : 图像上四角点，顺序必须为 LT, RT, RB, LB（与 3D 模型一致）
 *  - K, dist     : 相机内参与畸变（与 img_corners 同一像素系）
 *  - width_m     : 装甲板实际宽度（米）
 *  - height_m    : 装甲板实际高度（米）
 *  - reproj_thresh_px : 重投影误差通过阈值（像素）
 *
 * 返回：
 *  - Pose 结构体（包含 rvec/tvec/误差/ok）
 */
Pose SolveArmorPnP(const std::array<cv::Point2f,4>& img_corners,
                   const cv::Mat& K, const cv::Mat& dist,
                   double width_m, double height_m,
                   double reproj_thresh_px);

} // namespace armor
