#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "armor.hpp"

namespace auto_aim {

class Solver
{
public:
  explicit Solver(const std::string & config_path);

  // PnP：把位姿结果写回 Armor（xyz_in_* / ypr_in_* / ypd_in_world）
  void solve(Armor & armor) const;

  // 你的 main 里要用来画坐标轴
  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints) const;

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_; // 默认 I
};

} // namespace auto_aim

#endif // AUTO_AIM__SOLVER_HPP
