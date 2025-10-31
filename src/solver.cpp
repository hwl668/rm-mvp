#include "solver.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <cmath>
#include "tools/math_tools.hpp"

namespace auto_aim {

constexpr double LIGHTBAR_LENGTH   = 56e-3;
constexpr double BIG_ARMOR_WIDTH   = 230e-3;
constexpr double SMALL_ARMOR_WIDTH = 135e-3;

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0,  BIG_ARMOR_WIDTH/2,   LIGHTBAR_LENGTH/2},
  {0, -BIG_ARMOR_WIDTH/2,   LIGHTBAR_LENGTH/2},
  {0, -BIG_ARMOR_WIDTH/2,  -LIGHTBAR_LENGTH/2},
  {0,  BIG_ARMOR_WIDTH/2,  -LIGHTBAR_LENGTH/2}
};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0,  SMALL_ARMOR_WIDTH/2,   LIGHTBAR_LENGTH/2},
  {0, -SMALL_ARMOR_WIDTH/2,   LIGHTBAR_LENGTH/2},
  {0, -SMALL_ARMOR_WIDTH/2,  -LIGHTBAR_LENGTH/2},
  {0,  SMALL_ARMOR_WIDTH/2,  -LIGHTBAR_LENGTH/2}
};

Solver::Solver(const std::string & config_path)
: R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  auto yaml = YAML::LoadFile(config_path);

  auto R_g2i = yaml["R_gimbal2imubody"].as<std::vector<double>>();
  auto R_c2g = yaml["R_camera2gimbal"].as<std::vector<double>>();
  auto t_c2g = yaml["t_camera2gimbal"].as<std::vector<double>>();
  R_gimbal2imubody_ = Eigen::Matrix<double,3,3,Eigen::RowMajor>(R_g2i.data());
  R_camera2gimbal_  = Eigen::Matrix<double,3,3,Eigen::RowMajor>(R_c2g.data());
  t_camera2gimbal_  = Eigen::Matrix<double,3,1>(t_c2g.data());

  auto Kv = yaml["camera_matrix"].as<std::vector<double>>();
  auto Dv = yaml["distort_coeffs"].as<std::vector<double>>();
  Eigen::Matrix<double,3,3,Eigen::RowMajor> K(Kv.data());
  Eigen::Matrix<double,1,5> D(Dv.data());
  cv::eigen2cv(K, camera_matrix_);
  cv::eigen2cv(D, distort_coeffs_);
}

void Solver::solve(Armor & armor) const
{
  const auto & obj =
      (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  cv::solvePnP(obj, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
               cv::SOLVEPNP_IPPE);

  Eigen::Vector3d xyz_cam;
  cv::cv2eigen(tvec, xyz_cam);

  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_cam + t_camera2gimbal_;
  armor.xyz_in_world  = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat Rcv; cv::Rodrigues(rvec, Rcv);
  Eigen::Matrix3d R_ac;
  cv::cv2eigen(Rcv, R_ac);

  Eigen::Matrix3d R_ag = R_camera2gimbal_ * R_ac;
  Eigen::Matrix3d R_aw = R_gimbal2world_  * R_ag;

  armor.ypr_in_gimbal = tools::eulers(R_ag, 2, 1, 0);
  armor.ypr_in_world  = tools::eulers(R_aw, 2, 1, 0);
  armor.ypd_in_world  = tools::xyz2ypd(armor.xyz_in_world);
}

std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints) const
{
  Eigen::Matrix3d R_w2c = R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_w2c = -R_camera2gimbal_.transpose() * t_camera2gimbal_;

  cv::Mat rvec, tvec;
  cv::eigen2cv(R_w2c, rvec);
  cv::eigen2cv(t_w2c, tvec);

  std::vector<cv::Point3f> valid;
  valid.reserve(worldPoints.size());
  for (const auto & p : worldPoints) {
    Eigen::Vector3d Pw(p.x, p.y, p.z);
    Eigen::Vector3d Pc = R_w2c * Pw + t_w2c;
    if (Pc.z() > 0) valid.push_back(p);
  }
  if (valid.empty()) return {};

  std::vector<cv::Point2f> uv;
  cv::projectPoints(valid, rvec, tvec, camera_matrix_, distort_coeffs_, uv);
  return uv;
}

} // namespace auto_aim
