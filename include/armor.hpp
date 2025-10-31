#ifndef AUTO_AIM__ARMOR_HPP
#define AUTO_AIM__ARMOR_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace auto_aim {

enum Color { red, blue, extinguish, purple };
static const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType { big, small };
static const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

// 必须保留：solver 里会用到这些名称（three/four/five/outpost/base/one）
enum ArmorName {
  one, two, three, four, five, sentry, outpost, base, not_armor
};

struct Lightbar {
  std::size_t id{0};
  Color color{blue};
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle{0}, angle_error{0}, length{0}, width{0}, ratio{0};
  cv::RotatedRect rotated_rect;

  Lightbar() = default;
  Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id);
};

struct Armor {
  Color color{blue};
  Lightbar left, right;
  cv::Point2f center;          // 非对角线交点
  cv::Point2f center_norm;     // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio{0};             // 两灯条中点连线 / 长灯条
  double side_ratio{0};        // 长/短灯条
  double rectangular_error{0}; // 与90°偏差

  ArmorType type{small};
  ArmorName name{not_armor};   // 需要存在，solver 会访问

  cv::Rect box;
  double confidence{0};
  bool duplicated{false};

  // PnP 结果
  Eigen::Vector3d xyz_in_gimbal{0,0,0};
  Eigen::Vector3d xyz_in_world{0,0,0};
  Eigen::Vector3d ypr_in_gimbal{0,0,0};
  Eigen::Vector3d ypr_in_world{0,0,0};
  Eigen::Vector3d ypd_in_world{0,0,0};
  double yaw_raw{0};

  // 仅保留几何配对构造
  Armor(const Lightbar& left, const Lightbar& right);
};

} // namespace auto_aim

#endif // AUTO_AIM__ARMOR_HPP
