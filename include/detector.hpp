#ifndef AUTO_AIM__DETECTOR_HPP
#define AUTO_AIM__DETECTOR_HPP

#include <string>
#include <list>
#include <vector>
#include <opencv2/opencv.hpp>

#include "armor.hpp"

namespace auto_aim {

class Detector {
public:
  explicit Detector(const std::string& config_path, bool debug = false);

  // 主检测：整帧 → 装甲列表
  std::list<Armor> detect(const cv::Mat& bgr_img, int frame_count);

  // ROI 内再检测：用于细化已知装甲四点（可选）
  bool detect(Armor& armor, const cv::Mat& bgr_img);

private:
  // 参数（来自 configs/auto_aim.yaml）
  double threshold_{0.0};
  double max_angle_error_{0.0};       // rad
  double min_lightbar_ratio_{0.0};
  double max_lightbar_ratio_{0.0};
  double min_lightbar_length_{0.0};
  double min_armor_ratio_{0.0};
  double max_armor_ratio_{0.0};
  double max_side_ratio_{0.0};
  double max_rectangular_error_{0.0}; // rad

  bool debug_{false};

  // 内部工具函数（与 detector.cpp 一一对应）
  bool check_geometry(const Lightbar& lb) const;
  bool check_geometry(const Armor& a) const;

  Color get_color(const cv::Mat& bgr_img,
                  const std::vector<cv::Point>& contour) const;

  ArmorType get_type(const Armor& a);

  cv::Point2f get_center_norm(const cv::Mat& img,
                              const cv::Point2f& c) const;

  void lightbar_points_corrector(Lightbar& lightbar,
                                 const cv::Mat& gray_img) const;
};

} // namespace auto_aim

#endif // AUTO_AIM__DETECTOR_HPP
