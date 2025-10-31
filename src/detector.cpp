#include "detector.hpp"

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <list>
#include <numeric>
#include <limits>

namespace auto_aim
{

Detector::Detector(const std::string & config_path, bool debug)
: debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);

  threshold_            = yaml["threshold"].as<double>();
  max_angle_error_      = yaml["max_angle_error"].as<double>() / 57.3;  // deg -> rad
  min_lightbar_ratio_   = yaml["min_lightbar_ratio"].as<double>();
  max_lightbar_ratio_   = yaml["max_lightbar_ratio"].as<double>();
  min_lightbar_length_  = yaml["min_lightbar_length"].as<double>();
  min_armor_ratio_      = yaml["min_armor_ratio"].as<double>();
  max_armor_ratio_      = yaml["max_armor_ratio"].as<double>();
  max_side_ratio_       = yaml["max_side_ratio"].as<double>();
  max_rectangular_error_= yaml["max_rectangular_error"].as<double>() / 57.3; // deg -> rad

  // 不再需要：classify_model / min_confidence / 保存样本等
}

std::list<Armor> Detector::detect(const cv::Mat & bgr_img, int /*frame_count*/)
{
  // 1) 灰度 + 二值
  cv::Mat gray_img, binary_img;
  cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);
  cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);
  // 注：去掉了 imshow("binary_img")，避免额外开销

  // 2) 轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  // 3) 轮廓 -> 灯条（几何 + 颜色）
  std::size_t lightbar_id = 0;
  std::list<Lightbar> lightbars;
  for (const auto & contour : contours) {
    const auto rotated_rect = cv::minAreaRect(contour);
    Lightbar lb(rotated_rect, lightbar_id);

    if (!check_geometry(lb)) continue;

    lb.color = get_color(bgr_img, contour);
    // 如需角点细化：lightbar_points_corrector(lb, gray_img);
    lightbars.emplace_back(lb);
    ++lightbar_id;
  }

  // 从左到右排序
  lightbars.sort([](const Lightbar & a, const Lightbar & b){ return a.center.x < b.center.x; });

  // 4) 配对 -> 装甲
  std::list<Armor> armors;
  for (auto left = lightbars.begin(); left != lightbars.end(); ++left) {
    for (auto right = std::next(left); right != lightbars.end(); ++right) {
      if (left->color != right->color) continue;

      Armor armor(*left, *right);
      if (!check_geometry(armor)) continue;

      armor.type        = get_type(armor);                   // 基于 ratio 的大小装甲判断
      armor.center_norm = get_center_norm(bgr_img, armor.center);
      armors.emplace_back(std::move(armor));
    }
  }

  // 5) 去重：共用灯条/相邻重叠
  for (auto a1 = armors.begin(); a1 != armors.end(); ++a1) {
    for (auto a2 = std::next(a1); a2 != armors.end(); ++a2) {
      const bool share_lightbar =
        (a1->left.id == a2->left.id)  || (a1->left.id == a2->right.id) ||
        (a1->right.id == a2->left.id) || (a1->right.id == a2->right.id);

      if (!share_lightbar) continue;

      // 简单面积比较：保留 ROI 较小者（用四点的外接矩形面积近似）
      auto rect_area = [](const Armor& a)->double{
        cv::Rect r = cv::boundingRect(std::vector<cv::Point2f>(a.points.begin(), a.points.end()));
        return static_cast<double>(r.area());
      };

      // 相邻共享：面积小者优先保留（你也可以改为面积大者，看数据分布）
      const double area1 = rect_area(*a1);
      const double area2 = rect_area(*a2);

      if (area1 <= area2) a2->duplicated = true; else a1->duplicated = true;
    }
  }
  armors.remove_if([](const Armor& a){ return a.duplicated; });

  // 6) 可选调试可视化：为了去掉 tools/fmt，这里不实现；在 main.cpp 里画四边形即可

  return armors;
}

// ROI 内细化一对灯条（如果你在外部用到了该接口，可以保留；否则可整段删除）
bool Detector::detect(Armor & armor, const cv::Mat & bgr_img)
{
  // 取得原四角点
  auto tl = armor.points[0];
  auto tr = armor.points[1];
  auto br = armor.points[2];
  auto bl = armor.points[3];

  // 计算一个收缩/扩展后的 ROI，尽量对齐装甲方向
  auto lt2b = bl - tl;
  auto rt2b = br - tr;
  auto tl1 = (tl + bl) / 2 - lt2b;
  auto bl1 = (tl + bl) / 2 + lt2b;
  auto br1 = (tr + br) / 2 + rt2b;
  auto tr1 = (tr + br) / 2 - rt2b;
  auto tl2tr = tr1 - tl1;
  auto bl2br = br1 - bl1;
  auto tl2 = (tl1 + tr) / 2 - 0.75f * tl2tr;
  auto tr2 = (tl1 + tr) / 2 + 0.75f * tl2tr;
  auto bl2 = (bl1 + br) / 2 - 0.75f * bl2br;
  auto br2 = (bl1 + br) / 2 + 0.75f * bl2br;

  std::vector<cv::Point> pts = {tl2, tr2, br2, bl2};
  cv::Rect bbox = cv::minAreaRect(pts).boundingRect();

  if (bbox.x < 0 || bbox.y < 0 || bbox.x + bbox.width > bgr_img.cols || bbox.y + bbox.height > bgr_img.rows)
    return false;

  cv::Mat roi = bgr_img(bbox);
  if (roi.empty()) return false;

  // 在 ROI 内重新做一次简化版检测（仅用于矫正四点）
  cv::Mat gray, bin;
  cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, bin, threshold_, 255, cv::THRESH_BINARY);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  std::list<Lightbar> lbs;
  std::size_t id = 0;
  for (const auto & c : contours) {
    Lightbar lb(cv::minAreaRect(c), id);
    if (!check_geometry(lb)) continue;
    lb.color = get_color(roi, c);
    // lightbar_points_corrector(lb, gray); // 如需精修可打开
    lbs.emplace_back(lb);
    ++id;
  }
  if (lbs.size() < 2) return false;

  lbs.sort([](const Lightbar & a, const Lightbar & b){ return a.center.x < b.center.x; });

  // 最近匹配（简单启发式）
  Lightbar *L = nullptr, *R = nullptr;
  float best_L = std::numeric_limits<float>::max();
  float best_R = std::numeric_limits<float>::max();

  for (auto & lb : lbs) {
    const cv::Point2f bias(static_cast<float>(bbox.x), static_cast<float>(bbox.y));
    float dL = cv::norm(tl - (lb.top + bias)) + cv::norm(bl - (lb.bottom + bias));
    float dR = cv::norm(br - (lb.bottom + bias)) + cv::norm(tr - (lb.top + bias));
    if (dL < best_L) { best_L = dL; L = &lb; }
    if (dR < best_R) { best_R = dR; R = &lb; }
  }

  if (L && R && (best_L + best_R < 15.0f)) {
    const cv::Point2f bias(static_cast<float>(bbox.x), static_cast<float>(bbox.y));
    armor.points[0] = L->top    + bias; // 左上
    armor.points[1] = R->top    + bias; // 右上
    armor.points[2] = R->bottom + bias; // 右下
    armor.points[3] = L->bottom + bias; // 左下
    return true;
  }
  return false;
}

bool Detector::check_geometry(const Lightbar & lb) const
{
  const bool angle_ok  = lb.angle_error < max_angle_error_;
  const bool ratio_ok  = lb.ratio > min_lightbar_ratio_ && lb.ratio < max_lightbar_ratio_;
  const bool length_ok = lb.length > min_lightbar_length_;
  return angle_ok && ratio_ok && length_ok;
}

bool Detector::check_geometry(const Armor & a) const
{
  const bool ratio_ok  = a.ratio > min_armor_ratio_ && a.ratio < max_armor_ratio_;
  const bool side_ok   = a.side_ratio < max_side_ratio_;
  const bool rect_ok   = a.rectangular_error < max_rectangular_error_;
  return ratio_ok && side_ok && rect_ok;
}

Color Detector::get_color(const cv::Mat & bgr_img, const std::vector<cv::Point> & contour) const
{
  int red_sum = 0, blue_sum = 0;
  for (const auto & p : contour) {
    const auto & px = bgr_img.at<cv::Vec3b>(p);
    blue_sum += px[0];
    red_sum  += px[2];
  }
  return (blue_sum > red_sum) ? Color::blue : Color::red;
}

// 根据几何比例粗判大小装甲（不依赖 ArmorName）
ArmorType Detector::get_type(const Armor & a)
{
  if (a.ratio > 3.0)   return ArmorType::big;
  if (a.ratio < 2.5)   return ArmorType::small;
  // 模糊区间就按更常见的小装甲处理（也可以根据宽度阈值再精细化）
  return ArmorType::small;
}

cv::Point2f Detector::get_center_norm(const cv::Mat & img, const cv::Point2f & c) const
{
  return { c.x / static_cast<float>(img.cols), c.y / static_cast<float>(img.rows) };
}

// 角点细化（可选使用），保留，无第三方依赖
void Detector::lightbar_points_corrector(Lightbar & lightbar, const cv::Mat & gray_img) const
{
  constexpr float ROI_SCALE = 0.07f;

  cv::Rect roi_box = lightbar.rotated_rect.boundingRect();
  roi_box.x      -= static_cast<int>(roi_box.width  * ROI_SCALE);
  roi_box.y      -= static_cast<int>(roi_box.height * ROI_SCALE);
  roi_box.width  += static_cast<int>(2 * roi_box.width  * ROI_SCALE);
  roi_box.height += static_cast<int>(2 * roi_box.height * ROI_SCALE);

  roi_box &= cv::Rect(0, 0, gray_img.cols, gray_img.rows);
  if (roi_box.empty()) return;

  cv::Mat roi = gray_img(roi_box).clone();
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, 25, cv::NORM_MINMAX);

  std::vector<cv::Point2f> pts; pts.reserve(roi.rows * roi.cols / 4);
  for (int y = 0; y < roi.rows; ++y) {
    const float* row = roi.ptr<float>(y);
    for (int x = 0; x < roi.cols; ++x) if (row[x] > 1e-3f) pts.emplace_back((float)x, (float)y);
  }
  if (pts.size() < 10) return;

  cv::PCA pca(cv::Mat(pts).reshape(1), cv::Mat(), cv::PCA::DATA_AS_ROW);
  cv::Point2f axis(pca.eigenvectors.at<float>(0,0), pca.eigenvectors.at<float>(0,1));
  axis /= cv::norm(axis); if (axis.y > 0) axis = -axis;

  auto search_corner = [&](int dir)->cv::Point2f {
    const float dx = axis.x * dir, dy = axis.y * dir;
    const float L  = static_cast<float>(lightbar.length) * 0.2f;
    cv::Point2f c  = lightbar.center;

    cv::Point2f best = c; float best_diff = -1.f;
    for (float t = 0.f; t < L; t += 1.f) {
      cv::Point2f p = c + cv::Point2f(dx * t, dy * t);
      if (p.x < 1 || p.y < 1 || p.x >= gray_img.cols-1 || p.y >= gray_img.rows-1) break;
      float v1 = gray_img.at<uchar>(cv::Point2f(p.x - dx, p.y - dy));
      float v2 = gray_img.at<uchar>(p);
      float diff = v1 - v2;
      if (diff > best_diff) { best_diff = diff; best = p; }
    }
    return best;
  };

  lightbar.top    = search_corner(+1);
  lightbar.bottom = search_corner(-1);
}

} // namespace auto_aim
