#include "armor.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>

// 文件: armor.cpp
// 说明: Armor 与 Lightbar 相关的数据结构实现。
// 主要职责:
// - Lightbar: 从旋转矩形计算灯条的几何特征（中心、上下端点、宽度、长度、角度等）。
// - Armor: 提供多种构造函数, 支持从两个灯条（传统方法）、神经网络检测结果和 YOLOV5 检测结果
//   构造装甲板对象，并计算装甲的几何特征（中心、角度误差、长宽比、类型等）。
// 注: 该文件只修改注释，不改变原有算法逻辑或数值计算。

namespace auto_aim
{
// Lightbar 构造函数
// 输入: 一个 OpenCV 的 RotatedRect 表示单个灯条的最小外接旋转矩形。
// 计算并填充:
// - 四个角点并按 y 排序以确定上下端点
// - top / bottom: 灯条的近似上端和下端点
// - center: 矩形中心
// - top2bottom: 从上端指向下端的向量
// - width: 灯条短边长度（横向宽度）
// - length: 灯条长度（纵向）
// - angle: 灯条主方向角（弧度）
// - angle_error: 与垂直方向的偏差量（用于筛选接近垂直的灯条）
Lightbar::Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id)
: id(id), rotated_rect(rotated_rect)
{
  std::vector<cv::Point2f> corners(4);
  rotated_rect.points(&corners[0]);
  std::sort(corners.begin(), corners.end(), [](const cv::Point2f & a, const cv::Point2f & b) {
    return a.y < b.y;
  });

  center = rotated_rect.center;
  top = (corners[0] + corners[1]) / 2;
  bottom = (corners[2] + corners[3]) / 2;
  top2bottom = bottom - top;

  points.emplace_back(top);
  points.emplace_back(bottom);

  width = cv::norm(corners[0] - corners[1]);
  angle = std::atan2(top2bottom.y, top2bottom.x);
  angle_error = std::abs(angle - CV_PI / 2);
  length = cv::norm(top2bottom);
  ratio = length / width;
}

//传统构造函数
// 传统方法: 由左右两个灯条构造 Armor
// 输入: left / right 两个 Lightbar
// 计算并填充:
// - color: 使用左灯条颜色作为装甲颜色（通常两侧颜色一致）
// - center: 装甲中心（左右中心点的中点）
// - points: 装甲四个角点，按顺序存放便于后续绘制或几何计算
// - ratio: 装甲宽度与灯条长度的比值（用于判定是否符合装甲长宽比）
// - side_ratio: 两侧灯条长度比（用于判定灯条是否大小一致）
// - rectangular_error: 两侧灯条与装甲滚转方向的垂直误差的最大值
Armor::Armor(const Lightbar & left, const Lightbar & right)
: left(left), right(right), duplicated(false)
{
  color = left.color;
  center = (left.center + right.center) / 2;

  points.emplace_back(left.top);
  points.emplace_back(right.top);
  points.emplace_back(right.bottom);
  points.emplace_back(left.bottom);

  auto left2right = right.center - left.center;
  auto width = cv::norm(left2right);
  auto max_lightbar_length = std::max(left.length, right.length);
  auto min_lightbar_length = std::min(left.length, right.length);
  ratio = width / max_lightbar_length;
  side_ratio = max_lightbar_length / min_lightbar_length;

  auto roll = std::atan2(left2right.y, left2right.x);
  auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
  auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);
  rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
}


} // namespace auto_aim

