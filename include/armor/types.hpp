#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

namespace armor {

// 阵营（自动判色）
enum class TeamColor { RED, BLUE, UNKNOWN };

// 灯条候选（含端点）
struct LBBox {
  cv::RotatedRect rrect;     // minAreaRect 结果
  double area = 0.0;         // 轮廓面积
  double ratio = 0.0;        // 长宽比 max(w,h)/min(w,h) >= 1
  double upright_deg = 180;  // 与竖直夹角（度），越小越竖
  cv::Point2f ep[2];         // 长边两端点（像素坐标）约定：ep[0] 的 y <= ep[1].y
  float height = 0.f;        // 长边长度（像素）
  cv::Point2f center;        // 中心
  float tilt_deg = 0.f;      // 长边相对竖直的角度（度）= upright_deg
};

} // namespace armor

