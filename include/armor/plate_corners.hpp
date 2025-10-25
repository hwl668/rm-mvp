#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <array>
#include "armor/types.hpp"

namespace armor {

/**
 * @brief 由一对灯条外推装甲板四个角点（像素坐标）
 *
 * 输入：
 *  - left, right : 配对后的左右灯条（若顺序颠倒函数内部会校正）
 *  - P           : YAML 参数，需包含 plate.top_offset_ratio / plate.bottom_offset_ratio（可为 0）
 *
 * 输出：
 *  - corners_img : 顺序固定为 LT, RT, RB, LB（顺时针）
 *
 * 返回：
 *  - true  成功
 *  - false 退化（高度/宽度过小等）
 */
bool EstimatePlateCorners(const LBBox& left,
                          const LBBox& right,
                          const YAML::Node& P,
                          std::array<cv::Point2f,4>& corners_img);

} // namespace armor
