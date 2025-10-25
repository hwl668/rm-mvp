#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "armor/types.hpp"

namespace armor {

// 构建“灰度二值”通路（固定阈值 or Otsu），随后做形态学
// 参数：thresh(0-255), use_otsu(0/1), morph_k(≥0)
void BuildGrayBinary(const cv::Mat& bgr, int thresh, bool use_otsu, int morph_k, cv::Mat& bin);

// 从任意二值图中提取“细长旋转矩形”候选（灯条雏形）
std::vector<LBBox> ExtractLightBarCandidates(const cv::Mat& bin, const YAML::Node& P);


// 新增：基于 rrect 计算每个候选的长边端点/中心/高度/角度
void EnrichLightBarsWithEndpoints(std::vector<LBBox>& bars);
// 调试绘制候选框

void DrawLBCandidates(cv::Mat& img, const std::vector<LBBox>& cands,
                      const cv::Scalar& color={0,255,255}, int thickness=2);

} // namespace armor
