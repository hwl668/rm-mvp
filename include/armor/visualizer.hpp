#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include "armor/types.hpp"
#include "armor/pairing.hpp"

namespace armor {

void DrawLightBar(cv::Mat& img, const LBBox& b,
                  const cv::Scalar& box_color={0,255,255},
                  const cv::Scalar& ep_color={0,0,255});

void DrawBestPair(cv::Mat& img, const std::vector<LBBox>& bars, const PairLB& p,
                  const cv::Scalar& color={0,255,0});

void PutTextShadow(cv::Mat& img, const std::string& text,
                   cv::Point org, double scale=0.7,
                   const cv::Scalar& fg={255,255,255});

} // namespace armor
