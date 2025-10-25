#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include "armor/types.hpp"

namespace armor
{

struct ColorResult
{
    TeamColor color = TeamColor::UNKNOWN;
    cv::Mat mask;
    double score_red = 0.0;
    double score_blue = 0.0;
};

void BuildHSVRedBlue(const cv::Mat& bgr,
                     const YAML::Node& params,
                     cv::Mat& mask_red,
                     cv::Mat& mask_blue);

ColorResult SelectColor(const cv::Mat& bgr,
                        const YAML::Node& params);

} // namespace armor
