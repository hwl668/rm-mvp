#include "armor/color_selector.hpp"

#include <algorithm>
#include <vector>

namespace armor
{
namespace
{

cv::Mat makeKernel(int k)
{
  int kk = std::max(1, k | 1);
  return cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kk, kk));
}

void morphOpenClose(cv::Mat& mask, int ksize)
{
  if(ksize <= 0)
    return;

  cv::Mat kernel = makeKernel(ksize);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
}

void hsvRange6(const std::vector<int>& values, cv::Scalar& lo, cv::Scalar& hi)
{
  CV_Assert(values.size() == 6);
  lo = cv::Scalar(values[0], values[1], values[2]);
  hi = cv::Scalar(values[3], values[4], values[5]);
}

double bigAreaScore(const cv::Mat& mask, int min_area)
{
  double score = static_cast<double>(cv::countNonZero(mask));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  for(const auto& contour : contours)
  {
    double area = std::fabs(cv::contourArea(contour));
    if(area >= min_area)
    {
      score += 2.0 * area;
    }
  }
  return score;
}

} // namespace

void BuildHSVRedBlue(const cv::Mat& bgr,
           const YAML::Node& params,
           cv::Mat& mask_red,
           cv::Mat& mask_blue)
{
  CV_Assert(!bgr.empty() && bgr.type() == CV_8UC3);

  const YAML::Node& color = params["color"];

  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

  const auto red1 = color["hsv_red1"].as<std::vector<int>>();
  const auto red2 = color["hsv_red2"].as<std::vector<int>>();
  const auto blue = color["hsv_blue"].as<std::vector<int>>();

  cv::Scalar r1_lo, r1_hi, r2_lo, r2_hi, b_lo, b_hi;
  hsvRange6(red1, r1_lo, r1_hi);
  hsvRange6(red2, r2_lo, r2_hi);
  hsvRange6(blue, b_lo, b_hi);

  cv::Mat red_mask1, red_mask2;
  cv::inRange(hsv, r1_lo, r1_hi, red_mask1);
  cv::inRange(hsv, r2_lo, r2_hi, red_mask2);
  cv::bitwise_or(red_mask1, red_mask2, mask_red);

  cv::inRange(hsv, b_lo, b_hi, mask_blue);
}

ColorResult SelectColor(const cv::Mat& bgr,
            const YAML::Node& params)
{
  cv::Mat mask_red;
  cv::Mat mask_blue;
  BuildHSVRedBlue(bgr, params, mask_red, mask_blue);

  int morph_kernel = params["color"]["morph_kernel"].as<int>(3);
  morphOpenClose(mask_red, morph_kernel);
  morphOpenClose(mask_blue, morph_kernel);

  int min_area = params["lightbar"]["min_area"].as<int>(20);
  double score_red = bigAreaScore(mask_red, min_area);
  double score_blue = bigAreaScore(mask_blue, min_area);

  ColorResult result;
  result.score_red = score_red;
  result.score_blue = score_blue;

  if(score_red <= 0.0 && score_blue <= 0.0)
  {
    result.color = TeamColor::UNKNOWN;
    result.mask = cv::Mat::zeros(bgr.size(), CV_8U);
    return result;
  }

  if(score_red >= score_blue)
  {
    result.color = TeamColor::RED;
    result.mask = mask_red;
  }
  else
  {
    result.color = TeamColor::BLUE;
    result.mask = mask_blue;
  }

  return result;
}

} // namespace armor


