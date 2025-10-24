#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "General.h"

namespace rm
{

struct ArmorParam
{
    // Pre-treatment
    int brightness_threshold = 225;
    float light_color_detect_extend_ratio = 1.1f;

    // Filter lights
    float light_min_area = 400.0f;
    float light_max_ratio = 1.0f;
    float light_contour_min_solidity = 0.5f;

    // Filter pairs
    float light_max_angle_diff_ = 12.0f;
    float light_max_height_diff_ratio_ = 0.30f;
    float light_max_y_diff_ratio_ = 0.30f;
    float light_min_x_diff_ratio_ = 0.5f;

    // Filter armor
    float armor_big_armor_ratio = 3.2f;
    float armor_small_armor_ratio = 2.0f;
    float armor_min_aspect_ratio_ = 1.8f;
    float armor_max_aspect_ratio_ = 3.4f;

    int enemy_color = BLUE;
};

class LightDescriptor
{
public:
    float width;
    float length;
    cv::Point2f center;
    float angle;
    float area;

    LightDescriptor() {}
    LightDescriptor(const cv::RotatedRect& light)
    {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();
    }

    cv::RotatedRect rec() const
    {
        return cv::RotatedRect(center, cv::Size2f(width, length), angle);
    }
};

class ArmorDescriptor
{
public:
    std::vector<cv::Point2f> vertex; // 4个角点
    int type; // 装甲板类型

    ArmorDescriptor();
    ArmorDescriptor(const LightDescriptor& lLight, const LightDescriptor& rLight, const int armorType);
};

class ArmorDetector
{
public:
    enum ArmorFlag
    {
        ARMOR_NO = 0,
        ARMOR_LOCAL = 3
    };

    ArmorDetector();
    ArmorDetector(const ArmorParam& armorParam);

    void init(const ArmorParam& armorParam);
    void setEnemyColor(int enemy_color);
    void loadImg(const cv::Mat& srcImg);
    int detect();

    const std::vector<cv::Point2f> getArmorVertex() const;
    int getArmorType() const;
    cv::Mat getDebugImg() const;

private:
    ArmorParam _param;
    int _enemy_color;

    cv::Mat _srcImg;
    cv::Mat _grayImg;

    std::vector<ArmorDescriptor> _armors;
    ArmorDescriptor _targetArmor;

    int _flag;

    float distance(const cv::Point2f& p1, const cv::Point2f& p2) const;
    cv::RotatedRect& adjustRec(cv::RotatedRect& rec, const int mode);
};

}