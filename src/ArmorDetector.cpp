#include "ArmorDetector.h"
#include <algorithm>

using namespace cv;
using namespace std;

namespace rm
{

enum
{
    WIDTH_GREATER_THAN_HEIGHT,
    ANGLE_TO_UP
};

cv::RotatedRect& ArmorDetector::adjustRec(cv::RotatedRect& rec, const int mode)
{
    using std::swap;

    float& width = rec.size.width;
    float& height = rec.size.height;
    float& angle = rec.angle;

    if(mode == WIDTH_GREATER_THAN_HEIGHT)
    {
        if(width < height)
        {
            swap(width, height);
            angle += 90.0;
        }
    }

    while(angle >= 90.0) angle -= 180.0;
    while(angle < -90.0) angle += 180.0;

    if(mode == ANGLE_TO_UP)
    {
        if(angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if(angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }
    }

    return rec;
}

float ArmorDetector::distance(const cv::Point2f& p1, const cv::Point2f& p2) const
{
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return sqrt(dx * dx + dy * dy);
}

ArmorDescriptor::ArmorDescriptor()
{
    vertex.resize(4);
    for(int i = 0; i < 4; i++)
    {
        vertex[i] = cv::Point2f(0, 0);
    }
    type = UNKNOWN_ARMOR;
}

ArmorDescriptor::ArmorDescriptor(const LightDescriptor& lLight, const LightDescriptor& rLight, const int armorType)
{
    cv::RotatedRect leftLight = lLight.rec();
    cv::RotatedRect rightLight = rLight.rec();

    // 扩展灯条高度
    cv::Size exLSize(int(leftLight.size.width), int(leftLight.size.height * 2));
    cv::Size exRSize(int(rightLight.size.width), int(rightLight.size.height * 2));
    cv::RotatedRect exLLight(leftLight.center, exLSize, leftLight.angle);
    cv::RotatedRect exRLight(rightLight.center, exRSize, rightLight.angle);

    cv::Point2f pts_l[4];
    exLLight.points(pts_l);
    cv::Point2f upper_l = pts_l[2];
    cv::Point2f lower_l = pts_l[3];

    cv::Point2f pts_r[4];
    exRLight.points(pts_r);
    cv::Point2f upper_r = pts_r[1];
    cv::Point2f lower_r = pts_r[0];

    vertex.resize(4);
    vertex[0] = upper_l;  // 左上
    vertex[1] = upper_r;  // 右上
    vertex[2] = lower_r;  // 右下
    vertex[3] = lower_l;  // 左下

    type = armorType;
}

ArmorDetector::ArmorDetector()
{
    _flag = ARMOR_NO;
    _enemy_color = BLUE;
}

ArmorDetector::ArmorDetector(const ArmorParam& armorParam)
{
    _param = armorParam;
    _flag = ARMOR_NO;
    _enemy_color = armorParam.enemy_color;
}

void ArmorDetector::init(const ArmorParam& armorParam)
{
    _param = armorParam;
    _enemy_color = armorParam.enemy_color;
}

void ArmorDetector::setEnemyColor(int enemy_color)
{
    _enemy_color = enemy_color;
}

void ArmorDetector::loadImg(const cv::Mat& srcImg)
{
    _srcImg = srcImg.clone();
    cvtColor(_srcImg, _grayImg, COLOR_BGR2GRAY);
}

int ArmorDetector::detect()
{
    _armors.clear();
    std::vector<LightDescriptor> lightInfos;

    // 预处理：二值化
    cv::Mat binBrightImg;
    cv::threshold(_grayImg, binBrightImg, _param.brightness_threshold, 255, cv::THRESH_BINARY);
    
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    dilate(binBrightImg, binBrightImg, element);

    // 查找并筛选灯条 - 修改这里的常量
    vector<vector<Point>> lightContours;
    cv::findContours(binBrightImg.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    for(const auto& contour : lightContours)
    {
        float lightContourArea = contourArea(contour);
        if(contour.size() <= 5 || lightContourArea < _param.light_min_area) 
            continue;

        RotatedRect lightRec = fitEllipse(contour);
        adjustRec(lightRec, ANGLE_TO_UP);

        if(lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
           lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity) 
            continue;

        // 颜色检测
        lightRec.size.width *= _param.light_color_detect_extend_ratio;
        lightRec.size.height *= _param.light_color_detect_extend_ratio;
        Rect lightRect = lightRec.boundingRect();
        const Rect srcBound(Point(0, 0), _srcImg.size());
        lightRect &= srcBound;
        
        if(lightRect.area() <= 0) continue;
        
        Mat lightImg = _srcImg(lightRect);
        Mat lightMask = Mat::zeros(lightRect.size(), CV_8UC1);
        
        Point2f lightVertexArray[4];
        lightRec.points(lightVertexArray);
        std::vector<Point> lightVertex;
        for(int i = 0; i < 4; i++)
        {
            lightVertex.emplace_back(Point(lightVertexArray[i].x - lightRect.tl().x,
                                           lightVertexArray[i].y - lightRect.tl().y));
        }
        fillConvexPoly(lightMask, lightVertex, 255);
        cv::dilate(lightMask, lightMask, element);
        
        const Scalar meanVal = mean(lightImg, lightMask);

        // 检查敌方颜色
        if(((_enemy_color == BLUE) && (meanVal[BLUE] - meanVal[RED] > 20.0)) || 
           (_enemy_color == RED && meanVal[RED] - meanVal[BLUE] > 20.0))
        {
            lightInfos.push_back(LightDescriptor(lightRec));
        }
    }

    if(lightInfos.empty())
    {
        return _flag = ARMOR_NO;
    }

    // 灯条配对
    sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2)
    {
        return ld1.center.x < ld2.center.x;
    });

    for(size_t i = 0; i < lightInfos.size(); i++)
    {
        for(size_t j = i + 1; j < lightInfos.size(); j++)
        {
            const LightDescriptor& leftLight  = lightInfos[i];
            const LightDescriptor& rightLight = lightInfos[j];

            // 检查形态相似性
            float angleDiff_ = abs(leftLight.angle - rightLight.angle);
            float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
            if(angleDiff_ > _param.light_max_angle_diff_ ||
               LenDiff_ratio > _param.light_max_height_diff_ratio_)
            {
                continue;
            }

            // 检查位置合理性
            float dis = distance(leftLight.center, rightLight.center);
            float meanLen = (leftLight.length + rightLight.length) / 2;
            float yDiff = abs(leftLight.center.y - rightLight.center.y);
            float yDiff_ratio = yDiff / meanLen;
            float xDiff = abs(leftLight.center.x - rightLight.center.x);
            float xDiff_ratio = xDiff / meanLen;
            float ratio = dis / meanLen;
            
            if(yDiff_ratio > _param.light_max_y_diff_ratio_ ||
               xDiff_ratio < _param.light_min_x_diff_ratio_ ||
               ratio > _param.armor_max_aspect_ratio_ ||
               ratio < _param.armor_min_aspect_ratio_)
            {
                continue;
            }

            // 判断装甲板类型
            int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
            
            ArmorDescriptor armor(leftLight, rightLight, armorType);
            _armors.emplace_back(armor);
            break;
        }
    }

    if(_armors.empty())
    {
        return _flag = ARMOR_NO;
    }

    // 选择最佳装甲板（这里简单选择第一个）
    _targetArmor = _armors[0];

    return _flag = ARMOR_LOCAL;
}

const std::vector<cv::Point2f> ArmorDetector::getArmorVertex() const
{
    return _targetArmor.vertex;
}

int ArmorDetector::getArmorType() const
{
    return _targetArmor.type;
}

cv::Mat ArmorDetector::getDebugImg() const
{
    Mat debugImg = _srcImg.clone();
    
    // 绘制装甲板角点
    for(size_t i = 0; i < _targetArmor.vertex.size(); i++)
    {
        line(debugImg, _targetArmor.vertex[i], _targetArmor.vertex[(i+1)%4], Scalar(0, 255, 0), 2);
        circle(debugImg, _targetArmor.vertex[i], 5, Scalar(0, 0, 255), -1);
    }
    
    return debugImg;
}

}