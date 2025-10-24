#include "PnPSolver.h"
#include <iostream>

using namespace cv;
using namespace std;

namespace rm
{

PnPSolver::PnPSolver()
{
}

PnPSolver::PnPSolver(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
    _cameraMatrix = cameraMatrix.clone();
    _distCoeffs = distCoeffs.clone();
}

void PnPSolver::setCameraParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs)
{
    _cameraMatrix = cameraMatrix.clone();
    _distCoeffs = distCoeffs.clone();
}

std::vector<cv::Point3f> PnPSolver::getObjectPoints(int armorType)
{
    std::vector<cv::Point3f> objectPoints;
    
    // 装甲板实际尺寸（单位：米）
    // 小装甲板: 135mm x 54mm
    // 大装甲板: 230mm x 54mm
    
    if(armorType == SMALL_ARMOR)
    {
        float halfWidth = 0.135f / 2.0f;  // 67.5mm
        float halfHeight = 0.054f / 2.0f;  // 27mm
        
        objectPoints = {
            cv::Point3f(-halfWidth, halfHeight, 0),   // 左上
            cv::Point3f(halfWidth, halfHeight, 0),    // 右上
            cv::Point3f(halfWidth, -halfHeight, 0),   // 右下
            cv::Point3f(-halfWidth, -halfHeight, 0)   // 左下
        };
    }
    else // BIG_ARMOR
    {
        float halfWidth = 0.230f / 2.0f;  // 115mm
        float halfHeight = 0.054f / 2.0f;  // 27mm
        
        objectPoints = {
            cv::Point3f(-halfWidth, halfHeight, 0),
            cv::Point3f(halfWidth, halfHeight, 0),
            cv::Point3f(halfWidth, -halfHeight, 0),
            cv::Point3f(-halfWidth, -halfHeight, 0)
        };
    }
    
    return objectPoints;
}

bool PnPSolver::solve(const std::vector<cv::Point2f>& imagePoints, int armorType)
{
    if(imagePoints.size() != 4)
    {
        cerr << "Error: Need 4 image points!" << endl;
        return false;
    }
    
    _imagePoints = imagePoints;
    
    std::vector<cv::Point3f> objectPoints = getObjectPoints(armorType);
    
    cv::Mat rvec, tvec;
    bool success = cv::solvePnP(objectPoints, imagePoints, _cameraMatrix, _distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
    
    if(success)
    {
        rvec.copyTo(_rvec);
        tvec.copyTo(_tvec);
    }
    
    return success;
}

double PnPSolver::getDistance() const
{
    return cv::norm(_tvec);
}

void PnPSolver::drawAxis(cv::Mat& image, float axisLength)
{
    // 定义坐标轴的3D点 - 从原点出发，沿着三个互相垂直的方向
    std::vector<cv::Point3f> axisPoints;
    axisPoints.push_back(cv::Point3f(0, 0, 0));           // 原点
    axisPoints.push_back(cv::Point3f(axisLength, 0, 0)); // X轴 - 沿X方向
    axisPoints.push_back(cv::Point3f(0, axisLength, 0)); // Y轴 - 沿Y方向
    axisPoints.push_back(cv::Point3f(0, 0, axisLength)); // Z轴 - 沿Z方向
    
    // 投影到图像平面
    std::vector<cv::Point2f> imageAxisPoints;
    cv::projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imageAxisPoints);
    
    // 绘制坐标轴 - 使用箭头表示方向，清晰显示三个向量互相垂直
    const double arrowTipLength = 0.3;  // 箭头尖端长度比例
    cv::arrowedLine(image, imageAxisPoints[0], imageAxisPoints[1], cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0, arrowTipLength); // X轴 - 红色
    cv::arrowedLine(image, imageAxisPoints[0], imageAxisPoints[2], cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0, arrowTipLength); // Y轴 - 绿色
    cv::arrowedLine(image, imageAxisPoints[0], imageAxisPoints[3], cv::Scalar(255, 0, 0), 3, cv::LINE_AA, 0, arrowTipLength); // Z轴 - 蓝色
    
    // 添加标签
    cv::putText(image, "X", imageAxisPoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    cv::putText(image, "Y", imageAxisPoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
    cv::putText(image, "Z", imageAxisPoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
}

void PnPSolver::drawResult(cv::Mat& image)
{
    // 显示位置信息
    char text[100];
    sprintf(text, "Distance: %.2f m", getDistance());
    cv::putText(image, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
    
    sprintf(text, "X: %.3f m", _tvec[0]);
    cv::putText(image, text, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    
    sprintf(text, "Y: %.3f m", _tvec[1]);
    cv::putText(image, text, cv::Point(10, 85), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
    
    sprintf(text, "Z: %.3f m", _tvec[2]);
    cv::putText(image, text, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
}

}