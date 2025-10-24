#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include "General.h"

namespace rm
{

class PnPSolver
{
public:
    PnPSolver();
    PnPSolver(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

    void setCameraParams(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    
    bool solve(const std::vector<cv::Point2f>& imagePoints, int armorType);
    
    cv::Vec3d getPosition() const { return _tvec; }
    cv::Vec3d getRotation() const { return _rvec; }
    double getDistance() const;
    
    void drawAxis(cv::Mat& image, float axisLength = 0.1f);
    void drawResult(cv::Mat& image);

private:
    cv::Mat _cameraMatrix;
    cv::Mat _distCoeffs;
    cv::Vec3d _rvec;
    cv::Vec3d _tvec;
    std::vector<cv::Point2f> _imagePoints;
    
    std::vector<cv::Point3f> getObjectPoints(int armorType);
};

}
