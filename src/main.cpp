
              #include <iostream>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.h"
#include "PnPSolver.h"

using namespace std;
using namespace cv;
using namespace rm;

int main(int argc, char** argv)
{
    cout << "=== Armor Detection Debug Version ===" << endl;
    
    string imagePath = (argc > 1) ? argv[1] : "test.jpg";
    Mat srcImg = imread(imagePath);
    
    if(srcImg.empty())
    {
        cerr << "Error: Cannot load image: " << imagePath << endl;
        return -1;
    }
    
    cout << "Image size: " << srcImg.cols << "x" << srcImg.rows << endl;
    
    imshow("1. Original", srcImg);
    
    Mat grayImg;
    cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
    imshow("2. Gray", grayImg);
    
    // 统计
    double minGray, maxGray;
    minMaxLoc(grayImg, &minGray, &maxGray);
    Scalar meanGray = mean(grayImg);
    cout << "\nGray: Min=" << minGray << ", Max=" << maxGray << ", Mean=" << meanGray[0] << endl;
    
    // ===== 策略：分离连接的亮区域 =====
    cout << "\n=== Trying to separate bright regions ===" << endl;
    
    vector<int> thresholds = {200, 220, 235};
    
    for(int thresh : thresholds)
    {
        cout << "\n--- Threshold: " << thresh << " ---" << endl;
        
        // 二值化
        Mat binImg;
        threshold(grayImg, binImg, thresh, 232, THRESH_BINARY);
        
        // 显示原始二值化
        imshow("3a. Binary (raw) T=" + to_string(thresh), binImg);
        
        // 使用形态学操作来分离连接的区域
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 11));
        
        // 先腐蚀来分离连接的区域
        Mat eroded;
        erode(binImg, eroded, element, Point(-1,-1), 2);
        imshow("3b. Eroded T=" + to_string(thresh), eroded);
        
        // 再膨胀回来
        Mat processed;
        dilate(eroded, processed, element, Point(-1,-1), 0);
        imshow("3c. Processed T=" + to_string(thresh), processed);
        
        float whiteRatio = (float)countNonZero(processed) / processed.total() * 100;
        cout << "Bright pixels: " << whiteRatio << "%" << endl;
        
        // 查找轮廓
        vector<vector<Point>> contours;
        findContours(processed.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        cout << "Found " << contours.size() << " regions" << endl;
        
        // 详细分析每个轮廓
        Mat analysisImg = srcImg.clone();
        int validCount = 0;
        
        for(size_t i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            
            cout << "\n  Region " << i << ":" << endl;
            cout << "    Area: " << area << endl;
            
            if(contours[i].size() < 5) {
                cout << "    → Too few points" << endl;
                continue;
            }
            
            RotatedRect rect = fitEllipse(contours[i]);
            float w = rect.size.width;
            float h = rect.size.height;
            float ratio = max(w, h) / (min(w, h) + 0.01f);
            
            cout << "    Size: " << w << " x " << h << endl;
            cout << "    Ratio: " << ratio << endl;
            cout << "    Angle: " << rect.angle << endl;
            
            // 非常宽松的条件
            bool isValid = false;
            if(area >= 5 && area <= 50000) {  // 面积范围很宽
                if(ratio >= 1.0 && ratio <= 50) {  // 长宽比很宽
                    isValid = true;
                    validCount++;
                }
            }
            
            cout << "    Valid: " << (isValid ? "YES" : "NO") << endl;
            
            // 绘制
            Scalar color = isValid ? Scalar(0, 255, 0) : Scalar(0, 0, 255);
            drawContours(analysisImg, contours, i, color, 2);
            
            // 绘制拟合的椭圆
            ellipse(analysisImg, rect, Scalar(255, 0, 255), 2);
            
            // 标注信息
            Point2f center = rect.center;
            putText(analysisImg, "A:" + to_string((int)area), 
                   Point((int)center.x, (int)center.y - 15),
                   FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            putText(analysisImg, "R:" + to_string((float)((int)(ratio*10))/10), 
                   Point((int)center.x, (int)center.y),
                   FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
            putText(analysisImg, to_string(i), 
                   Point((int)center.x, (int)center.y + 15),
                   FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 0), 2);
        }
        
        cout << "\nValid candidates: " << validCount << "/" << contours.size() << endl;
        imshow("4. Analysis T=" + to_string(thresh), analysisImg);
        
        if(validCount < 2) {
            cout << "⚠ Need at least 2 valid regions!" << endl;
            continue;
        }
        
        // 尝试装甲板检测
        ArmorParam param;
        param.enemy_color = BLUE;  // 如果是红色装甲板，改成 RED
        param.brightness_threshold = thresh;
        
        // 极度宽松的参数
        param.light_min_area = 3000.0f;
        param.light_max_ratio = 9.0f;  // 允许更宽的灯条
        param.light_contour_min_solidity = 0.2f;
        param.light_max_angle_diff_ = 12.0f;
        param.light_max_height_diff_ratio_ = 0.8f;
        param.light_max_y_diff_ratio_ = 5.0f;
        param.light_min_x_diff_ratio_ = 0.2f;
        param.armor_min_aspect_ratio_ = 0.5f;
        param.armor_max_aspect_ratio_ = 8.0f;
        
        ArmorDetector detector(param);
        detector.loadImg(srcImg);
        int result = detector.detect();
        
        if(result == ArmorDetector::ARMOR_LOCAL)
        {
            cout << "\n✓✓✓ ARMOR DETECTED! ✓✓✓" << endl;
            
            vector<Point2f> armorVertex = detector.getArmorVertex();
            int armorType = detector.getArmorType();
            
            cout << "Type: " << (armorType == SMALL_ARMOR ? "Small" : "Big") << endl;
            for(size_t i = 0; i < armorVertex.size(); i++)
            {
                cout << "P" << i << ": (" << (int)armorVertex[i].x 
                     << ", " << (int)armorVertex[i].y << ")" << endl;
            }
            
            Mat detectImg = detector.getDebugImg();
            imshow("5. Detection", detectImg);
            imwrite("detection_result.jpg", detectImg);
            
            // PnP
            cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
                9.28130989e+02, 0, 3.77572945e+02,
                0, 9.30138391e+02, 2.83892859e+02,
                0, 0, 1.0);
            
            cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
                -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, 
                -1.09433818e-03, -1.33846840e+00);
            
            PnPSolver pnpSolver(cameraMatrix, distCoeffs);
            
            if(pnpSolver.solve(armorVertex, armorType))
            {
                cout << "\n✓✓✓ PnP SOLVED! ✓✓✓" << endl;
                
                Vec3d pos = pnpSolver.getPosition();
                double dist = pnpSolver.getDistance();
                
                cout << "Position: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ") m" << endl;
                cout << "Distance: " << dist << " m" << endl;
                
                Mat resultImg = detectImg.clone();
                pnpSolver.drawAxis(resultImg, 0.1f);
                pnpSolver.drawResult(resultImg);
                
                namedWindow("6. FINAL RESULT", WINDOW_NORMAL);
                imshow("6. FINAL RESULT", resultImg);
                imwrite("final_result.jpg", resultImg);
                
                cout << "\n✓ Saved: detection_result.jpg, final_result.jpg" << endl;
                cout << "\nPress any key..." << endl;
                waitKey(0);
                return 0;
            }
        }
    }
    
    cout << "\n✗ Detection failed!" << endl;
    cout << "\nDEBUG INFO:" << endl;
    cout << "Check windows '3a/3b/3c' - do you see separated white regions?" << endl;
    cout << "Check window '4. Analysis' - are regions marked in GREEN?" << endl;
    cout << "\nIf you see only 1 big white blob, the image might have:" << endl;
    cout << "  - Bright background" << endl;
    cout << "  - Connected bright regions" << endl;
    cout << "  - Overexposure" << endl;
    
    cout << "\nPress any key..." << endl;
    waitKey(0);
    
    return -1;
}