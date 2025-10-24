#include <iostream>
#include <opencv2/opencv.hpp>
#include "ArmorDetector.h"
#include "PnPSolver.h"

using namespace std;
using namespace cv;
using namespace rm;

int main(int argc, char** argv)
{
    cout << "=== Armor Detection with PnP ===" << endl;
    
    string imagePath = (argc > 1) ? argv[1] : "test.jpg";
    Mat srcImg = imread(imagePath);
    
    if(srcImg.empty())
    {
        cerr << "Error: Cannot load image: " << imagePath << endl;
        return -1;
    }
    
    cout << "Image size: " << srcImg.cols << "x" << srcImg.rows << endl;
    
    // 裁剪图像边缘以移除亮边框（如果存在）
    // 使用固定的50像素边框，这对于大多数测试图像已经足够
    // 可以通过命令行参数或配置文件自定义此值
    int borderSize = 50;
    
    Mat processImg;
    if(srcImg.cols > 2 * borderSize && srcImg.rows > 2 * borderSize)
    {
        Rect roi(borderSize, borderSize, 
                 srcImg.cols - 2 * borderSize, 
                 srcImg.rows - 2 * borderSize);
        processImg = srcImg(roi).clone();
        cout << "Processing image (cropped " << borderSize << "px border)" << endl;
        cout << "Processing size: " << processImg.cols << "x" << processImg.rows << endl;
    }
    else
    {
        processImg = srcImg.clone();
        cout << "Image too small for border cropping, using original image" << endl;
    }
    
    // 配置装甲板检测参数
    ArmorParam param;
    param.enemy_color = BLUE;  // 蓝色装甲板，红色则设为 RED
    param.brightness_threshold = 200;  // 亮度阈值
    
    // 优化后的参数
    param.light_min_area = 500.0f;  // 最小灯条面积
    param.light_max_ratio = 1.0f;  // 灯条宽高比
    param.light_contour_min_solidity = 0.5f;  // 灯条凸度
    param.light_max_angle_diff_ = 12.0f;  // 灯条角度差
    param.light_max_height_diff_ratio_ = 0.30f;  // 灯条高度差比例
    param.light_max_y_diff_ratio_ = 0.30f;  // 灯条Y坐标差比例
    param.light_min_x_diff_ratio_ = 0.5f;  // 灯条X坐标差比例
    param.armor_min_aspect_ratio_ = 1.8f;  // 装甲板最小宽高比
    param.armor_max_aspect_ratio_ = 3.4f;  // 装甲板最大宽高比
    
    // 检测装甲板
    ArmorDetector detector(param);
    detector.loadImg(processImg);
    int result = detector.detect();
    
    if(result == ArmorDetector::ARMOR_LOCAL)
    {
        cout << "\n✓ Armor detected successfully!" << endl;
        
        vector<Point2f> armorVertex = detector.getArmorVertex();
        int armorType = detector.getArmorType();
        
        cout << "Armor type: " << (armorType == SMALL_ARMOR ? "Small" : "Big") << endl;
        cout << "Armor vertices:" << endl;
        for(size_t i = 0; i < armorVertex.size(); i++)
        {
            cout << "  P" << i << ": (" << (int)armorVertex[i].x 
                 << ", " << (int)armorVertex[i].y << ")" << endl;
        }
        
        Mat detectImg = detector.getDebugImg();
        
        // 相机内参和畸变系数（根据实际相机标定结果）
        // 这些参数应该通过相机标定获得
        // TODO: 考虑从配置文件读取这些参数
        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
            9.28130989e+02, 0, 3.77572945e+02,
            0, 9.30138391e+02, 2.83892859e+02,
            0, 0, 1.0);
        
        cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
            -2.54433647e-01, 5.69431382e-01, 3.65405229e-03, 
            -1.09433818e-03, -1.33846840e+00);
        
        // 使用PnP求解器计算3D位姿
        PnPSolver pnpSolver(cameraMatrix, distCoeffs);
        
        if(pnpSolver.solve(armorVertex, armorType))
        {
            cout << "\n✓ PnP solved successfully!" << endl;
            
            Vec3d pos = pnpSolver.getPosition();
            Vec3d rot = pnpSolver.getRotation();
            double dist = pnpSolver.getDistance();
            
            cout << "\n=== 3D Position (Camera Coordinate System) ===" << endl;
            cout << "X: " << pos[0] << " m" << endl;
            cout << "Y: " << pos[1] << " m" << endl;
            cout << "Z: " << pos[2] << " m" << endl;
            cout << "Distance: " << dist << " m" << endl;
            
            cout << "\n=== 3D Rotation (Rodrigues Vector) ===" << endl;
            cout << "RX: " << rot[0] << " rad" << endl;
            cout << "RY: " << rot[1] << " rad" << endl;
            cout << "RZ: " << rot[2] << " rad" << endl;
            
            // 在图像上绘制坐标轴和结果
            Mat resultImg = detectImg.clone();
            pnpSolver.drawAxis(resultImg, 0.1f);  // 绘制坐标轴
            pnpSolver.drawResult(resultImg);  // 绘制位置信息
            
            // 保存和显示结果
            imwrite("detection_result.jpg", detectImg);
            imwrite("final_result.jpg", resultImg);
            
            cout << "\n✓ Results saved:" << endl;
            cout << "  - detection_result.jpg" << endl;
            cout << "  - final_result.jpg" << endl;
            
            namedWindow("Detection Result", WINDOW_NORMAL);
            imshow("Detection Result", detectImg);
            
            namedWindow("Final Result with PnP", WINDOW_NORMAL);
            imshow("Final Result with PnP", resultImg);
            
            cout << "\nPress any key to exit..." << endl;
            waitKey(0);
            return 0;
        }
        else
        {
            cerr << "\n✗ PnP solve failed!" << endl;
            return -1;
        }
    }
    else
    {
        cerr << "\n✗ Armor detection failed!" << endl;
        cerr << "Tips:" << endl;
        cerr << "  - Check if the image contains armor plates" << endl;
        cerr << "  - Adjust brightness_threshold parameter" << endl;
        cerr << "  - Verify enemy_color setting (BLUE or RED)" << endl;
        return -1;
    }
}