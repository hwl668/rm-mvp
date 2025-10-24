#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
    string imagePath = (argc > 1) ? argv[1] : "test.jpg";
    Mat srcImg = imread(imagePath);
    
    if(srcImg.empty())
    {
        cerr << "Cannot load image: " << imagePath << endl;
        return -1;
    }
    
    cout << "Image loaded: " << srcImg.cols << "x" << srcImg.rows << endl;
    
    // 显示原图
    namedWindow("Original", WINDOW_NORMAL);
    imshow("Original", srcImg);
    
    // 转灰度
    Mat gray;
    cvtColor(srcImg, gray, COLOR_BGR2GRAY);
    imshow("Gray", gray);
    
    // 多个阈值测试
    for(int thresh = 150; thresh <= 220; thresh += 20)
    {
        Mat binary;
        threshold(gray, binary, thresh, 255, THRESH_BINARY);
        
        string name = "Threshold " + to_string(thresh);
        imshow(name, binary);
        
        // 查找轮廓
        vector<vector<Point>> contours;
        findContours(binary.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        
        cout << "Threshold " << thresh << ": Found " << contours.size() << " contours" << endl;
        
        // 绘制轮廓
        Mat contourImg = srcImg.clone();
        drawContours(contourImg, contours, -1, Scalar(0, 255, 0), 2);
        imshow(name + " Contours", contourImg);
    }
    
    cout << "\nPress any key to continue..." << endl;
    waitKey(0);
    
    return 0;
}