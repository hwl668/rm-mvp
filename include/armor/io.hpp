#pragma once
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <string>

namespace armor {

struct Camera {
  cv::Mat K;           // 3x3, CV_64F
  cv::Mat dist;        // 1x5 或 1x8, CV_64F
  int width = 0;       // 当前输入分辨率
  int height = 0;
  // 标定分辨率（用于K缩放）
  int calib_width = 0;
  int calib_height = 0;
};

// 从 camera.yaml 读取相机参数（不做打开视频）
Camera LoadCamera(const std::string& yaml_path);

// 按当前帧分辨率适配内参（若与标定不同）
void AdaptIntrinsicsToFrame(Camera& cam, int curr_w, int curr_h);

// 简单视频读取器：既可打开文件也可打开摄像头
class VideoReader {
public:
  // source 可以是 "data/videos/demo.mp4" 或 "0"/"1" 表示摄像头索引
  bool open(const std::string& source, int api_pref = cv::CAP_ANY);
  bool read(cv::Mat& frame);
  void close();
  bool reset();
  bool isOpened() const { return cap_.isOpened(); }
  bool isCamera() const { return is_camera_; }
  double fps() const;              // 文件能返回FPS，摄像头可能为0
  cv::Size size() const;           // 返回当前流的分辨率
  std::string source() const { return source_; }

private:
  cv::VideoCapture cap_;
  bool is_camera_ = false;
  std::string source_;
  std::string opened_path_;
};

} // namespace armor
