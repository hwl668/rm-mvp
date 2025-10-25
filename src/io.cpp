#include "armor/io.hpp"
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
namespace armor {

static cv::Mat VecToMat(const std::vector<double>& v, int rows, int cols) {
  cv::Mat m(rows, cols, CV_64F);
  std::memcpy(m.data, v.data(), v.size()*sizeof(double));
  return m.clone();
}

Camera LoadCamera(const std::string& yaml_path) {
  YAML::Node node = YAML::LoadFile(yaml_path);
  Camera cam;

  cam.width  = node["image_width"].as<int>();
  cam.height = node["image_height"].as<int>();

  cam.calib_width  = node["calib_width"] .as<int>();
  cam.calib_height = node["calib_height"].as<int>();

  auto Kvec = node["K"].as<std::vector<double>>();
  if (Kvec.size() != 9) throw std::runtime_error("K must have 9 elements");
  cam.K = VecToMat(Kvec, 3, 3);

  auto Dvec = node["dist"].as<std::vector<double>>();
  if (Dvec.size()!=5 && Dvec.size()!=8)
    throw std::runtime_error("dist must have 5 or 8 elements");
  cam.dist = cv::Mat(Dvec).clone().reshape(1,1); // 1xN, CV_64F

  return cam;
}

void AdaptIntrinsicsToFrame(Camera& cam, int curr_w, int curr_h) {
  if (cam.calib_width<=0 || cam.calib_height<=0) return; // 未配置则跳过
  if (curr_w==cam.calib_width && curr_h==cam.calib_height) {
    cam.width = curr_w; cam.height = curr_h;
    return;
  }
  const double sx = static_cast<double>(curr_w) / cam.calib_width;
  const double sy = static_cast<double>(curr_h) / cam.calib_height;

  // K = [fx 0 cx; 0 fy cy; 0 0 1]
  cam.K.at<double>(0,0) *= sx; // fx
  cam.K.at<double>(0,2) *= sx; // cx
  cam.K.at<double>(1,1) *= sy; // fy
  cam.K.at<double>(1,2) *= sy; // cy

  cam.width  = curr_w;
  cam.height = curr_h;
}

static bool IsNumber(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s) if (!std::isdigit(c)) return false;
  return true;
}

bool VideoReader::open(const std::string& source, int api_pref) {
  source_ = source;
  opened_path_.clear();
  is_camera_ = IsNumber(source);

  if (is_camera_) {
    int idx = std::stoi(source);
    if (!cap_.open(idx, api_pref)) {
      std::cerr << "[VideoReader] cannot open camera index " << idx << "\n";
      return false;
    }
  } else {
    // 相对路径优先从工作目录找；找不到再尝试项目下 data/videos/
    fs::path p(source);
    if (!fs::exists(p)) {
      fs::path alt = fs::path("data") / "videos" / source;
      if (fs::exists(alt)) p = alt;
    }
    if (!cap_.open(p.string(), api_pref)) {
      std::cerr << "[VideoReader] cannot open file: " << p << "\n";
      return false;
    }
    opened_path_ = p.string();
    source_ = opened_path_;
  }
  return cap_.isOpened();
}

bool VideoReader::read(cv::Mat& frame) {
  if (!cap_.isOpened()) return false;
  return cap_.read(frame);
}

void VideoReader::close() {
  if (cap_.isOpened()) cap_.release();
}

bool VideoReader::reset() {
  if (!cap_.isOpened()) return false;
  if (is_camera_) return false;

  if (cap_.set(cv::CAP_PROP_POS_FRAMES, 0)) {
    return true;
  }

  std::string path = !opened_path_.empty() ? opened_path_ : source_;
  cap_.release();
  bool ok = cap_.open(path);
  if (ok) {
    opened_path_ = path;
  }
  return ok;
}

double VideoReader::fps() const {
  if (!cap_.isOpened()) return 0.0;
  double f = cap_.get(cv::CAP_PROP_FPS);
  return (std::isfinite(f) && f>0) ? f : 0.0;
}

cv::Size VideoReader::size() const {
  if (!cap_.isOpened()) return {};
  int w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  int h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  return {w,h};
}

} // namespace armor
