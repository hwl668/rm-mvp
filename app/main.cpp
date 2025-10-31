#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <iostream>
#include <iomanip>


#include "armor.hpp"
#include "detector.hpp"
#include "solver.hpp"
#include "tools/math_tools.hpp"  // 用于 rotation_matrix / limit_rad 等

using namespace auto_aim;

static void drawArmorBox(cv::Mat& img, const Armor& a) {
  // 绿色四边形
  for (int i = 0; i < 4; ++i) {
    cv::line(img, a.points[i], a.points[(i+1)%4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::circle(img, a.points[i], 3, cv::Scalar(0, 255, 255), -1, cv::LINE_AA);
  }
}

static void drawAxes(cv::Mat& img, const Solver& solver, const Armor& a, double axis_len_m = 0.1) {
  // 用已求解的位姿在装甲中心画三轴（世界坐标下的装甲姿态）
  // 1) 由 ypr_in_world 得到 R_armor2world
  Eigen::Matrix3d R_armor2world = tools::rotation_matrix(a.ypr_in_world);

  // 2) 在装甲系定义三个轴终点（m）
  Eigen::Vector3d origin_w = a.xyz_in_world;
  Eigen::Vector3d x_end_w  = origin_w + R_armor2world * (Eigen::Vector3d(1,0,0) * axis_len_m);
  Eigen::Vector3d y_end_w  = origin_w + R_armor2world * (Eigen::Vector3d(0,1,0) * axis_len_m);
  Eigen::Vector3d z_end_w  = origin_w + R_armor2world * (Eigen::Vector3d(0,0,1) * axis_len_m);

  // 3) 投影到像素
  std::vector<cv::Point3f> worldPts;
  worldPts.emplace_back(origin_w.x(), origin_w.y(), origin_w.z());
  worldPts.emplace_back(x_end_w.x(),  x_end_w.y(),  x_end_w.z());
  worldPts.emplace_back(y_end_w.x(),  y_end_w.y(),  y_end_w.z());
  worldPts.emplace_back(z_end_w.x(),  z_end_w.y(),  z_end_w.z());

  std::vector<cv::Point2f> pix = solver.world2pixel(worldPts);
  if (pix.size() != 4) return; // 可能被裁掉

  // 4) 画出三轴（X=蓝、Y=绿、Z=红）
  cv::line(img, pix[0], pix[1], cv::Scalar(255, 0, 0),   2, cv::LINE_AA);
  cv::line(img, pix[0], pix[2], cv::Scalar(0, 255, 0),   2, cv::LINE_AA);
  cv::line(img, pix[0], pix[3], cv::Scalar(0, 0, 255),   2, cv::LINE_AA);

  // 轴名
  cv::putText(img, "X", pix[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,0,0), 2, cv::LINE_AA);
  cv::putText(img, "Y", pix[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,255,0), 2, cv::LINE_AA);
  cv::putText(img, "Z", pix[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0,0,255), 2, cv::LINE_AA);
}

static void drawHUD(cv::Mat& img, double fps,
                    const Armor* best, // 可以为空
                    int frame_id, int armor_cnt)
{
  // 左上角信息框
  int x = 12, y = 18, lh = 22;
  auto put = [&](const std::string& s, const cv::Scalar& c=cv::Scalar(255,255,255)) {
    cv::putText(img, s, cv::Point(x,y), cv::FONT_HERSHEY_SIMPLEX, 0.6, c, 2, cv::LINE_AA);
    y += lh;
  };
  cv::rectangle(img, cv::Rect(5,5, 370, 5 + (best? 7:3)*lh), cv::Scalar(30,30,30), cv::FILLED);
  cv::rectangle(img, cv::Rect(5,5, 370, 5 + (best? 7:3)*lh), cv::Scalar(80,80,80), 1);

  std::ostringstream oss;
  oss << "Frame: " << frame_id << "   FPS: " << std::fixed << std::setprecision(1) << fps;
  put(oss.str());
  put("Detections: " + std::to_string(armor_cnt));

  if (best) {
    const auto& a = *best;
    // 距离与姿态
    double dist = a.ypd_in_world[2];
    double yaw_deg   = a.ypr_in_world[0] * 180.0 / CV_PI;
    double pitch_deg = a.ypr_in_world[1] * 180.0 / CV_PI;
    double roll_deg  = a.ypr_in_world[2] * 180.0 / CV_PI;

    std::ostringstream p1, p2, p3, p4;
    p1 << "Dist: " << std::fixed << std::setprecision(3) << dist << " m";
    p2 << "YPR(deg): " << std::fixed << std::setprecision(1)
       << yaw_deg << ", " << pitch_deg << ", " << roll_deg;
    p3 << "Center(norm): (" << std::fixed << std::setprecision(3)
       << a.center_norm.x << ", " << a.center_norm.y << ")";
    p4 << "Type: " << (a.type==ArmorType::big ? "big" : "small");

    put(p1.str());
    put(p2.str());
    put(p3.str());
    put(p4.str());
  }
}

int main(int argc, char** argv) {
  // 固定路径（按你需求）
  const std::string det_cfg     = "configs/auto_aim.yaml";
  const std::string solver_cfg  = "configs/camera.yaml";

  // ---- 打开视频（不依赖 filesystem）----
std::string video_path = (argc > 1) ? argv[1] : "data/arm.avi";

cv::VideoCapture cap;  // 先声明，再在 lambda 里按引用使用

auto try_open_video = [&](const std::string& path)->bool{
  cv::VideoCapture tmp;
  if (tmp.open(path, cv::CAP_FFMPEG)) { cap = std::move(tmp); return true; }
  if (cap.open(path, cv::CAP_GSTREAMER))  return true;
  if (cap.open(path, cv::CAP_ANY))        return true;
  return false;
};

bool ok = false;

// 1) 直接用传入或默认路径
ok = try_open_video(video_path);

// 2) 常见从 build 目录运行：尝试 ../data/arm.avi
if (!ok) ok = try_open_video("../data/arm.avi");

// 3) 再试 ./data/arm.avi（工程根）
if (!ok) ok = try_open_video("./data/arm.avi");

if (!ok) {
  std::cerr << "Failed to open video. Tried:\n"
            << "  " << video_path << "\n"
            << "  ../data/arm.avi\n"
            << "  ./data/arm.avi\n"
            << "Tips: 在 build 目录运行： ./rv_mvp ../data/arm.avi\n"
            << "      或在工程根运行：   ./build/rv_mvp\n";
  return 1;
}
// ---- 打开视频结束 ----



  // 初始化模块
  Detector detector(det_cfg, /*debug=*/false);
  Solver solver(solver_cfg);

  cv::namedWindow("rv-mvp", cv::WINDOW_NORMAL);
  cv::resizeWindow("rv-mvp", 1280, 720);

  int frame_id = 0;
  auto t_prev = std::chrono::high_resolution_clock::now();
  double fps = 0.0;

  while (true) {
    // 读并显示第一帧
cv::Mat frame;
if (!cap.read(frame)) {
  std::cerr << "Video has no frames.\n";
  return 1;
}
int frame_id = 1;
double fps = 0.0;  // 单步模式不需要实时FPS，显示0或"--"都行

auto render = [&](cv::Mat& f){
  // 检测
  auto armors = detector.detect(f, frame_id);

  // 选一个“最佳”（简单按框面积大）
  Armor* best = nullptr;
  double best_area = -1.0;
  for (auto& a : armors) {
    solver.solve(a);
    drawArmorBox(f, a);
    cv::Rect r = cv::boundingRect(std::vector<cv::Point2f>(a.points.begin(), a.points.end()));
    double area = r.area();
    if (area > best_area) { best_area = area; best = &a; }
  }
  if (best) drawAxes(f, solver, *best, 0.12);

  drawHUD(f, fps, best, frame_id, static_cast<int>(armors.size()));
  cv::imshow("rv-mvp", f);
};

// 先渲染第一帧
render(frame);

// 单步循环：按键才推进
while (true) {
  int key = cv::waitKey(0) & 0xFF;  // 0 表示一直等按键

  if (key == 27 || key == 'q' || key == 'Q') break;     // 退出
  else if (key == 'r' || key == 'R') {                  // 回到第一帧
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    if (!cap.read(frame)) break;
    frame_id = 1;
    render(frame);
  }
  else if (key == 'p' || key == 'P') {                  // 上一帧（不一定所有视频都支持）
    // 当前已经显示的帧索引是 POS_FRAMES-1，所以退回 2 再读一帧
    int cur = static_cast<int>(cap.get(cv::CAP_PROP_POS_FRAMES)); // 下一次要读的帧号
    int tgt = std::max(0, cur - 2);
    if (cap.set(cv::CAP_PROP_POS_FRAMES, tgt) && cap.read(frame)) {
      frame_id = tgt + 1;
      render(frame);
    }
  }
  else if (key == ' ' || key == 'n' || key == 'N') {    // 下一帧 / 单步前进
    if (!cap.read(frame)) break;
    ++frame_id;
    render(frame);
  }
  // 其他键：忽略
}

  }

  return 0;
}
