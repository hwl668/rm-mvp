 armor_vision

装甲板识别与位姿解算项目 —— 基于 OpenCV 的视频装甲检测与 PnP 姿态求解。  
项目整体结构遵循东南大学开源项目风格，进行了精简与教学化调整，适用于装甲识别算法学习与实验。

---

## 📁 项目结构

armor_vision/
├─ CMakeLists.txt
├─ README.md
├─ config/
│ ├─ camera.yaml # 相机内参与畸变系数
│ └─ params.yaml # 阈值、几何参数、颜色配置
├─ data/
│ ├─ videos/arm.avi # 测试视频（输入）
│ └─ answer.jpg # 结果示例图（坐标轴姿态显示）
├─ include/armor/
│ ├─ io.hpp # 视频读取与配置加载
│ ├─ color_selector.hpp # 自动颜色识别（红/蓝自适应）
│ ├─ lightbar_detector.hpp# 灯条候选与端点提取
│ ├─ pairing.hpp # 灯条配对与几何筛选
│ ├─ plate_corners.hpp # 装甲板角点推算
│ ├─ pnp_solver.hpp # solvePnP 姿态求解
│ ├─ visualizer.hpp # 结果可视化（坐标轴绘制）
│ └─ types.hpp # 数据结构定义
├─ src/
│ ├─ io.cpp
│ ├─ color_selector.cpp
│ ├─ lightbar_detector.cpp
│ ├─ pairing.cpp
│ ├─ plate_corners.cpp
│ ├─ pnp_solver.cpp
│ ├─ visualizer.cpp
│ └─ main.cpp # 最终整合入口（Step4）
└─ tools/
├─ step2_main.cpp # 二值化/灯条检测调试
├─ step3_main.cpp # 灯条配对验证
└─ step4_main.cpp # PnP 求解 + 坐标轴显示

yaml
复制代码

---

## 🧭 功能简介

| 模块 | 功能说明 |
|------|-----------|
| io             | 视频输入、参数加载（YAML） |
| color_selector | 自动检测红/蓝灯条颜色 |
| lightbar_detector | 提取灯条轮廓、端点与角度 |
| pairing        | 匹配成装甲板候选对 |
| plate_corners  | 推算装甲板四角坐标 |
| pnp_solver     | solvePnP 姿态解算 |
| visualizer     | 绘制坐标轴与中心标记 |

---

## ⚙️ 编译与运行

### 1️⃣ 编译项目
```bash
mkdir -p build && cd build
cmake ..
make -j
2️⃣ 运行主程序
bash
复制代码
./step4_main ../data/videos/arm.avi
3️⃣ 运行结果
程序会显示名为 Pose Viewer 的窗口：

展示识别到的装甲板与坐标轴；

X(红)、Y(绿)、Z(蓝) 三维姿态轴；

窗口大小为 1280×960；

不显示调参文本（无参数显示）；

视频播放完自动停在最后一帧。

结果截图示例：

位于 data/answer.jpg

🧩 参数配置
所有参数均在 config/params.yaml 与 config/camera.yaml 中设置。

示例：

yaml
复制代码
color:
  hsv_red1: [0, 50, 50, 12, 255, 255]
  hsv_red2: [168, 50, 50, 180, 255, 255]
  hsv_blue: [95, 40, 40, 135, 255, 255]
  morph_kernel: 3

lightbar:
  min_area: 10
  min_ratio: 1.8
  max_ratio: 15
  angle_upright_deg: 35
📷 示例结果
下图展示了成功识别装甲板并通过 PnP 解算后的姿态可视化（坐标轴与装甲板框）：

<p align="center"> <img src="data/answer.jpg" width="640"/> </p>
📘 环境依赖
C++17

OpenCV ≥ 4.5.4

CMake ≥ 3.10

yaml-cpp（系统库或源码集成）

安装依赖：

bash
复制代码
sudo apt install libopencv-dev libyaml-cpp-dev
📚 学习建议
从 step2_main.cpp 学习视频帧操作与二值化；

在 step3_main.cpp 验证灯条端点与几何匹配；

最终在 step4_main.cpp 运行姿态求解与 3D 坐标轴可视化；

调整 params.yaml 获取稳定识别结果。

📎 致谢
参考东南大学 RoboMaster 视觉组开源项目结构与编码风格。
本项目用于学习、教学与调试，不用于竞赛部署。

🧠 项目作者：
基于 SEU-RM-Vision 教学版简化实现
支持平台： Ubuntu + OpenCV + CMake
最后更新： 2025.10.25

yaml
复制代码

---

复制到你的 `README.md` 文件即可，格式、emoji、代码块、表格都能在 GitHub 上正确渲染。
