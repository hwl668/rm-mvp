rv-mvp
Robomaster 装甲板最小可行识别与位姿求解（C++/OpenCV 4.5.4 + Eigen + yaml-cpp）。
输入视频逐帧检测灯条 → 组合装甲板 → solvePnP 计算位姿 → 在画面上绘制识别框与 PnP 坐标轴。
已提供“一帧一帧手动切换”与“实时播放”两种模式。

目录结构
rv-mvp/
├─ app/
│  └─ main.cpp             # 程序入口（显示窗口、按键控制、HUD）
├─ include/
│  ├─ armor.hpp            # Armor/Lightbar 结构体定义
│  ├─ detector.hpp         # 灯条与装甲检测
│  └─ solver.hpp           # PnP 与坐标变换
├─ src/
│  ├─ armor.cpp
│  ├─ detector.cpp
│  └─ solver.cpp
├─ tools/
│  ├─ math_tools.hpp       # 角度限制、欧拉角/矩阵互转、坐标变换等通用函数
│  └─ math_tools.cpp
├─ configs/
│  ├─ auto_aim.yaml        # 检测与几何约束的参数
│  └─ camera.yaml          # 相机内参、畸变、外参（相机-云台-IMU）
├─ data/
│  ├─ arm.avi              # 示例视频（自行放置）
│  └─ answer/              # 可放评测/结果截图（可选）
├─ CMakeLists.txt
└─ README.md


依赖


C++17（GCC 9.4 测试通过）


OpenCV 4.5.4（core, imgproc, highgui, videoio, calib3d）


Eigen3


yaml-cpp


（可选）GStreamer/FFmpeg 解码组件，提升视频兼容性


Ubuntu 20.04 示例安装（若缺失）：
sudo apt-get update
sudo apt-get install libeigen3-dev libyaml-cpp-dev
# OpenCV 建议使用你已编译的 4.5.4；若用系统版本，请自行调整 CMake 的 OpenCV 路径
# GStreamer/FFmpeg 常见组件（可选）
sudo apt-get install gstreamer1.0-libav gstreamer1.0-plugins-base \
                     gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
                     gstreamer1.0-plugins-ugly


编译
cd rv-mvp
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j


如果 CMake 提示找不到 Eigen3，确认已安装 libeigen3-dev，或设置 Eigen3_DIR。


运行
方式一：工程根目录运行
./build/rv_mvp               # 默认读取 data/arm.avi
./build/rv_mvp path/to/video # 指定视频路径

方式二：build 目录运行
cd build
./rv_mvp ../data/arm.avi


程序会尝试多种后端打开视频：FFmpeg → GStreamer → Any。若打不开，检查路径与编解码器。


按键与模式


实时播放模式（默认，一帧等待时间 1ms）


q / ESC：退出


space：切换到“逐帧模式”




逐帧模式


n：下一帧


p：上一帧（如果实现了缓存；当前默认“只前进”）


space：切回“实时播放模式”


q / ESC：退出





如果你只需要“逐帧点进”，可以在 configs/auto_aim.yaml 把 step_mode: true（若你在 main.cpp 暴露了该开关），或者直接用每帧 waitKey() 控制。


可视化说明


绿色四边形：装甲板四个顶点与连线（检测结果）


PnP 坐标轴（画在最佳装甲中心）：


X：蓝线


Y：绿线


Z：红线




左上角 HUD（帧号、FPS、检测数量、距离、YPR、归一化中心、装甲大小类型）



如果你希望“显示窗上不要有参数”，可以在 main.cpp 中关闭 drawHUD(...) 调用。


配置文件
configs/camera.yaml


相机内参（示例）
camera_matrix: [928.130989, 0, 377.572945,
                0, 930.138391, 283.892859,
                0, 0, 1.0]
distort_coeffs: [-0.254433647, 0.569431382, 0.00365405229, -0.00109433818, -1.33846840]


已按你提供的内参/畸变填入，保持 3×3、1×5 顺序。



外参（示例为单位阵/零向量，可按需求标定）
R_gimbal2imubody: [1,0,0, 0,1,0, 0,0,1]
R_camera2gimbal:  [1,0,0, 0,1,0, 0,0,1]
t_camera2gimbal:  [0,0,0]



configs/auto_aim.yaml
常用可调项（命名以你现有实现为准）：


min_lightbar_length：最短灯条长度（像素）


min_lightbar_ratio：灯条长宽比阈值（过滤“矮胖”的伪灯条）


max_side_ratio：两边灯条长度比（长/短）。用来抑制“真实灯条 + 短反光”被误配


例：从 1.6 降到 1.30 ~ 1.20，可显著减少“短反光”拼对。




min_armor_ratio / max_armor_ratio：两灯条几何间距/角度比例范围


binary_threshold / use_otsu / invert_binary：二值化相关


hsv_lower / hsv_upper：颜色掩膜范围（如只识别某方颜色）



经验：先用 min_lightbar_length 和 min_lightbar_ratio 把明显短/胖的反光踢掉；再用 max_side_ratio 防止左右不对称配对；最后微调 min_armor_ratio/max_armor_ratio。


调参建议（针对“反光短、距离相等”的场景）


把 max_side_ratio 调小（更严格）


例如：1.60 → 1.30 → 1.20，观察误配下降情况。




略增大 min_lightbar_length（+10% ~ +20%），把很短的反光先在“灯条阶段”过滤。


提高 min_lightbar_ratio（长宽比），反光通常更矮胖。



常见问题


视频打不开 / GStreamer 报错


确认路径（相对/绝对），在 build 目录运行时请用：./rv_mvp ../data/arm.avi


安装视频解码组件（见上文依赖）


用 CAP_FFMPEG 优先打开：cap.open(path, cv::CAP_FFMPEG)




前 N 帧识别不到，后面正常


可能是阈值/颜色掩膜在开场光照条件下过严；降低 binary_threshold 或开启 use_otsu；


放宽 min_lightbar_length（如果开场目标较小/远）；


开启调试可视化（在 Detector 中添加/查看中间图）定位是哪一步丢了。




有些帧两条灯条误配到“灯条+反光”


按上文调 max_side_ratio、min_lightbar_length、min_lightbar_ratio。




坐标轴与目标姿态不一致


确认 R_camera2gimbal / t_camera2gimbal / R_gimbal2imubody 是否正确；


确认欧拉角顺序（本项目使用 ZYX，即 yaw-pitch-roll）。





参考与致谢


SEU-SuperNova-CVRA / Tongji / SJTU 等开源实现思路


OpenCV 官方文档（calib3d、solvePnP、Rodrigues）


Eigen 官方文档（旋转矩阵/四元数/欧拉角）



若后续需要加入更强的鲁棒性（多帧融合、稳定跟踪、SOT/SORT/卡尔曼滤波），可在 solver 外再包一层 Tracker。


许可证
MIT（若你需要其它协议，请自行替换并在仓库根目录放置 LICENSE）。

快速清单（TL;DR）


把视频放到 data/arm.avi


按“依赖”装包 → mkdir build && cd build && cmake .. && make -j


运行：


工程根：./build/rv_mvp


build：./rv_mvp ../data/arm.avi




逐帧模式：按 space 切换后用 n 看下一帧


有短反光误配：调小 max_side_ratio，增大 min_lightbar_length，提高 min_lightbar_ratio


