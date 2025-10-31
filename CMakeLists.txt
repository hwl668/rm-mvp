cmake_minimum_required(VERSION 3.10)
project(rv-mvp VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(OpenCV REQUIRED core imgproc highgui videoio calib3d)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# 包含路径
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# 源码
add_executable(rv_mvp
  app/main.cpp
  src/armor.cpp
  src/detector.cpp
  src/solver.cpp
  tools/math_tools.cpp
)

# 链接
target_link_libraries(rv_mvp
  ${OpenCV_LIBS}
  yaml-cpp
)

# Windows/WSL 控制台 UTF-8（可选）
if (MSVC)
  target_compile_options(rv_mvp PRIVATE /utf-8)
endif()

# 运行目录（可选，把 configs/ 和 data/ 当作与可执行同级）
set_target_properties(rv_mvp PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
)
