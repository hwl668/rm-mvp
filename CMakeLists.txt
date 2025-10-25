cmake_minimum_required(VERSION 3.14)
project(armor_vision CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ARMOR_COMMON_SOURCES
  src/io.cpp
  src/color_selector.cpp
  src/lightbar_detector.cpp
  src/pairing.cpp
  src/plate_corners.cpp
  src/pnp_solver.cpp
  src/visualizer.cpp
)

# 主程序
add_executable(armor_vision
  ${ARMOR_COMMON_SOURCES}
  src/main.cpp
)
target_include_directories(armor_vision PRIVATE include)
target_link_libraries(armor_vision PRIVATE ${OpenCV_LIBS} yaml-cpp)

# Step2 主程序（逐帧处理）
add_executable(step2_main
  src/io.cpp
  src/color_selector.cpp
  src/lightbar_detector.cpp
  tools/step2_main.cpp
)
target_include_directories(step2_main PRIVATE include)
target_link_libraries(step2_main PRIVATE ${OpenCV_LIBS} yaml-cpp)

# 独立调试程序 step3_main
add_executable(step3_main
  ${ARMOR_COMMON_SOURCES}
  tools/step3_main.cpp
)
target_include_directories(step3_main PRIVATE include)
target_link_libraries(step3_main PRIVATE ${OpenCV_LIBS} yaml-cpp)

add_executable(step4_main
  ${ARMOR_COMMON_SOURCES}
  tools/step4_main.cpp
)
target_include_directories(step4_main PRIVATE include)
target_link_libraries(step4_main PRIVATE ${OpenCV_LIBS} yaml-cpp)

