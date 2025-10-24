cmake_minimum_required(VERSION 3.10)
project(ArmorDetection)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")

# 包含头文件目录
include_directories(${PROJECT_SOURCE_DIR}/include)

# 源文件
set(SOURCES
    src/main.cpp
    src/ArmorDetector.cpp
    src/PnPSolver.cpp
)

# 生成可执行文件
add_executable(armor_detection ${SOURCES})

# 链接OpenCV库
target_link_libraries(armor_detection ${OpenCV_LIBS})

# 设置输出目录 - 直接输出到项目根目录
set_target_properties(armor_detection PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

message(STATUS "Executable will be output to: ${PROJECT_SOURCE_DIR}/armor_detection")