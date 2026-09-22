cmake_minimum_required(VERSION 3.16)
project(vision_calibration_tools LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)

# apriltag2_bridge: 将 ethz_apriltag2 检测器封装为 C 接口, 供 Python ctypes 调用。
# 依赖 basalt 源码中的 thirdparty/apriltag/ethz_apriltag2 头文件。
# 设置 BASALT_DIR 指向 basalt 源码根目录, 例如:
#   cmake .. -DBASALT_DIR=/path/to/basalt
if(NOT DEFINED BASALT_DIR)
    set(BASALT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/basalt")
endif()

set(APRILTAG2_INCLUDE_DIR "${BASALT_DIR}/thirdparty/apriltag/ethz_apriltag2/include")

if(EXISTS "${APRILTAG2_INCLUDE_DIR}/apriltags/TagDetector.h")
    add_library(apriltag2 SHARED apriltag2_bridge.cpp)
    target_include_directories(apriltag2 PRIVATE ${APRILTAG2_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(apriltag2 PRIVATE ${OpenCV_LIBS})
    set_target_properties(apriltag2 PROPERTIES OUTPUT_NAME apriltag2)
    message(STATUS "apriltag2: will build libapriltag2.so from ${APRILTAG2_INCLUDE_DIR}")
else()
    message(WARNING "apriltag2: ethz_apriltag2 headers not found at ${APRILTAG2_INCLUDE_DIR}. "
                    "Set -DBASALT_DIR=/path/to/basalt to build libapriltag2.so. "
                    "The prebuilt libapriltag2.so in the repo root can be used as-is.")
endif()
