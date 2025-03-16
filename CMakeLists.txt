cmake_minimum_required(VERSION 3.10)
project(FaceLib)

# 指定 C++ 版本
set(CMAKE_CXX_STANDARD 14)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_VERSION}")
else()
    message(FATAL_ERROR "OpenCV not found!")
endif()

set(BUILD_SHARED_LIBS ON)

# 查找 dlib
find_package(dlib REQUIRED)
if (dlib_FOUND)
    message(STATUS "dlib found")
else()
    message(FATAL_ERROR "dlib not found!")
endif()

# 设置 JNI 头文件路径
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${dlib_INCLUDE_DIRS})
include_directories($ENV{JAVA_HOME}/include)
include_directories($ENV{JAVA_HOME}/include/linux) # Windows 用 `win32`

# 生成共享库
add_library(FaceLib SHARED src/FaceLib.cpp)

# 链接 OpenCV 和 dlib
target_link_libraries(FaceLib ${OpenCV_LIBS} dlib::dlib)

