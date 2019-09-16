cmake_minimum_required(VERSION 2.8)
project(useSophus)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")
add_executable(useSophus useSophus.cpp)

#add_subdirectory(example)
