project(g2o_ba_example)

cmake_minimum_required(VERSION 3.18)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

add_executable(g2o_ba_example g2o_ba_example.cpp)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(g2o_ba_example ${OpenCV_LIBS})

# Eigen
include_directories("/usr/include/eigen3")

# Pangoline
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(g2o_ba_example ${Pangolin_LIBRARIES})



