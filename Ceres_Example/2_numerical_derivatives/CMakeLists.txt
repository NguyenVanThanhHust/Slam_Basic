cmake_minimum_required(VERSION 2.8)

project(helloworld)

set (CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# helloworld
add_executable(num_diff num_diff.cpp)
target_link_libraries(num_diff ${CERES_LIBRARIES})
