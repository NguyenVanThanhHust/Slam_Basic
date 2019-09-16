cmake_minimum_required(VERSION 2.8)

project(helloworld)

set (CMAKE_CXX_STANDARD 11)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# helloworld
add_executable(analytic_diff analytic_diff.cpp)
target_link_libraries(analytic_diff ${CERES_LIBRARIES})
