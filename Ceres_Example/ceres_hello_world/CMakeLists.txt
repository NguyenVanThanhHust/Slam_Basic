cmake_minimum_required(VERSION 2.8)

project(helloworld)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# helloworld
add_executable(helloworld helloworld.cpp)
target_link_libraries(helloworld ${CERES_LIBRARIES})
