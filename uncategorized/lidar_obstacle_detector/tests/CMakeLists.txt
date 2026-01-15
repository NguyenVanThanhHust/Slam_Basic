project(test)
cmake_minimum_required(VERSION 3.22)

# Set C++ standard to 17 (required for PCL 1.15)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL 1.10 REQUIRED)

add_executable(
    voxel_grid
    voxel_grid.cpp 
)

target_link_libraries(voxel_grid ${PCL_LIBRARIES})