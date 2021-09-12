project(stereo_image)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")

add_executable(stereoVision stereoVision.cpp)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(stereoVision ${OpenCV_LIBS})

# Eigen
include_directories("/usr/include/eigen3")

# Pangoline
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
target_link_libraries(stereoVision ${Pangolin_LIBRARIES})
