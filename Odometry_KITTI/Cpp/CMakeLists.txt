project(visual_odometry)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++11 -O3")
cmake_minimum_required(VERSION 3.18)

file(GLOB viso_files
    "src/*.h"
    "src/*.cpp"
)

add_executable(visu_odo 
    "src/headers.h"
    "src/Constants.h"
    "src/main.cpp"
    "src/utils.cpp"

)

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(visu_odo ${OpenCV_LIBS})

# Eigen
include_directories("/usr/include/eigen3")

# Boost
find_package( Boost 1.58 COMPONENTS system filesystem REQUIRED )
include_directories( ${Boost_INCLUDE_DIR} )
target_link_libraries(visu_odo ${Boost_LIBRARIES} )