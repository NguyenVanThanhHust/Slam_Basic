project(eigenGeometry)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

include_directories("/usr/include/eigen3")

add_executable(eigenGeometry eigenGeometry.cpp)
