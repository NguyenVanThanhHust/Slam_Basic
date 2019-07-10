project(imageBasics)

set(CMAKE_CXX_FLAGS "-std=c++11")
add_executable(imageBasics imageBasics.cpp)
# add link OPENCV
find_package( OpenCV REQUIRED )
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# add link OPENCV
target_link_libraries(undistortImage ${OpenCV_LIBS})

