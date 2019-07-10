project(imageBasics)

add_executable(imageBasics imageBasics.cpp)
# add link OPENCV
target_link_libraries(imageBasics ${OpenCV_LIBS})

add_executable(undistortImage undistortImage.cpp)
# add link OPENCV
target_link_libraries(undistortImage ${OpenCV_LIBS})

