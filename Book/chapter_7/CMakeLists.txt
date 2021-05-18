cmake_minimum_required(VERSION 2.8)
project(vo1)

set(CMAKE_BUILD_TYPE "Release")
set(OpenGL_GL_PREFERENCE  GLVND)
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${SSE_FLAGS} -msse4")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# set(CMAKE_PREFIX_PATH "/home/thanhnv/Downloads/g2o/build" )
set(G2O_INCLUDE_DIRS "/usr/local/include/g2o")

find_package(OpenCV 4 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt)
# find_package(G2O REQUIRED)

# Print some status
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "Sophus library status:")
message(STATUS "    version: ${Sophus_VERSION}")
message(STATUS "    libraries: ${Sophus_LIBS}")
message(STATUS "    include path: ${Sophus_INCLUDE_DIRS}")

message(STATUS "G2o library status:")
message(STATUS "    version: ${G2O_VERSION}")
message(STATUS "    libraries: ${G2O_LIBS}")
message(STATUS "    include path: ${G2O_INCLUDE_DIR}")

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        /usr/local/include/g2o/
        /usr/include/eigen3/
)
# add_subdirectory(../../../fmt/src)

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS}
fmt::fmt
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_eigen.so
/home/thanhnv/Downloads/g2o/lib/libg2o_calibration_odom_laser.so
/home/thanhnv/Downloads/g2o/lib/libg2o_cli.so
/home/thanhnv/Downloads/g2o/lib/libg2o_core.so
/home/thanhnv/Downloads/g2o/lib/libg2o_csparse_extension.so
/home/thanhnv/Downloads/g2o/lib/libg2o_ext_freeglut_minimal.so
/home/thanhnv/Downloads/g2o/lib/libg2o_incremental.so
/home/thanhnv/Downloads/g2o/lib/libg2o_interactive.so
/home/thanhnv/Downloads/g2o/lib/libg2o_interface.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_eigen.so
/home/thanhnv/Downloads/g2o/lib/libg2o_opengl_helper.so
/home/thanhnv/Downloads/g2o/lib/libg2o_parser.so
/home/thanhnv/Downloads/g2o/lib/libg2o_simulator.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_cholmod.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_csparse.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_dense.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_pcg.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_slam2d_linear.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_structure_only.so
/home/thanhnv/Downloads/g2o/lib/libg2o_stuff.so
/home/thanhnv/Downloads/g2o/lib/libg2o_tutorial_slam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_data.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_icp.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sba.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sclam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sim3.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam2d_addons.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam3d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam3d_addons.so
)


add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS}
fmt::fmt
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_eigen.so
/home/thanhnv/Downloads/g2o/lib/libg2o_calibration_odom_laser.so
/home/thanhnv/Downloads/g2o/lib/libg2o_cli.so
/home/thanhnv/Downloads/g2o/lib/libg2o_core.so
/home/thanhnv/Downloads/g2o/lib/libg2o_csparse_extension.so
/home/thanhnv/Downloads/g2o/lib/libg2o_ext_freeglut_minimal.so
/home/thanhnv/Downloads/g2o/lib/libg2o_incremental.so
/home/thanhnv/Downloads/g2o/lib/libg2o_interactive.so
/home/thanhnv/Downloads/g2o/lib/libg2o_interface.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_eigen.so
/home/thanhnv/Downloads/g2o/lib/libg2o_opengl_helper.so
/home/thanhnv/Downloads/g2o/lib/libg2o_parser.so
/home/thanhnv/Downloads/g2o/lib/libg2o_simulator.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_cholmod.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_csparse.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_dense.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_pcg.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_slam2d_linear.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_structure_only.so
/home/thanhnv/Downloads/g2o/lib/libg2o_stuff.so
/home/thanhnv/Downloads/g2o/lib/libg2o_tutorial_slam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_data.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_icp.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sba.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sclam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sim3.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam2d_addons.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam3d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam3d_addons.so
)


add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBS}
fmt::fmt
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_eigen.so
/home/thanhnv/Downloads/g2o/lib/libg2o_calibration_odom_laser.so
/home/thanhnv/Downloads/g2o/lib/libg2o_cli.so
/home/thanhnv/Downloads/g2o/lib/libg2o_core.so
/home/thanhnv/Downloads/g2o/lib/libg2o_csparse_extension.so
/home/thanhnv/Downloads/g2o/lib/libg2o_ext_freeglut_minimal.so
/home/thanhnv/Downloads/g2o/lib/libg2o_incremental.so
/home/thanhnv/Downloads/g2o/lib/libg2o_interactive.so
/home/thanhnv/Downloads/g2o/lib/libg2o_interface.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_eigen.so
/home/thanhnv/Downloads/g2o/lib/libg2o_opengl_helper.so
/home/thanhnv/Downloads/g2o/lib/libg2o_parser.so
/home/thanhnv/Downloads/g2o/lib/libg2o_simulator.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_cholmod.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_csparse.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_dense.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_pcg.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_slam2d_linear.so
/home/thanhnv/Downloads/g2o/lib/libg2o_solver_structure_only.so
/home/thanhnv/Downloads/g2o/lib/libg2o_stuff.so
/home/thanhnv/Downloads/g2o/lib/libg2o_tutorial_slam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_data.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_icp.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sba.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sclam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_sim3.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam2d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam2d_addons.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam3d.so
/home/thanhnv/Downloads/g2o/lib/libg2o_types_slam3d_addons.so
)
