cmake_minimum_required(VERSION 2.8)
project(ch6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3 -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(Ceres_DIR "/home/thanhnv/Downloads/ceres-solver-2.0.0/")

set(CMAKE_PREFIX_PATH "/home/thanhnv/Downloads/g2o/build" )


# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Ceres
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# Eigen
include_directories("/usr/include/eigen3")

add_executable(gaussNewton gaussNewton.cpp)

add_executable(ceresCurveFitting ceresCurveFitting.cpp)

add_executable(g2oCurveFitting g2oCurveFitting.cpp)

target_link_libraries(ceresCurveFitting 
${OpenCV_LIBS}
${CERES_LIBRARIES}
)

target_link_libraries(gaussNewton 
${OpenCV_LIBS}
)

target_link_libraries(g2oCurveFitting 
${OpenCV_LIBS}
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
