cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(slam_sample LANGUAGES CXX CUDA)

option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

set(CMAKE_BUILD_TYPE "Release")
set(OpenGL_GL_PREFERENCE  GLVND)
add_definitions("-DENABLE_SSE")

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 -pthread -Wall -Ofast -Wfatal-errors -D_MWAITXINTRIN_H_INCLUDED")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(OpenCV 4 REQUIRED)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

FIND_PACKAGE(Boost COMPONENTS program_options REQUIRED)

message(STATUS "Gstreamer source: ${GSTREAMER_INCLUDE_DIR}")
include_directories(
    /usr/include/glib-2.0
    /usr/local/cuda/include
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    src
    )

link_directories(/usr/local/cuda/lib64)
link_directories(${GSTLIBS_LIBRARY_DIRS})

set(MAIN_SRC src)
add_executable(${PROJECT_NAME}
    ${MAIN_SRC}/main.cpp
    )

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    ${Boost_LIBRARIES} 
    ${OpenCV_LIBS}
    cudart
    cuda
)

SET(THREADS_PREFER_PTHREAD_FLAG ON)
FIND_PACKAGE(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

FIND_PACKAGE(OpenCV REQUIRED)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

FIND_PACKAGE(glfw3 3.3 REQUIRED)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE "")
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

SET(OpenGL_GL_PREFERENCE "GLVND")
FIND_PACKAGE(OpenGL REQUIRED)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${OPENGL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENGL_LIBRARIES})

LIST(APPEND CMAKE_MODULE_PATH "/opt/g2o/cmake_modules")
FIND_PACKAGE(G2O REQUIRED)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${G2O_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} g2o_solver_eigen g2o_types_sba g2o_types_slam3d)

FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)