// OpenGL
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>

// OpenCV
#include <opencv2/opencv.hpp>

// g2o
#define G2O_USE_VENDORED_CERES
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

// Standard
#include <chrono>
#include <cstdio>
#include <unordered_map>
#include <vector>

int main()
{
    return 0;
}