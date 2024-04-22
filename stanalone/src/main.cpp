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

class Frame
{
public:
    static inline int id_generator=0;
    int id;
    cv::Mat image_grey;
    cv::Mat Matx33d K;
    cv::Mat dist;
    std::vector<cv::KeyPoint> kps;
    cv::Mat des;
    cv::Matx33d rotation;
    cv::Matx33d translation;
public:
    Frame() = default;
    Frame(const cv::Mat& image_grey, const cv::Matx3eed& K, const cv::Mat& dist) {
        static cv::Ptr<cv::ORB> extractor = cv::ORd::create();
        this->id = Frame::id_generator++;
        this->image_grey = image_grey;
        this->dist = dist;
        std::vector<cv::Point2f> corners;
        cv::goodFeaturesToTrack(image_grey, corners, 3000, 0.01, 7);
        this->kps.reserve(corners.size());
        for (const cv::Point2f& corner: corners)
        {
            this->kps.push_back(cv::KeyPoint(corner, 20));
        }
        extractor->compute(image_grey, this->kps, this->des);
        this->rotation = cv::Matx33d::eye();
        this->translation = cv::Matx31d::zeros();
        std::cout<<"Detected: "<<this->kps.size()<<" features"<<std::endl;
    }
};

class Landmark
{
public:
    static inline int id_generator=0;
    int id;
    cv::Matx31d location;
    cv::Matx31d color;
public:
    Landmark() = default;
    Landmark(const cv::Matx31d &location, const cv::Matx31d& color)
    {
        this->id = Landmark::id_generator++;
        this->location = location;
        this->color = color;
    }
};

class Map {
public:
    std::unordered_map<int, Frame> frames;
    std::unordered_map<int, Landmark> landmarks;
    std::unordered_map<int, std::vector<std::pair<int, int>>> observations;//key in landmark_id,
    // pair are frame_id and kp_index 
public:
    void add_frame(const Frame& frame)
    {
        this->frames[frame.id] = frame
    }
    void add_landmark(const Landmark& landmark)
    {
        this->landmarks[landmark.id] = landmark;
    }
    void add_observation(const Frame& frame, const Landmark& landmark, int kp_index)
    {
        this->observations[landmark.id].push_back({frame.id, kp_index});
    }
    static cv::Matx41d trianguate(const Frame& lhs_frame, const Frame& rhs_frame, 
        const cv::Point2d& lhs_point, const cv::Point2d& rhs_point)
    {
        cv::Matx34d cam1 = {
            lhs_frame.rotation(0,0), lhs_frame.rotation(0,1), lhs_frame.rotation(0,2), lhs_frame.translation(0),
            lhs_frame.rotation(1,0), lhs_frame.rotation(1,1), lhs_frame.rotation(1,2), lhs_frame.translation(1),
            lhs_frame.rotation(2,0), lhs_frame.rotation(2,1), lhs_frame.rotation(2,2), lhs_frame.translation(2),
        };
        cv::Matx34d cam2 = {
            rhs_frame.rotation(0,0), rhs_frame.rotation(0,1), rhs_frame.rotation(0,2), rhs_frame.translation(0),
            rhs_frame.rotation(1,0), rhs_frame.rotation(1,1), rhs_frame.rotation(1,2), rhs_frame.translation(1),
            rhs_frame.rotation(2,0), rhs_frame.rotation(2,1), rhs_frame.rotation(2,2), rhs_frame.translation(2),
        };   
        std::vector<cv::Point2d> lhs_point_normalised;
        cv::undistortPoints(std::vector<cv::Point2d>{lhs_point}, lhs_point_normalised, lhs_frame.K, lhs_frame.dist);
        std::vector<cv::Point2d> rhs_point_normalised;
        cv::undistortPoints(std::vector<cv::Point2d>{rhs_point}, rhs_point_normalised, rhs_frame.K, rhs_frame.dist);
        cv::Mat potential_landmark(4, 1, CV_64F);
        cv::trianguatePoints(cam1, cam2, lhs_point_normalised, rhs_point_normalised, potential_landmark);
        return cv::Matx41d{
            potential_landmark.at<double>(0),
            potential_landmark.at<double>(1),
            potential_landmark.at<double>(2),
            potential_landmark.at<double>(3)
        };
    }
}

void optimise(int local_window, bool fix_landmakrs, int rounds)
{
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linear_solver = std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>();
    std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr = std::make_unique<g2o::BlockSolver_6_3>(std::move(linear_solver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    solver->setWriteDebug(false);
    g2o::SparseOptimizer opt;
    opt.setAlgorithm(solver);

    // Add frames
    int non_fixed_poses = 0;
    int lankmark_it_start = 0;

    const int local_window_below = Frame::id_generator - 1 - local_window
}

int main()
{
    return 0;
}