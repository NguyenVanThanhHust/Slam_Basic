#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <sophus/se3.hpp>
#include <chrono>
#include <memory>
#include <utility>

using namespace std;
using namespace cv;

// This is how can we estimate camera position with 3d-3d feature matching.

// This is an example for bundle adjustment to refine the 3D coordiate from 2 pair of rgb-depth image

void find_feature_matches(
	const Mat &img_1, const Mat &img_2, 
	std::vector<KeyPoint> &keypoints_1,
	std::vector<KeyPoint> &keypoints_2,
	std::vector<DMatch> &matches
	);

void pose_estimation_3d3d(
	std::vector<Point3f> keypoints_1,
	std::vector<Point3f> keypoints_2,
	Mat &R, Mat &t
	);

// Pixel coordiate to camera normalized ordinate
Point2d pixel2cam(const Point2d &p, const Mat &K);

void bundleAdjustment(
	const std::vector<Point3f> &pts_1,
	const std::vector<Point3f> &pts_2,
	Mat &R, Mat &t
	);

/// vertex and edges used in g2o ba
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  	virtual void setToOriginImpl() override {
    	_estimate = Sophus::SE3d();
 	}

  	/// left multiplication on SE3
  	virtual void oplusImpl(const double *update) override {
    	Eigen::Matrix<double, 6, 1> update_eigen;
    	update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
    	_estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  	}

  	virtual bool read(istream &in) override {}

  	virtual bool write(ostream &out) const override {}
};

/// g2o edge
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPose> {
public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  	EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

  	virtual void computeError() override {
    	const VertexPose *pose = static_cast<const VertexPose *> ( _vertices[0] );
    	_error = _measurement - pose->estimate() * _point;
  	}

  	virtual void linearizeOplus() override {
    	VertexPose *pose = static_cast<VertexPose *>(_vertices[0]);
    	Sophus::SE3d T = pose->estimate();
    	Eigen::Vector3d xyz_trans = T * _point;
   		_jacobianOplusXi.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
    	_jacobianOplusXi.block<3, 3>(0, 3) = Sophus::SO3d::hat(xyz_trans);
  }

  	bool read(istream &in) {}

  	bool write(ostream &out) const {}

protected:
  	Eigen::Vector3d _point;
};

int main(int argc, char **argv)
{
	if (argc != 5)
	{
		/* code */
		cout<<"usage: pose_estimation_3d3d img_1 img_2 depth_img1 depth_img2"<<endl;
		return 1;
	}
	Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
	Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

	// Check if image is not none
	assert(img_1.data && img_2.data && "Can not load images");
	std::vector<KeyPoint> keypoints_1, keypoints_2;
	std::vector<DMatch> matches;

	find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
	cout<<"Found: "<<matches.size()<<" matches"<<endl;

	// Create 3d Points
	// depth map is 16-bit unsigned number, single channel image
	Mat depth_1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
	Mat depth_2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
	std::vector<Point3f> pts_1, pts_2;

	for (DMatch m:matches)
	{
		/* code */
		ushort d1 = depth_1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
		ushort d2 = depth_2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
		if (d1 == 0 || d2 == 0) // bad depth
		{
			continue;
		}
		Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
		Point2d p2 = pixel2cam(keypoints_2[m.queryIdx].pt, K);

		float dd1 = float(d1)/5000.0;
		float dd2 = float(d2)/5000.0;

		pts_1.push_back(Point3f(p1.x * dd1, p1.y *dd1, dd1));
		pts_2.push_back(Point3f(p2.x * dd2, p2.y *dd2, dd2));
	}

	cout<<"3d-3d pairs: "<< pts_1.size()<<endl;
  	Mat R, t;
  	pose_estimation_3d3d(pts_1, pts_2, R, t);
	
	cout<<"ICP via SVD results: "<<endl;
	cout<<"R = "<<R<<endl;
	cout<<"t = "<<t<<endl;
	cout<<"R inverse = "<<R.t()<<endl;
	cout<<"t inverse = "<<t.t()<<endl;

	cout<<"calling bundle adjustment"<<endl;
	bundleAdjustment(pts_1, pts_2, R, t);

	// verify p1 = R * p2 + t
	for (int i = 0; i < 5; i++) {
		cout << "p1 = " << pts_1[i] << endl;
		cout << "p2 = " << pts_2[i] << endl;
		cout << "(R*p2+t) = " <<
			R * (Mat_<double>(3, 1) << pts_2[i].x, pts_2[i].y, pts_2[i].z) + t
			<< endl;
		cout << endl;
	}
	return 0;
}

void find_feature_matches(
	const Mat &img_1, const Mat &img_2, 
	std::vector<KeyPoint> &keypoints_1,
	std::vector<KeyPoint> &keypoints_2,
	std::vector<DMatch> &matches
	)
{
	Mat descriptors_1, descriptors_2;
	Ptr<FeatureDetector> detector = ORB::create();
	Ptr<DescriptorExtractor> descriptor = ORB::create();

	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

	// Detect Oriented Fast Corner Detection
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);
	
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);

	std::vector<DMatch> match;

	matcher->match(descriptors_1, descriptors_2, match);

	double min_dist = 10000, max_dist = 0;
	// Find the minimum distance and maximum distance between all matches, 
	// that is, the distance between the two most similar and least similar points
	for (int i = 0; i < descriptors_1.rows; ++i)
	{
		/* code */
		double dist = match[i].distance;
    	if (dist < min_dist) min_dist = dist;
    	if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
 	printf("-- Min dist : %f \n", min_dist);

 	//When the distance between the descriptors is greater than twice the minimum distance, the match is considered incorrect. 
 	//But sometimes the minimum distance is very small, and an empirical value of 30 is set as the lower limit.

 	for (int i = 0; i < descriptors_1.rows; ++i)
 	{
 		/* code */
 		if (match[i].distance <= max(2 *min_dist, 30.0))
 		{
 			/* code */
 			matches.push_back(match[i]);
 		}
 	}
}

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
	cv::Point2d point_at_cam;
	point_at_cam.x = (p.x - K.at<double>(0,2)) / K.at<double>(0, 0);
	point_at_cam.y = (p.y - K.at<double>(1,2)) / K.at<double>(1, 1);

	return point_at_cam;
}

void pose_estimation_3d3d(const std::vector<Point3f> &pts_1, 
						  const std::vector<Point3f> &pts_2,
						  Mat &R, Mat &t )
{
	Point3f p1, p2; // center of mass
	int N = pts_1.size();
	for (int i = 0; i < N; ++i)
	{
		p1 += pts_1[i];
		p2 += pts_2[i];
	}

	p1 = Point3f(Vec3f(p1) / N);
	p2 = Point3f(Vec3f(p2) / N);

	// Remove the center
	std::vector<Point3f> q1(N), q2(N);
	for (int i = 0; i < N; ++i)
	{
		q1[i] = pts_1[i] - p1;
		q2[i] = pts_2[i] - p2;
	}

	// Compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	cout<<" W = "<<W<<endl;

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();

	cout << "U=" << U << endl;
	cout << "V=" << V << endl;

	Eigen::Matrix3d R_ = U * (V.transpose());
	if (R_.determinant() < 0) {
		R_ = -R_;
	}
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

	// convert to cv::Mat
	R = (Mat_<double>(3, 3) <<
		R_(0, 0), R_(0, 1), R_(0, 2),
		R_(1, 0), R_(1, 1), R_(1, 2),
		R_(2, 0), R_(2, 1), R_(2, 2)
	);
	t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}

void bundleAdjustment(
  const vector<Point3f> &pts1,
  const vector<Point3f> &pts2,
  Mat &R, Mat &t) 
{
	// Optimize the construction graph, set g2o first
	typedef g2o::BlockSolverX BlockSolverType;
	typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType; // linear solver type
	// Gradient descent method, you can choose from GN, LM, DogLeg
	auto solver = new g2o::OptimizationAlgorithmLevenberg(
		g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
	g2o::SparseOptimizer optimizer;     // graph model
	optimizer.setAlgorithm(solver);   // set the solver
	optimizer.setVerbose(true);       // turn on debug output

	// vertex
	VertexPose *pose = new VertexPose(); // camera pose
	pose->setId(0);
	pose->setEstimate(Sophus::SE3d());
	optimizer.addVertex(pose);

	// edges
	for (size_t i = 0; i < pts1.size(); i++) {
		EdgeProjectXYZRGBDPoseOnly *edge = new EdgeProjectXYZRGBDPoseOnly(
		Eigen::Vector3d(pts2[i].x, pts2[i].y, pts2[i].z));
		edge->setVertex(0, pose);
		edge->setMeasurement(Eigen::Vector3d(
		pts1[i].x, pts1[i].y, pts1[i].z));
		edge->setInformation(Eigen::Matrix3d::Identity());
		optimizer.addEdge(edge);
	}

	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
	optimizer.initializeOptimization();
	optimizer.optimize(10);
	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
	cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

	cout << endl << "after optimization:" << endl;
	cout << "T=\n" << pose->estimate().matrix() << endl;

	// convert to cv::Mat
	Eigen::Matrix3d R_ = pose->estimate().rotationMatrix();
	Eigen::Vector3d t_ = pose->estimate().translation();
	R = (Mat_<double>(3, 3) <<
		R_(0, 0), R_(0, 1), R_(0, 2),
		R_(1, 0), R_(1, 1), R_(1, 2),
		R_(2, 0), R_(2, 1), R_(2, 2)
	);
	t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}
