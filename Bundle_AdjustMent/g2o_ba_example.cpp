#include <iostream>>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <boost/concept_check.hpp>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;

// Find the corresponding points in images, pixel coordinate system
int findCorrespondingPoints( const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 );
// Camera internal parameters
double cx = 325.5;
double cy = 253.5;
double fx = 518.0;
double fy = 519.0;
const int MAX_FEATURES = 500;
const float GOOD_MATCH_PERCENT = 0.8f;

int main(int argc, char** argv)
{
	if(argc != 3)
	{
		cout <<" Usage ba_example img1 img2" <<endl;
		exit(1);
	}

	cv::Mat img1 = cv::imread(argv[1]);
	cv::Mat img2 = cv::imread(argv[2]);

	vector<cv::Point2f> pts1, pts2;
	if(findCorrespondingPoints(img1, img2, pts1, pts1) == false)
	{
		cout<<"Can't find matched keypoints" <<endl;
		return 0;
	}

	cout<<"Found " << pts1.size() << " corresponding pair of points"<<endl;
	// Construct the graph in g2o
	g2o::SparseOptimizer optimizer;

	// Construct the solver
	g2o::BlockSolver_6_3::LinearSolverType* linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
	
	g2o::BlockSolver_6_3* block_soler = new g2o::BlockSolver_6_3(linearSolver);

	g2o::OptimizationAlgorithmLevenberg* algortihm = new g2o::OptimizationAlgorithmLevenberg(block_soler);

	optimizer.setAlgorithm(algortihm);
	optimizer.setVerbose(false);

	// add node, we have 2 poses node
	for(int i=0; i<2; i++)
	{
		g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
		v->setId(i);
		if(i==0)
		{
			// The first point is fixed to zero
			v->setFixed(true);
		}
		// The default value is the unit Pose, because we don’t know any information
		v->setEstimate( g2o::SE3Quat() );
        optimizer.addVertex( v );
	}
	// Nodes with many feature points
	// Subject to the first frame
	for(size_t i=0; i<pts1.size(); i++)
	{
		g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
		v->setId(i+i);
		// Depth is unknow so is set to 1
		double z=1;
		double x = (pts1[i].x - cx)*z /fx;
		double y = (pts1[i].y - cy)*z /fy;
		v->setMarginalized(true);
		v->setEstimate(Eigen::Vector3d(x, y, z));
		optimizer.addVertex(v);
	}

	// Set camera parameter
	g2o::CameraParameters* camera = new g2o::CameraParameters(fx, Eigen::Vector2d(cx, cy), 0);
    camera->setId(0);
	optimizer.addParameter(camera);

	// prepare side
	// first frame
	std::vector<g2o::EdgeProjectXYZ2UV*> edges;
	for(size_t i=0; i<pts1.size(); i++)
	{
		g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
		edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2)));
		edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
		edge->setMeasurement(Eigen::Vector2d(pts1[i].x, pts1[i].y));
		edge->setParameterId(0, 0);
		// define kernel
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		optimizer.addEdge(edge);
		edges.push_back(edge);
	}

	for(size_t i=0; i<pts2.size(); i++)
	{
		g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
		edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(i+2)));
		edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*> (optimizer.vertex(0)));
		edge->setMeasurement(Eigen::Vector2d(pts2[i].x, pts2[i].y));
		edge->setParameterId(0, 0);
		// define kernel
		edge->setRobustKernel(new g2o::RobustKernelHuber());
		optimizer.addEdge(edge);
		edges.push_back(edge);
	}

	cout<<"Start optimization"<<endl;
	optimizer.setVerbose(true);
	optimizer.initializeOptimization();
	optimizer.optimize(itertions=10);
	cout<<"End optimzation"<<endl;

    // Transformation between two frame
    g2o::VertextSE3Expmap* v=  dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertext(1));
    Eigen::Isometry3d pose = v->estimate();
    cout<<"Pose = "<<endl<<pose.matrix()<<endl;

    // Location of feature points
    for(size_t i=0; i<pts1.size(); i++)
    {
        g2o::VertextSBAPointsXYZ* v = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertext(i+2));
        cout<<"vertex id: "<<i+2<<", pose = ";
        Eigen::Vector3d pos = v->estimate();
        cout<<pos(0)<<","<<pos(1)<<","<<pos(2)<<endl;
    }

    // Estimate number of inliners
    int inliers = 0;
    for ( auto e:edges )
    {
        e->computeError();
        // chi2 就是 error*\Omega*error, 如果这个数很大，说明此边的值与其他边很不相符
        if ( e->chi2() > 1 )
        {
            cout<<"error = "<<e->chi2()<<endl;
        }
        else 
        {
            inliers++;
        }
    }
    
    cout<<"inliers in total points: "<<inliers<<"/"<<pts1.size()+pts2.size()<<endl;
    optimizer.save("ba.g2o");
    return 0;
}

int findCorrespondingPoints(const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 )
{
    // copy from https://github.com/NguyenVanThanhHust/OpenCV-Pratice/blob/master/Learning_OpenCV_with_cpp/Image%20Aligment%20Features%20Based.cpp
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create(MAX_FEATURES);
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Mat mask(img1.size(), img1.type(), cv::Scalar::all(0));;
    cv::Mat descs_1, descs_2;
	std::vector<cv::KeyPoint> keypoint_1, keypoint_2; 


    // detector->detectAndCompute(img1,  Mat(), points1, descs_1);
    detector->detectAndCompute(img1, mask, keypoint_1, descs_1);
    detector->detectAndCompute(img1, mask, keypoint_2, descs_1);

    // Match features
	std::vector<cv::DMatch> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
	matcher->match(descs_1, descs_2, matches, cv::Mat());

    // Sort matches by score
	std::sort(matches.begin(), matches.end());

	// Remove not so good matches
	const int numGoodMatches = matches.size() * GOOD_MATCH_PERCENT;
	matches.erase(matches.begin() + numGoodMatches, matches.end());

	if(matches.size() <= 20)
	{
		return false;
	}
	// Extract location of good matches
	std::vector<cv::Point2f> points1, points2;
	for (size_t i = 0; i < matches.size(); i++)
	{
		points1.push_back(keypoint_1[matches[i].queryIdx].pt);
		points2.push_back(keypoint_2[matches[i].trainIdx].pt);
	}
	return true;
}
