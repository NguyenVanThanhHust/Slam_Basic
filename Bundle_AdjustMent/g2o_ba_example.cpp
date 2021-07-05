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

int main()
{
	
    return 0;
}

int findCorrespondingPoints(const cv::Mat& img1, const cv::Mat& img2, vector<cv::Point2f>& points1, vector<cv::Point2f>& points2 )
{
    // copy from https://github.com/NguyenVanThanhHust/OpenCV-Pratice/blob/master/Learning_OpenCV_with_cpp/Image%20Aligment%20Features%20Based.cpp
    Ptr<FeatureDetector> detector = ORB::create(MAX_FEATURES);
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    cv::Mat mask(img1.size(), img1.type(), Scalar::all(0));;
    cv::Mat descs_1, descs_2:

    // detector->detectAndCompute(img1,  Mat(), points1, descs_1);
    detector->detectAndCompute(img1, mask, points1, descs_1);
    detector->detectAndCompute(img1, mask, points1, descs_1);

    // Match features
	std::vector<DMatch> matches;
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	matcher->match(descriptor1, descriptor2, matches, Mat());

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
	vector<Point2f> points1, points2;
	for (size_t i = 0; i < matches.size(); i++)
	{
		points1.push_back(keypoint1[matches[i].queryIdx].pt);
		points2.push_back(keypoint2[matches[i].trainIdx].pt);
	}
	return true;
}
