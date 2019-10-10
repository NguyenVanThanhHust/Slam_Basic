#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

// This is how can we estimate camera position with 2d-2d feature matching.
void find_feature_matches(
	const Mat &img_1, const Mat &img_2, 
	std::vector<KeyPoint> &keypoints_1,
	std::vector<KeyPoint> &keypoints_2,
	std::vector<DMatch> &matches
	);

void pose_estimation_2d2d(
	std::vector<KeyPoint> keypoints_1,
	std::vector<KeyPoint> keypoints_2,
	std::vector<DMatch> matches,
	Mat &R, Mat &t
	);

// Pixel coordiante to camera normalized ordinate
Point2d pixel2cam(const Point2d &p, const Mat &K);

int main(int argc, char **argv)
{
	if (argc != 3)
	{
		/* code */
		cout<<"usage: pose_estimation_2d2d img_1 img_2"<<endl;
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

	// Estimate motion from image
	Mat R, t;
	pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

	// Verify the E = t*R*scale
	Mat t_x =
    (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
      t.at<double>(2, 0), 0, -t.at<double>(0, 0),
      -t.at<double>(1, 0), t.at<double>(0, 0), 0);

  	cout << "t^R=" << endl << t_x * R << endl;

  	// Verify the polarity constraint
  	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  	for (DMatch m: matches) 
  	{
   	 	Point2d pt1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
   		Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    	Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
    	Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    	Mat d = y2.t() * t_x * R * y1;
    	cout << "epipolar constraint = " << d << endl;
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

void pose_estimation_2d2d(
	std::vector<KeyPoint> keypoints_1,
	std::vector<KeyPoint> keypoints_2,
	std::vector<DMatch> matches,
	Mat &R, Mat &t
	)
{
	// Camera internal reference, TUM Freiburg2
	Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);

	//-- Convert matching points to the form of vector<Point2f>
	std::vector<Point2f> points1;
	std::vector<Point2f> points2;
	for (int i = 0; i < matches.size(); ++i)
	{
		points1.push_back(keypoints_1[matches[i].queryIdx].pt);
		points2.push_back(keypoints_2[matches[i].queryIdx].pt);
	}

	//-- Calculate the base matrix
	Mat fundamental_matrix;
	fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
    cout << "fundamental_matrix is " << endl << fundamental_matrix << endl;

    // Calculate essence matrix
    Point2d principal_point(325.1, 249.7);
    double focal_length = 521;
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
  	cout << "essential_matrix is " << endl << essential_matrix << endl;

  	//-- Calculate the homography matrix

	//-- But in this case, the scene is not a plane, and the matrix should not be meaningful.
	Mat homography_matrix;
	homography_matrix = findHomography(points1, points2, RANSAC, 3);
	cout<<"homography matrix is: "<<endl<<homography_matrix<<endl;
	
	// Restore rotation and translation matrix from essence matrix
	recoverPose(essential_matrix, points1, points2, R, t, focal_length, principal_point);
	cout<<"Rotation matrix: "<<endl<<R<<endl;
	cout<<"translation matrix: "<<endl<<t<<endl;
}

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
	cv::Point2d point_at_cam;
	point_at_cam.x = (p.x - K.at<double>(0,2)) / K.at<double>(0, 0);
	point_at_cam.y = (p.y - K.at<double>(1,2)) / K.at<double>(1, 1);

	return point_at_cam;
}
