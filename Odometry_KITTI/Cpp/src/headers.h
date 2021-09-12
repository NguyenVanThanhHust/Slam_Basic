#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "boost/filesystem.hpp"

#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <math.h>

using std::cout;
using std::endl;

std::vector<std::string> get_file_list(std::string path);
float getAbsoluteScale(int frame_id, float z_cal);
void featureDetection(cv::Ptr<cv::Feature2D>& detector, cv::Mat& img, std::vector<cv::Point2f>& points);
void featureTracking(cv::Mat prev_frame, cv::Mat curr_frame, std::vector<cv::Point2f>& prev_points, std::vector<cv::Point2f>& curr_points, std::vector<uchar>& status);