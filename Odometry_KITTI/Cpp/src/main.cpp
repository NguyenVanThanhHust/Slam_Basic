#include "headers.h"
#include "Constants.h"


int main()
{
    int frame_height = 600;
    int frame_width  = 600;
    cv::VideoWriter video("outcpp_1.avi", 
                            cv::VideoWriter::fourcc('M','J','P','G'), 
                            10, cv::Size(frame_width,frame_height));

    std::string left_im_folder = "../../../datasets/KITTI/dataset/sequences/00/image_0/";
    std::vector<std::string> left_imagepaths = get_file_list(left_im_folder);

    std::string right_im_folder = "../../../datasets/KITTI/dataset/sequences/00/image_1/";
    std::vector<std::string> right_imagepaths = get_file_list(right_im_folder);

    cv::Mat R_final, T_final; 
    cv::Mat R, T, mask;

	cv::Ptr<cv::Feature2D> orb = cv::ORB::create(MAX_FEATURES);
    
    // Below number from https://www.youtube.com/playlist?list=PLrHDCRerOaI9HfgZDbiEncG5dx7S3Nz6X 
    // First video
    float focal = 718.8560;
    cv::Point2d pp(607.1928, 185.2157);
    float abso_scale = 1.0;

    cv::Mat img_1_c = cv::imread(left_imagepaths[0]);
    cv::Mat img_2_c = cv::imread(left_imagepaths[1]);

    cv::Mat img_1, img_2;
    cv::cvtColor(img_1_c, img_1, cv::COLOR_BGR2GRAY);
    cv::cvtColor(img_2_c, img_2, cv::COLOR_BGR2GRAY);

    std::vector<cv::Point2f> points_1, points_2;
    std::vector<uchar> status;

    featureDetection(orb, img_1, points_1);
    featureTracking(img_1, img_2, points_1, points_2, status);

    cv::Mat essentialMat = cv::findEssentialMat(
                    points_2, points_1,
                    focal, pp, cv::RANSAC, 0.999,
                    1.0, mask);

    cv::recoverPose(essentialMat, points_2, 
                    points_1, R, 
                    T, focal,
                    pp, mask);

    R_final = R.clone();
    T_final = T.clone();

    std::vector<cv::Point2f> prev_points = points_2;
    std::vector<cv::Point2f> curr_points;
    
    cv::Mat prev_frame, curr_frame;
    prev_frame = img_2.clone();

    // mat to draw trajectory
    cv::Mat traj = cv::Mat::zeros(600, 600, CV_8UC3);
    for(int i=0; i < left_imagepaths.size(); i++)
    {   
        if(i<2) continue;

        curr_frame = cv::imread(left_imagepaths[i], cv::IMREAD_GRAYSCALE);
        std::vector<uchar> status;

        featureTracking(prev_frame, curr_frame, prev_points, curr_points, status);
        essentialMat = cv::findEssentialMat(
            curr_points, prev_points,
            focal, pp, cv::RANSAC, 0.999,
            1.0, mask );
        cv::recoverPose(
            essentialMat, prev_points, 
            curr_points, R, 
            T, focal,
            pp, mask 
        );

        cv::Mat prev_Pts(2,prev_points.size(), CV_64F), curr_Pts(2,curr_points.size(), CV_64F);

        abso_scale = getAbsoluteScale(i, T.at<float>(2));
        for(int i=0;i<prev_points.size();i++)	{   //this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
            prev_Pts.at<double>(0,i) = prev_points.at(i).x;
            prev_Pts.at<double>(1,i) = prev_points.at(i).y;

            prev_Pts.at<double>(0,i) = curr_points.at(i).x;
            prev_Pts.at<double>(1,i) = curr_points.at(i).y;
        }

        abso_scale = getAbsoluteScale(i, T.at<float>(2));
        if ((abso_scale>0.1)&&(T.at<double>(2) > T.at<double>(0)) && (T.at<double>(2) > T.at<double>(1)))
        {
            T_final = T_final + abso_scale*(R_final*T);
            R_final = R*R_final;
        }

        prev_frame = curr_frame.clone();
        prev_points = curr_points;

        if(prev_points.size() < MIN_NUM_FEAT)
        {
            featureDetection(orb, prev_frame, prev_points);
            featureTracking(prev_frame, curr_frame, prev_points, curr_points, status);
        }

        int x = int(T_final.at<double>(0)) + 300;
        int y = int(T_final.at<double>(2)) + 100;
        cv::circle(traj, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);
        video<<traj;
        // cv::imshow( "Trajectory", traj );
        // cv::waitKey(1);
    }
    return 0;
}
