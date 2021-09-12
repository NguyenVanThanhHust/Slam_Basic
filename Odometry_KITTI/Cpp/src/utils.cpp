#include "headers.h"


std::vector<std::string> get_file_list(std::string path)
{
    std::vector<std::string> m_file_list; 
    if (!path.empty())
    {
        namespace fs = boost::filesystem;

        fs::path apk_path(path);
        fs::recursive_directory_iterator end;

        for (fs::recursive_directory_iterator i(apk_path); i != end; ++i)
        {
            const fs::path cp = (*i);
            m_file_list.push_back(cp.string());
        }
    }
    return m_file_list;
}

float getAbsoluteScale(int frame_id, float z_cal)
{
    std::string pose_file = "../../../datasets/KITTI/dataset/poses/00.txt";
    std::string line;
    std::ifstream poses(pose_file);
    int i = 0;
    float prev_x, prev_y, prev_z;
    float x, y, z;
    while(getline(poses, line) && (i<=frame_id)){
        prev_x = x; 
        prev_y = y;
        prev_z = z;
        std::istringstream in(line);
        for (int j=0; j<12; j++){
            in >> z ;
            if (j==7) y=z;
            if (j==3) x=z;
        }
        i++;
    }
    return sqrt(pow((x-prev_x), 2.0) + 
                pow((y-prev_y), 2.0) +
                pow((z-prev_z), 2.0)
    );
}

void featureDetection(cv::Ptr<cv::Feature2D>& detector, cv::Mat& img, std::vector<cv::Point2f>& points){
    std::vector<cv::KeyPoint> tmp_keypoints;
    detector->detect(img, tmp_keypoints);
    cv::KeyPoint::convert(tmp_keypoints, points);
}

void featureTracking(cv::Mat prev_frame, cv::Mat curr_frame, std::vector<cv::Point2f>& prev_points, std::vector<cv::Point2f>& curr_points, std::vector<uchar>& status)	
{ 
//this function automatically gets rid of points for which tracking fails

    std::vector<float> err;					
    cv::Size winSize=cv::Size(21,21);																								
    cv::TermCriteria termcrit=cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    calcOpticalFlowPyrLK(prev_frame, curr_frame, prev_points, curr_points, status, err, winSize, 3, termcrit, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int indexCorrection = 0;
    for( int i=0; i<status.size(); i++)
        {  
            cv::Point2f pt = curr_points.at(i- indexCorrection);
            if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
                if((pt.x<0)||(pt.y<0))	{
                    status.at(i) = 0;
                }
                prev_points.erase (prev_points.begin() + (i - indexCorrection));
                curr_points.erase (curr_points.begin() + (i - indexCorrection));
                indexCorrection++;
            }

        }

}