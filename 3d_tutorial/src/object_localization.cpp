#include "opencv2/opencv.hpp"

#define DEG2RAD(v)  (v * CV_PI / 180)

// Define rotation matrix, check slide page 26
#define Rx(rx)      (cv::Matx33d(1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx)))
#define Ry(ry)      (cv::Matx33d(cos(ry), 0, sin(ry), 0, 1, 0, -sin(rx), 0, cos(ry)))
#define Rz(rz)      (cv::Matx33d(cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1))

class MouseDrag
{
public:
    MouseDrag() : dragged(false) { }
    bool dragged;
    cv::Point start, end;
};

void MouseEventHandler(int event, int x, int y, int flags, void* param)
{
    if (param == NULL) return;
    MouseDrag* drag = (MouseDrag*)param;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        drag->dragged = true;
        drag->start = cv::Point(x, y);
        drag->end = cv::Point(0, 0);
    }
    else if (event == cv::EVENT_MOUSEMOVE)
    {
        if (drag->dragged) drag->end = cv::Point(x, y);
    }
    else if (event == cv::EVENT_LBUTTONUP)
    {
        if (drag->dragged)
        {
            drag->dragged = false;
            drag->end = cv::Point(x, y);
        }
    }
}

int main()
{
    const char* input = "data/daejeon_station.png";
    double f = 810.5, cx = 480, cy = 270, L = 3.31;
    cv::Point3d cam_ori(DEG2RAD(-18.7), DEG2RAD(-8.2), DEG2RAD(2.0));
    cv::Range grid_x(-2, 3), grid_z(5, 35);

    // Load an images
    cv::Mat image = cv::imread(input);
    if (image.empty()) return -1;

    // Configure mouse callback
    MouseDrag drag;
    cv::namedWindow("3DV Tutorial: Object Localization and Measurement");
    cv::setMouseCallback("3DV Tutorial: Object Localization and Measurement", MouseEventHandler, &drag);
    
}