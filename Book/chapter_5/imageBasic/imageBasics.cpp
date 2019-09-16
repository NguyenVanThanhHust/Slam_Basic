#include <iostream>
#include <chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
int main(int argc, char **argv)
{
    cv::Mat image;
    image = cv::imread(argv[1]);
    if( !image.data)
    {
        cerr<<"This image doesn't exist."<<endl;
        return 0;
    }
    cout<<"number of column: "<<image.cols<<", number of rows: "<<", number of channels"<<image.channels()<<endl;
    cv::waitKey(0);
    
    // Determine type of image
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        cout<<"This is not grayscale or color image"<<endl;
    }
    
    // Traversing the image
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y <image.rows; y++)
    {
        // Get the line pointer with cv::Mat::ptr
        // row_ptr is the head pointer of the yth line
        unsigned char *row_ptr = image.ptr<unsigned char>(y);
        for(size_t x = 0; x <image.cols; x++)
        {
            // access value of pixel(x,y) 
            // data_ptr point to data to be accessed
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            // output each channel of the pixel, if there is only one channel in grayscale
            for(int c = 0; c != image.channels(); c++)
            {
                unsigned char data = data_ptr[c];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast < chrono::duration < double >> (t2 - t1);
    cout<<"Time to traverse all data points in image: "<<time_used.count() << " ms" <<endl;
    cv::destroyAllWindows();
    return 0;
}
