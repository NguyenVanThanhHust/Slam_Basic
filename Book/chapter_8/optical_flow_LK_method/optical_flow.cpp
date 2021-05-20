#include <opencv2/opencv.hpp>
// #include <opencv/xfeatures2d.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

// first and second image
std::string file_1 = "../data/LK1.png";
std::string file_2 = "../data/LK2.png";

// Optical flow tracker and interface
class OpticalFlowTracker{
    public:
        OpticalFlowTracker(
            const cv::Mat &im1_,
            const cv::Mat &im2_,
            const std::vector<cv::KeyPoint> &kp1_,
            std::vector<cv::KeyPoint> &kp2_,
            std::vector<char> success_,
            bool inverse_ = true,
            bool has_initial_ = false
            ):
            im1(im1_), im2(im2_), kp1(kp1_), kp2(kp2_), success(success_), 
            inverse(inverse_), has_initial(has_initial_){}
        void calculateOpticalFlow(const cv::Range &range);
    private:
    const cv::Mat &im1;
    const cv::Mat &im2;
    const std::vector<cv::KeyPoint> &kp1;
    std::vector<cv::KeyPoint> &kp2;
    std::vector<char> success;
    bool inverse = true;
    bool has_initial = false;
};
/**
 * single level optical flow
 * @param [in] im1 the first image
 * @param [in] im2 the second image
 * @param [in] kp1 keypoints in im1
 * @param [in|out] kp2 keypoints in im2, if empty, use initial guess in kp1
 * @param [out] success true if a KeyPoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */

void OpticalFlowSingleLevel(
    const cv::Mat &im1,
    const cv::Mat &im2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<char> &success,
    bool inverse = false,
    bool has_initial_guess = false
);

/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] im1 the first pyramid
 * @param [in] im2 the second pyramid
 * @param [in] kp1 keypoints in im1
 * @param [out] kp2 keypoints in im2
 * @param [out] success true if a KeyPoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
    const cv::Mat &im1,
    const cv::Mat &im2,
    const vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<char> &success,
    bool inverse = false
);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return the interpolated value of this pixel
 */

inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]
    );
}

int main(int argc, char **argv)
{
    cv::Mat im1 = cv::imread(file_1, 0); // Read single channel
    cv::Mat im2 = cv::imread(file_2, 0); // Read single channel

    // Detect KeyPoint
    std::vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(500);

    detector->detect(im1, kp1);

    // Track these KeyPoint in the second image
    // first, use single level LK in the validation picture
    std::vector<cv::KeyPoint> kp2_single;
    std::vector<char> success_single;

    OpticalFlowSingleLevel(im1, im2, kp1, kp2_single, success_single);

    // Test with multi level
    std::vector<cv::KeyPoint> kp2_multi;
    std::vector<char> success_multi;

    // measure time
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    OpticalFlowMultiLevel(im1, im2, kp1, kp2_multi, success_multi, true);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by gauss-newton: " << time_used.count() << endl;


    // Validate with opencv's flow
    std::vector<cv::Point2f> pt1, pt2;
    for(auto &kp: kp1) pt1.push_back(kp.pt);
    std::vector<uchar> status;
    std::vector<float> error;
    t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(im1, im2, pt1, pt2, status, error);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv: " << time_used.count() << endl;

    // Plot the differences of those functions
    cv::Mat im2_single;
    cv::cvtColor(im2, im2_single, cv::COLOR_GRAY2BGR);
    for(int i=0; i < kp2_single.size(); i++)
    {
        if(success_single[i])
        {
            cv::circle(im2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(im2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
        } 
    }

    cv::Mat im2_multi;
    cv::cvtColor(im2, im2_multi, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            cv::circle(im2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(im2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }

    cv::Mat im2_CV;
    cv::cvtColor(im2, im2_CV, cv::COLOR_GRAY2BGR);
    for(int i=0; i < pt2.size(); i++)
    {
        if (status[i]) {
            cv::circle(im2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(im2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    cv::imwrite("track_single.jpg", im2_single);
    cv::imwrite("track_multi.jpg", im2_multi);
    cv::imwrite("track_opencv.jpg", im2_CV);
    return 0;
}

void OpticalFlowSingleLevel(
    const cv::Mat &im1,
    const cv::Mat &im2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<char> &success,
    bool inverse, bool has_initial) {
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(im1, im2, kp1, kp2, success, inverse, has_initial);
    parallel_for_(cv::Range(0, kp1.size()),
                  std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, placeholders::_1));
}

void OpticalFlowTracker::calculateOpticalFlow(const cv::Range &range) {
    // parameters
    int half_patch_size = 4;
    int iterations = 10;
    for (size_t i = range.start; i < range.end; i++) {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (has_initial) {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();    // hessian
        Eigen::Vector2d b = Eigen::Vector2d::Zero();    // bias
        Eigen::Vector2d J;  // jacobian
        for (int iter = 0; iter < iterations; iter++) {
            if (inverse == false) {
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            } else {
                // only reset b
                b = Eigen::Vector2d::Zero();
            }

            cost = 0;

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)
                for (int y = -half_patch_size; y < half_patch_size; y++) {
                    double error = GetPixelValue(im1, kp.pt.x + x, kp.pt.y + y) -
                                   GetPixelValue(im2, kp.pt.x + x + dx, kp.pt.y + y + dy);;  // Jacobian
                    if (inverse == false) {
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(im2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                   GetPixelValue(im2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)),
                            0.5 * (GetPixelValue(im2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                                   GetPixelValue(im2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))
                        );
                    } else if (iter == 0) {
                        // in inverse mode, J keeps same for all iterations
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J = -1.0 * Eigen::Vector2d(
                            0.5 * (GetPixelValue(im1, kp.pt.x + x + 1, kp.pt.y + y) -
                                   GetPixelValue(im1, kp.pt.x + x - 1, kp.pt.y + y)),
                            0.5 * (GetPixelValue(im1, kp.pt.x + x, kp.pt.y + y + 1) -
                                   GetPixelValue(im1, kp.pt.x + x, kp.pt.y + y - 1))
                        );
                    }
                    // compute H, b and set cost;
                    b += -error * J;
                    cost += error * error;
                    if (inverse == false || iter == 0) {
                        // also update H
                        H += J * J.transpose();
                    }
                }

            // compute update
            Eigen::Vector2d update = H.ldlt().solve(b);

            if (std::isnan(update[0])) {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }

            if (iter > 0 && cost > lastCost) {
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;

            if (update.norm() < 1e-2) {
                // converge
                break;
            }
        }

        success[i] = succ;

        // set kp2
        kp2[i].pt = kp.pt + cv::Point2f(dx, dy);
    }
}

void OpticalFlowMultiLevel(
    const cv::Mat &im1,
    const cv::Mat &im2,
    const std::vector<cv::KeyPoint> &kp1,
    std::vector<cv::KeyPoint> &kp2,
    std::vector<char> &success,
    bool inverse) {

    // parameters
    int pyramids = 4;
    double pyramid_scale = 0.5;
    double scales[] = {1.0, 0.5, 0.25, 0.125};

    // create pyramids
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    std::vector<cv::Mat> pyr1, pyr2; // image pyramids
    for (int i = 0; i < pyramids; i++) {
        if (i == 0) {
            pyr1.push_back(im1);
            pyr2.push_back(im2);
        } else {
            cv::Mat im1_pyr, im2_pyr;
            cv::resize(pyr1[i - 1], im1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));
            cv::resize(pyr2[i - 1], im2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(im1_pyr);
            pyr2.push_back(im2_pyr);
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "build pyramid time: " << time_used.count() << endl;

    // coarse-to-fine LK tracking in pyramids
    std::vector<cv::KeyPoint> kp1_pyr, kp2_pyr;
    for (auto &kp:kp1) {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--) {
        // from coarse to fine
        success.clear();
        t1 = chrono::steady_clock::now();
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true);
        t2 = chrono::steady_clock::now();
        auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "track pyr " << level << " cost time: " << time_used.count() << endl;

        if (level > 0) {
            for (auto &kp: kp1_pyr)
                kp.pt /= pyramid_scale;
            for (auto &kp: kp2_pyr)
                kp.pt /= pyramid_scale;
        }
    }

    for (auto &kp: kp2_pyr)
        kp2.push_back(kp);
}

