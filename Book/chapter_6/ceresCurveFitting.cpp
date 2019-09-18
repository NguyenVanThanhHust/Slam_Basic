#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// The calculation for curve fitting model
struct CURVE_FITTING_COST
{
	CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

	// Calculation of residuals
	template<typename T>
	bool operator() (const T *const abc, T *residual) // Model parameter: 3 dimensions
	const {
		residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
		return true;
	}
	// data 
	const double _x, _y;
};

int main()
{
	// True parameter
    double real_a=1.0, real_b=2.0, real_c=1.0;
    // estimate parameter
    double esti_a = 2.0, esti_b = -1.0, esti_c = 5.0;
    // number data points
    int N = 100;
    // Noise Sigma value
    double w_sigma = 1.0;
    double inverse_sigma = 1.0/w_sigma;

    // OpenCV random number generator
	cv::RNG rng;
    std::vector<double> x_data, y_data;
    for (int i = 0; i < N; ++i)
    {
    	/* code */
    	double x = i /100.0;
    	x_data.push_back(x);
    	y_data.push_back(exp(real_a * x * x + real_b * x + real_c) + rng.gaussian(w_sigma * w_sigma));
    }

    double abc[3] = {esti_a, esti_b, esti_c};

    // Build the least square problem
    ceres::Problem problem;
    for (int i = 0; i < N; ++i)
    {
    	// Add error term to the equation
    	// Use automatic derivation, template parameter: error type, output dimension, input dimension
    	problem.AddResidualBlock(
    		new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> 
    		(new CURVE_FITTING_COST(x_data[i], y_data[i])
    		), 
    		nullptr, // kernel function, not used here
    		abc // parameters to be estimated
		);
    }

    // Configure the solver
    ceres::Solver::Options options;
    // How to solve the incremental equation
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    // output to cout
    options.minimizer_progress_to_stdout = true;

    // Optimization
    ceres::Solver::Summary summary;
  	chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
  	ceres::Solve(options, &problem, &summary);
  	chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  	cout << "solve time cost = " << time_used.count() << " seconds. " << endl;
  	
  	cout << summary.BriefReport() << endl;
  	cout << "estimated a,b,c = ";
  	for (auto a:abc) cout << a << " ";
  	cout << endl;

 	 return 0;
}