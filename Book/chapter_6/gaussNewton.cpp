#include <iostream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

/*
in this programe, we will try to use gauss newton method to estimate coefficient of a 2 degree equation
*/
int main()
{
    // True parameter
    double real_a=1.0, real_b=2.0, real_c=1.0;
    // estimate parameter
    double esti_a = 2.0, esti_b = -1.0, esti_c = 1.0;
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

    // Start Gauss-Newton iteration
    int iterations = 100;
    // The cost of this iteration and the cost of the last iteration
    double cost = 0, last_cost = 0;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (int iter = 0; iter < iter; ++iter)
    {
    	/* code */
    	Eigen::Matrix3d HessianMatrix = Eigen::Matrix3d::Zero();
    	Eigen::Vector3d bias = Eigen::Vector3d::Zero();
    	cost = 0;
    	for (int i = 0; i < N; ++i)
    	{
    		/* code */
    		double x_i = x_data[i], y_i = y_data[i];
    		double error = y_i - exp(esti_a * x_i * x_i + esti_b * x_i + esti_c);
    		// Jacobian matrix
    		Eigen::Vector3d Jacobian;
    		Jacobian[0] = -x_i*x_i * exp(esti_a * x_i * x_i + esti_b * x_i + esti_c);
    		Jacobian[1] = -x_i * exp(esti_a * x_i * x_i + esti_b * x_i + esti_c);
    		Jacobian[2] = -exp(esti_a * x_i * x_i + esti_b * x_i + esti_c);

    		HessianMatrix += inverse_sigma * inverse_sigma * Jacobian * Jacobian.transpose();
    		bias += - inverse_sigma * inverse_sigma * error * Jacobian;

    		cost += error * error;

    	}
    	// Solve the linear equation Hx = b
    	Eigen::Vector3d dx = HessianMatrix.ldlt().solve(bias);
    	if (isnan(dx[0]))
    	{
    		/* code */
    		cout<<"Result isn't a number"<<endl;
    		break;
    	}

    	if (iter > 0 && cost >= last_cost) 
    	{
    		/* code */
    		cout << "cost: " << cost << ">= last cost: " << last_cost << ", break." << endl;
      		break;
    	}
    	esti_a += dx[0];
    	esti_b += dx[1];
    	esti_c += dx[2];

    	last_cost = cost;

    	cout << "total cost: " << cost << ", \t\tupdate: " << dx.transpose() <<
         "\t\testimated params: " << esti_a << "," << esti_b << "," << esti_c << endl;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
  	chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
  	cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

  	cout << "estimated abc = " << esti_a << ", " << esti_b << ", " << esti_c << endl;
    return 0;
}




















