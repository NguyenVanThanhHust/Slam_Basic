
//This is to calculate derivatives in closed form, instead of automatic differentiation

#include <iostream>
#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using std::cout;
using std::endl;

// Template SizedCostFunction receive 2 parameter kNumResiduals and size of first parameter

class QuadraticCostFunction : public ceres::SizedCostFunction<1, 1>
{
public:
    virtual ~QuadraticCostFunction() {}
    virtual bool Evaluate(double const* const* parameters,
                            double* residuals,
                            double** jacobians) const
    {
        const double x = parameters[0][0];
        residuals[0] = 10 -x;

        // Compute the Jacobian if asked for
        if (jacobians != NULL && jacobians[0] != NULL)
        {
            jacobians[0][0] = -1;
        }
        return true;
    }
};

// The rest is almost the same as numerical derivatives
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    // The variable to solver for with its initial value
    // It will be mutated in place by the solver
    double x = 0.5;
    const double initial_x = x;

    // Build the problem
    Problem problem;

    // Define the cost function
    CostFunction* cost_function = new QuadraticCostFunction;
    problem.AddResidualBlock(cost_function, NULL, &x);

    // Run the solver
    Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    cout << summary.BriefReport() << "\n";
    cout << "x : " << initial_x
            << " -> " << x << "\n";
    return 0;

}