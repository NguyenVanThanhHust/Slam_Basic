#include <iostream>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;
using std::cout;
using std::cin;

struct NumericDiffCostFunctor
{
    bool operator()(const double* const x, double *residual) const {
        residual[0] = 10.0 - x[0];
        return true;
    };
};

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
    CostFunction* cost_function = new NumericDiffCostFunction<NumericDiffCostFunctor, ceres::CENTRAL, 1, 1>(
        new NumericDiffCostFunctor);
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