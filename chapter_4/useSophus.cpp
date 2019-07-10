#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;


// This program demonstrates the basic usage of sophus
int main() {

  // Rotation matrix rotated 90 degrees along the Z axis
  Matrix3d R = AngleAxisd(M_PI / 2, Vector3d(0, 0, 1)).toRotationMatrix();
  // or quaternion
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);              // Sophus::SO3d can be constructed directly from the rotation matrix
  Sophus::SO3d SO3_q(q);              // can also be constructed by quaternion
  // Both are equivalent
  cout << "SO(3) from matrix:\n" << SO3_R.matrix() << endl;
  cout << "SO(3) from quaternion:\n" << SO3_q.matrix() << endl;
  cout << "they are equal" << endl;

  // Use the logarithmic map to get its Lie algebra
  Vector3d so3 = SO3_R.log();
  cout << "so3 = " << so3.transpose() << endl;
  // hat is vector to antisymmetric matrix
  cout << "so3 hat=\n" << Sophus::SO3d::hat(so3) << endl;
  // Relative, vee is the objection vector
  cout << "so3 hat vee= " << Sophus::SO3d::vee(Sophus::SO3d::hat(so3)).transpose() << endl;

  // Update of the incremental disturbance model
  Vector3d update_so3(1e-4, 0, 0); 
  Sophus::SO3d SO3_updated = Sophus::SO3d::exp(update_so3) * SO3_R;
  cout << "SO3 updated = \n" << SO3_updated.matrix() << endl;
    // Suppose the update is so much
  cout << "*******************************" << endl;
  // The same is true for SE(3) operations
  Vector3d t(1, 0, 0);           // pan 1 along the x axis
  Sophus::SE3d SE3_Rt(R, t);           // Construct SE from R, t(3)
  Sophus::SE3d SE3_qt(q, t);            // Construct SE(3) from q, t
  cout << "SE3 from R,t= \n" << SE3_Rt.matrix() << endl;
  cout << "SE3 from q,t= \n" << SE3_qt.matrix() << endl;
  // The Lie algebra se(3) is a six-dimensional vector. For convenience, typedef is used first.
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d se3 = SE3_Rt.log();
  cout << "se3 = " << se3.transpose() << endl;
  
  cout << "se3 hat = \n" << Sophus::SE3d::hat(se3) << endl;
  cout << "se3 hat vee = " << Sophus::SE3d::vee(Sophus::SE3d::hat(se3)).transpose() << endl;
    // Observe the output, you will find that in Sophus, the translation of se(3) is in front and the rotation is in the back.
   // Same, there are two operators of hat and vee
  // Finally, demonstrate the update
  Vector6d update_se3; 
  update_se3.setZero();
  update_se3(0, 0) = 1e-4d;
  Sophus::SE3d SE3_updated = Sophus::SE3d::exp(update_se3) * SE3_Rt; // update volume
  cout << "SE3 updated = " << endl << SE3_updated.matrix() << endl;

  return 0;
}
