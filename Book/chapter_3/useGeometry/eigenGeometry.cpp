#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
using namespace std;
using namespace Eigen;

// This to demonstrate how to use Eigen Geometry module
int main()
{
	// We use this module for rotation and translation transformation
	Matrix3d rotation_matrix = Matrix3d::Identity();
	
	// The rotation vector uses AngleAxis, the underlying layer is not directly Matrix,
	// But the operation can be treated as a matrix(being overloaded)
	AngleAxisd rotation_vector(M_PI /4, Vector3d(0,0,1)); // Rotate a around z axis 45 degree
	
	cout.precision(3);
	cout<<"Rotation matrix  = \n"<<rotation_vector.matrix()<<endl; // convert to matrix
	
	// It is can be assigned directly
	rotation_matrix = rotation_vector.toRotationMatrix();
	
	//Coordinate transformation with AngleAxis
	Vector3d v(1,0,0);
	Vector3d v_rotated = rotation_vector *v;
	
	cout<<"(1,0,0) after rotation (by angle axis) = "<<v_rotated.transpose()<<endl;
	
	//Euler angle: The rotation matrix can be directly converted into Euler angles
	Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);
	//// ZYX order, ie yaw pitch roll order
	cout<<"Yaw pitch roll = "<<euler_angles.transpose()<<endl;
	
	// Euclidean transformation matrix using Eigen::Isometry
	Isometry3d T = Isometry3d::Identity(); // This 4x4 matrix
	
	// Rotate according to a rotating vector
	T.prerotate(rotation_vector);
	
	// Set the translation vector to (1,3,4)
	T.pretranslate(Vector3d(1, 3, 4));   
	
	cout << "Transform matrix = \n" << T.matrix() << endl;
	
	// Coordinate transformation using transformation matrix
	Vector3d v_transformed = T*v ;// This is equivalent as R*v + t
	cout<<"v transformed = "<<v_transformed.transpose()<<endl;
	
	// For affine and projective transformations, use Eigen::Affine3d and Eigen::Projective3d.
	
	// Quaternion
    // You can assign AngleAxis directly to quaternions, and vice versa
	
	Quaterniond q = Quaterniond(rotation_vector);
	cout<<"quaternion from rotation vector = "<<q.coeffs().transpose()<<endl;
	// Note that the order of coeffs is (x, y, z, w), w is the real part, and the first three are the imaginary parts.
	
	q = Quaterniond(rotation_matrix);
	cout<<"quaternion from rotation matrix = "<<q.coeffs().transpose()<<endl;
	
	// Rotate a vector with a quaternion and use overloaded multiplication
	cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;
	
	// Expressed by conventional vector multiplication, it should be calculated as follows
	cout << "should be equal to " << (q * Quaterniond(0, 1, 0, 0) * q.inverse()).coeffs().transpose() << endl;
	return 0;
}
