#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "algorithm.h"


using namespace std;
using namespace Eigen;

int main()
{
    MatrixXd rot_matrix = MatrixXd::Zero(4,4);
    MatrixXd translation_matrix = MatrixXd::Zero(4,4);
    MatrixXd transform_matrix = MatrixXd::Zero(4,4);
    rot_matrix(3,3) = 1;
    rot_mx_camera_to_inertial(rot_matrix);

    translation_mx_camera_to_inertial(translation_matrix);

    transform_matrix = translation_matrix * rot_matrix;
    cout << transform_matrix << endl;
}
