
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "algorithm.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::AngleAxisf;
using Eigen::Vector3f;
using Eigen::Matrix3f;


void translation_mx_camera_to_inertial(MatrixXd &translation_matrix){
    translation_matrix(0, 3) = dx_cam;
    translation_matrix(1, 3) = dy_cam;
    translation_matrix(2, 3) = dz_cam;
    translation_matrix(0, 0) = translation_matrix(3, 3) = 1;
    translation_matrix(1, 1) = translation_matrix(2, 2) = 1;

}
void rot_mx_camera_to_inertial(MatrixXd &rot_mx){
    Matrix3f rot_mx_3d = Matrix3f::Zero(3,3);;
    float rot_z_angle_degree = 180;
    float rot_y_angle_degree = 90 + angle_camera;
    float rot_z_angle_radians = rot_z_angle_degree * M_PI/180;
    float rot_y_angle_radians = rot_y_angle_degree * M_PI/180;
    rot_mx_3d = AngleAxisf(rot_y_angle_radians, Vector3f::UnitY())
      * AngleAxisf(rot_z_angle_radians,  Vector3f::UnitZ());
    for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rot_mx(i,j) = rot_mx(i,j) + rot_mx_3d(i,j);
}
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