
#include "algorithm.h"

using namespace std;
using namespace Eigen;

double deg2rad(double x)
{
    return x * M_PI/180;
}

void translation_mx_camera_to_inertial(MatrixXd &translation_matrix)
{
    translation_matrix(0, 3) = dx_cam;
    translation_matrix(1, 3) = dy_cam;
    translation_matrix(2, 3) = dz_cam;
    translation_matrix(0, 0) = translation_matrix(3, 3) = 1;
    translation_matrix(1, 1) = translation_matrix(2, 2) = 1;

}
void rot_mx_camera_to_inertial(MatrixXd &rot_mx)
{
    Matrix3d rot_mx_3d = Matrix3d::Zero(3,3);
    double rot_z_angle_degree = 180;
    double rot_y_angle_degree = 90 + angle_camera;
    rot_mx_3d = AngleAxisd(deg2rad(rot_y_angle_degree), Vector3d::UnitY())
      * AngleAxisd(deg2rad(rot_z_angle_degree),  Vector3d::UnitZ());
    rot_mx.block(0,0,3,3) = rot_mx_3d;
}

