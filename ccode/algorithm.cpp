
#include "algorithm.h"
#include <iostream>
using namespace std;
using namespace Eigen;

double X_img_px_to_mm_and_centered(double point)
{
    return (point - IMG_SIZE_WIDTH_PX / 2) * PIXEL_SIZE; // cm
}

double Y_img_px_to_mm_and_centered(double point)
{
    return (point - IMG_SIZE_HEIGHT_PX / 2)* PIXEL_SIZE; // cm
}

MatrixXd Point(double x,double y, double z)
{
    MatrixXd p(1,4);
    p << x, y, z, 1;
    return p;
}

MatrixXd Vector(double x,double y, double z)
{
    MatrixXd v(1,4);
    v << x, y, z, 0;
    return v;
}

double deg2rad(double x)
{
    return x * M_PI/180;
}

void translation_mx_camera_to_inertial(MatrixXd &translation_matrix)
{
    translation_matrix(0, 3) = dx_cam;
    translation_matrix(1, 3) = dy_cam;
    translation_matrix(2, 3) = dz_cam;
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

MatrixXd transform_camera_to_inertial(MatrixXd &camera, MatrixXd &transform_matrix)
{
    return transform_matrix * camera.transpose();
}

MatrixXd transform_inertial_to_camera(MatrixXd &inertial, MatrixXd &transform_matrix)
{
    return (transform_matrix.inverse() * inertial.transpose()).transpose();
}

MatrixXd gets_center_drawing_plane(MatrixXd &n, MatrixXd &e3_inertial, MatrixXd &orig_inertial)
{
    MatrixXd P0_world;
    MatrixXd product(0,0);
    product = e3_inertial * n.transpose();
    if (product(0,0) < 0)
        n = -n;  // make sure n points up
    P0_world = orig_inertial + sphere_radius * n.normalized();
    return P0_world.transpose();
}

bool sphere_eq_verification(MatrixXd P0_world){
    if (floor(P0_world.norm()) == sphere_radius)
        return true;
    else
        return false;
}
bool verify_orthogonality(MatrixXd &a, MatrixXd &b){
    if ((a * b.transpose())(0,0) < 0.0001)
        return true;
    else
        return false;
}

double gs_coefficient(MatrixXd &v1, MatrixXd &v2){
    return (v2 * v1.transpose())(0,0)/(v1 * v1.transpose())(0,0);
}

MatrixXd proj(MatrixXd &v1, MatrixXd &v2){
    return gs_coefficient(v1,v2)*v1;
}

MatrixXd gs(MatrixXd &n, MatrixXd &u){
    MatrixXd u1;
    u1 = n;
    MatrixXd u2;
    u2 = u - proj(u1, u);
    return u2;
}


void find_vector_bases(MatrixXd &u_norm, MatrixXd &v_norm, MatrixXd &n_norm)
{
    MatrixXd u_arbitrary = Vector(1, 0, 0);
    MatrixXd u(1,4);
    MatrixXd v(1,4), v_cross_product(1,4);
    u = gs(n_norm, u_arbitrary);
    v_cross_product = u.block<1,3>(0,0).cross(n_norm.block<1,3>(0,0));
    v_norm = Vector(v_cross_product(0,0), v_cross_product(0,1), v_cross_product(0,2));
    u_norm = u.normalized();

}


