#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

// PARAMETERS OF THE ENVIRONMENT (EXTRINSIC)
// Position of the camera relative
// to the centre (inertial frame) in the axis of the inertial frame

#define dx_cam                  -24     // cm
#define dy_cam                  0       // cm
#define dz_cam                  30      // cm
#define angle_camera            45      // degrees (measured relative to XoY plane)
#define scale                   6
#define sphere_radius           10      // cm
#define unit                    6
#define IMG_SIZE_HEIGHT_PX      480     // PX
#define IMG_SIZE_WIDTH_PX       744     // PX
#define PIXEL_SIZE              0.006   // mm
#define IMG_SIZE_HEIGHT_MM      IMG_SIZE_HEIGHT_PX * PIXEL_SIZE
#define IMG_SIZE_WIDTH_MM       IMG_SIZE_WIDTH_PX * PIXEL_SIZE
#define FOCUS                   0.6     // cm


Eigen::MatrixXd Point(double x,double y,double z);
Eigen::MatrixXd Vector(double x,double y,double z);


double deg2rad(double x);
void translation_mx_camera_to_inertial(Eigen::MatrixXd &translation_matrix);
void rot_mx_camera_to_inertial(Eigen::MatrixXd &rot_mx);
double X_img_px_to_mm_and_centered(double point);
double Y_img_px_to_mm_and_centered(double point);
Eigen::MatrixXd gets_center_drawing_plane(Eigen::MatrixXd &n, Eigen::MatrixXd &e3_inertial, Eigen::MatrixXd &orig_inertial);


Eigen::MatrixXd transform_inertial_to_camera(Eigen::MatrixXd &inertial, Eigen::MatrixXd &transform_matrix);
Eigen::MatrixXd transform_camera_to_inertial(Eigen::MatrixXd &camera, Eigen::MatrixXd &transform_matrix);

bool sphere_eq_verification(Eigen::MatrixXd P0_world);
double gs_coefficient(Eigen::MatrixXd &v1, Eigen::MatrixXd &v2);
Eigen::MatrixXd proj(Eigen::MatrixXd &v1, Eigen::MatrixXd &v2);
Eigen::MatrixXd gs(Eigen::MatrixXd &n, Eigen::MatrixXd &u);
void find_vector_bases(Eigen::MatrixXd &u_norm, Eigen::MatrixXd &v_norm, Eigen::MatrixXd &n);
bool verify_orthogonality(Eigen::MatrixXd &a, Eigen::MatrixXd &b);


#endif