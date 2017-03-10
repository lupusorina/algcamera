#ifndef PHOTOGRAM_H
#define PHOTOGRAM_H



#include <Eigen/Dense>
#include <Eigen/Geometry>

//////////////////////Sorina Lupu Work///////////////////////////

// PARAMETERS OF THE ENVIRONMENT (EXTRINSIC)
// Position of the camera relative
// to the centre (inertial frame) in the axis of the inertial frame

#define dx_cam                  2.07098     // cm
#define dy_cam                  -15.8151       // cm
#define dz_cam                  34.308338      // cm
#define angle_camera            60      // degrees (measured relative to XoY plane)
#define scale                   6
#define sphere_radius           10.4      // cm
#define unit                    1
#define PIXEL_SIZE              0.006   // mm
#define IMG_SIZE_HEIGH          480
#define IMG_SIZE_WIDTH          744

#define IMG_SIZE_HEIGHT_MM      IMG_SIZE_HEIGH * PIXEL_SIZE
#define IMG_SIZE_WIDTH_MM       IMG_SIZE_WIDTH * PIXEL_SIZE
#define FOCUS                   0.60549     // cm


Eigen::MatrixXd PHOTOGRAM_Point(double x,double y,double z);
Eigen::MatrixXd PHOTOGRAM_Vector(double x,double y,double z);
Eigen::MatrixXd PHOTOGRAM_Vector_normalized(double x,double y, double z);
Eigen::MatrixXd PHOTOGRAM_normalize_vector(Eigen::MatrixXd &v);


float PHOTOGRAM_deg2rad(float x);
void PHOTOGRAM_translation_mx_camera_to_inertial(Eigen::MatrixXd &trans_mx);
void PHOTOGRAM_rot_mx_camera_to_inertial(Eigen::MatrixXd &rot_mx);
double PHOTOGRAM_X_img_px_to_mm_and_centered(double point);
double PHOTOGRAM_Y_img_px_to_mm_and_centered(double point);
Eigen::MatrixXd PHOTOGRAM_gets_center_drawing_plane(Eigen::MatrixXd &n_inertial, Eigen::MatrixXd &E_inertial);
Eigen::MatrixXd PHOTOGRAM_transform_inertial_to_camera(Eigen::MatrixXd &inertial, Eigen::MatrixXd &transform_matrix);
Eigen::MatrixXd PHOTOGRAM_transform_camera_to_inertial(Eigen::MatrixXd &camera, Eigen::MatrixXd &transform_matrix);

double PHOTOGRAM_gs_coefficient(Eigen::MatrixXd &v1, Eigen::MatrixXd &v2);
Eigen::MatrixXd PHOTOGRAM_proj(Eigen::MatrixXd &u1, Eigen::MatrixXd &v2);
Eigen::MatrixXd PHOTOGRAM_gs_first_vect(Eigen::MatrixXd &u1, Eigen::MatrixXd &v2);
Eigen::MatrixXd PHOTOGRAM_gs_second_vect(Eigen::MatrixXd &u1, Eigen::MatrixXd &u2, Eigen::MatrixXd &v3);
float PHOTOGRAM_dot_product(Eigen::MatrixXd &v1, Eigen::MatrixXd &v2);
void PHOTOGRAM_find_vector_bases(Eigen::MatrixXd &u_norm, Eigen::MatrixXd &v_norm, Eigen::MatrixXd &n);
bool PHOTOGRAM_verify_orthogonality(Eigen::MatrixXd &v1, Eigen::MatrixXd &v2);
void PHOTOGRAM_correct_normal_vector(Eigen::MatrixXd &n_inertial);
int PHOTOGRAM_verify_normal_vector(Eigen::MatrixXd &n_camera, Eigen::MatrixXd &transform_matrix);

Eigen::Vector3f PHOTOGRAM_solve_linear_eq(Eigen::MatrixXd &P0, Eigen::MatrixXd &C0, Eigen::MatrixXd &d, Eigen::MatrixXd &u, Eigen::MatrixXd &v);
Eigen::MatrixXd PHOTOGRAM_calculate_intersection_point(Eigen::MatrixXd &d, double landa, Eigen::MatrixXd &c);

Eigen::MatrixXd PHOTOGRAM_create_transform_matrix();
Eigen::MatrixXd PHOTOGRAM_input_transformations(Eigen::MatrixXd &n_camera, Eigen::MatrixXd &d_camera, Eigen::MatrixXd &transform_matrix, Eigen::MatrixXd &n_inertial, Eigen::MatrixXd &u_inertial, Eigen::MatrixXd &v_inertial);


#endif
