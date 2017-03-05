#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

// PARAMETERS OF THE ENVIRONMENT (EXTRINSIC)
// Position of the camera relative
// to the centre (inertial frame) in the axis of the inertial frame

#define dx_cam          24 // cm
#define dy_cam          0  // cm
#define dz_cam          30 // cm
#define angle_camera    45 // degrees (measured relative to XoY plane)
#define scale           6
#define sphere_radius   10 // cm


void translation_mx_camera_to_inertial(Eigen::MatrixXd &translation_matrix);
void rot_mx_camera_to_inertial(Eigen::MatrixXd &rot_mx);


#endif