#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "algorithm.h"


using namespace std;
using namespace Eigen;

MatrixXd create_transform_matrix(){
    // Create Rotation and Translation Matrix => Transform Matrix
    MatrixXd rot_matrix = MatrixXd::Identity(4,4);
    MatrixXd translation_matrix = MatrixXd::Identity(4,4);
    MatrixXd transform_matrix = MatrixXd::Zero(4,4);

    rot_mx_camera_to_inertial(rot_matrix);
    translation_mx_camera_to_inertial(translation_matrix);
    transform_matrix = translation_matrix * rot_matrix;
    return transform_matrix;
}

MatrixXd input_transformations(MatrixXd &n_camera, MatrixXd &elipse_center_cam, MatrixXd &transform_matrix){
    // Create Inertial Frame and World Frame

    MatrixXd e1_inertial, e2_inertial, e3_inertial, orig_inertial;
    MatrixXd cam_e1_cam, cam_e2_cam, cam_e3_cam, cam_orig_cam;

    e1_inertial = Vector(unit, 0, 0);
    e2_inertial = Vector(0, unit, 0);
    e3_inertial = Vector(0, 0, unit);
    orig_inertial = Point(0, 0, 0);

    cam_e1_cam = Vector(unit, 0, 0);
    cam_e2_cam = Vector(0, unit, 0);
    cam_e3_cam = Vector(0, 0, unit);
    cam_orig_cam = Point(0, 0, 0);

    MatrixXd c0_world = MatrixXd::Zero(1,4);

    // TERM 1: C0_world - position of the camera in world coordinates

    c0_world = transform_camera_to_inertial(cam_orig_cam, transform_matrix);

    // TERM 2: d (direction_center_elipse)
    // d represents the direction vector starting at the camera center in the
    // direction of the point of interest (e.g. center of the smaller ellipses)
    MatrixXd dir_center_elipse_world = transform_camera_to_inertial(elipse_center_cam, transform_matrix);

    // TERM 3: P0
    // P0 represents the center of the main ellipse. It is computed by starting
    // in the centre of the sphere, moving up in the direction of the n vector
    MatrixXd n_camera_transpose = n_camera.transpose();
    MatrixXd n_world = transform_camera_to_inertial(n_camera, transform_matrix);
    MatrixXd n_world_transpose = n_world.transpose();
    MatrixXd P0_world = gets_center_drawing_plane(n_world_transpose, e3_inertial, orig_inertial);

    if (!sphere_eq_verification(P0_world)) {
        throw std::invalid_argument( "P0 doesn't verify sphere equation" );
    }

    // TERM 4,5: U,V using Gram-Schmidt
    MatrixXd u_world_normalized(1,4);
    MatrixXd v_world_normalized(1,4);
    find_vector_bases(u_world_normalized, v_world_normalized, n_world_transpose);

    if (!verify_orthogonality(u_world_normalized, v_world_normalized) ||
        !verify_orthogonality(n_world_transpose, v_world_normalized) ||
        !verify_orthogonality(n_world_transpose, u_world_normalized)) {
        throw std::invalid_argument( "N,U,V not orthogonal" );
    }

    // Solve Linear equation
    MatrixXd coefficients(3,1);
    coefficients = solve_linear_eq(P0_world, c0_world, dir_center_elipse_world, u_world_normalized, v_world_normalized);

    // Get the point
    MatrixXd point(1,3);
    double landa = coefficients(0,0);
    point = calculate_intersection_point(dir_center_elipse_world, landa, c0_world);
    return point;
}

int main()
{

    MatrixXd transform_matrix = create_transform_matrix();

    // INPUT IN THE ALGORITHM - TO BE REPLACED
    // WITH OPENCV ellipse determination & n vectors
    // from Sebastien

    double x_elip_px = 375.992;
    double y_elip_px = 166.052;
    double x_elip_cm = X_img_px_to_mm_and_centered(x_elip_px)/10;
    double y_elip_cm = Y_img_px_to_mm_and_centered(y_elip_px)/10;

    MatrixXd elipse_center_cam = Vector(x_elip_cm, y_elip_cm, FOCUS);
    MatrixXd n_world_input = Vector(0.2, 0.1, -1);
    MatrixXd n_camera = transform_inertial_to_camera(n_world_input, transform_matrix);

    MatrixXd point(3,1);
    point = input_transformations(n_camera, elipse_center_cam, transform_matrix);
    cout << point;
}
