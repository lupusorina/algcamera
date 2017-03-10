#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "photogram.h"


using namespace std;
using namespace Eigen;

int main()
{

    MatrixXd transform_matrix = PHOTOGRAM_create_transform_matrix();

    // INPUT IN THE ALGORITHM - TO BE REPLACED
    // WITH OPENCV ellipse determination & n vectors
    // from Sebastien

    double x_elip_px = 375.992;
    double y_elip_px = 166.052;
    double x_elip_cm = PHOTOGRAM_X_img_px_to_mm_and_centered(x_elip_px)/10;
    double y_elip_cm = PHOTOGRAM_Y_img_px_to_mm_and_centered(y_elip_px)/10;


    MatrixXd elipse_center_cam = PHOTOGRAM_Vector(x_elip_cm, y_elip_cm, FOCUS);
    //MatrixXd n_world_input = PHOTOGRAM_Vector(0.2, 0.1, -1);
    //MatrixXd n_camera = PHOTOGRAM_transform_inertial_to_camera(n_world_input, transform_matrix);

    MatrixXd n_camera = PHOTOGRAM_Vector(0.023, 0.732, 0.680);
    MatrixXd point(3,1);
    MatrixXd n_inertial;
    MatrixXd u_inertial;
    MatrixXd v_inertial;
    point = PHOTOGRAM_input_transformations(n_camera, elipse_center_cam, transform_matrix, n_inertial, u_inertial, v_inertial);
    cout << point;
}
