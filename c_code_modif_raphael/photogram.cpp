#include "photogram.h"
//#include "constants.h"

#include <iostream>
//#include <QDebug>

//////////////////////Sorina Lupu Work///////////////////////////

using namespace std;
using namespace Eigen;

double PHOTOGRAM_X_img_px_to_mm_and_centered(double point)
{
    return (point - IMG_SIZE_WIDTH / 2) * PIXEL_SIZE; // mm
}

double PHOTOGRAM_Y_img_px_to_mm_and_centered(double point)
{
    return -(point - IMG_SIZE_HEIGH / 2)* PIXEL_SIZE; // mm -> minus sign to fit with photogrametric convention (y down)
}

MatrixXd PHOTOGRAM_Point(double x,double y, double z)
{
    MatrixXd p(4,1);
    p(0, 0) = x;
    p(1, 0) = y;
    p(2, 0) = z;
    p(3, 0) = 1;
    return p;
}

MatrixXd PHOTOGRAM_Vector(double x,double y, double z)
{
    MatrixXd v(4,1);
    v(0, 0) = x;
    v(1, 0) = y;
    v(2, 0) = z;
    v(3, 0) = 0;
    return v;
}

MatrixXd PHOTOGRAM_Vector_normalized(double x,double y, double z)
{
    Vector3f v;
    v(0) = x;
    v(1) = y;
    v(2) = z;
    v.normalize();

    return PHOTOGRAM_Vector(v(0), v(1), v(2));
}

MatrixXd PHOTOGRAM_normalize_vector(MatrixXd &v)
{
    Vector3f V;
    V(0) = v(0, 0);
    V(1) = v(1, 0);
    V(2) = v(2, 0);
    V.normalize();

    return PHOTOGRAM_Vector(V(0), V(1), V(2));
}

float PHOTOGRAM_deg2rad(float x)
{
    return x * M_PI/180;
}

void PHOTOGRAM_translation_mx_camera_to_inertial(MatrixXd &trans_mx)
{
    Matrix4d R = Matrix4d::Identity(4,4);

    R(0, 3) = dx_cam;
    R(1, 3) = dy_cam;
    R(2, 3) = dz_cam;

    trans_mx = R;

}

void PHOTOGRAM_rot_mx_camera_to_inertial(MatrixXd &rot_mx)
{
    Matrix4d R = Matrix4d::Identity(4,4);

    float x = PHOTOGRAM_deg2rad(-(90 + angle_camera));
    float y = PHOTOGRAM_deg2rad(0);
    float z = PHOTOGRAM_deg2rad(0);


    R(0,0) = 1;//cos(z) * cos(y);
    R(1,0) = 0;//cos(z) * sin(x) * sin(y) - sin(z) * cos(x);
    R(2,0) = 0;//cos(z) * cos(x) * sin(y) + sin(z) * sin(x);
    R(0,1) = 0;//sin(z) * cos(y);
    R(1,1) = cos(x);//sin(z) * sin(x) * sin(y) + cos(z) * cos(x);
    R(2,1) = sin(x);//sin(z) * cos(x) * sin(y) - sin(x) * cos(z);
    R(0,2) = 0;//-sin(y);
    R(1,2) = -sin(x);//cos(y) * sin(x);
    R(2,2) = cos(x);//cos(y) * cos(x);

    rot_mx = R;
    cout << rot_mx << endl;
}

MatrixXd PHOTOGRAM_transform_camera_to_inertial(MatrixXd &camera, MatrixXd &transform_matrix)
{
    return transform_matrix * camera;
}

MatrixXd PHOTOGRAM_transform_inertial_to_camera(MatrixXd &inertial, MatrixXd &transform_matrix)
{
    return (transform_matrix.inverse() * inertial).transpose(); //transpose ?
}

MatrixXd PHOTOGRAM_gets_center_drawing_plane(MatrixXd &n_inertial, MatrixXd &E_inertial)
{
    return E_inertial + sphere_radius * n_inertial;
}

bool PHOTOGRAM_verify_orthogonality(MatrixXd &v1, MatrixXd &v2){

    float dot_prod = PHOTOGRAM_dot_product(v1, v2);

    if (dot_prod < 0.0001)
        return true;
    else
        return false;
}

MatrixXd PHOTOGRAM_proj(MatrixXd &u1, MatrixXd &v2){

    return (PHOTOGRAM_dot_product(u1, v2) / PHOTOGRAM_dot_product(u1, u1)) * u1;
}

float PHOTOGRAM_dot_product(MatrixXd &v1, MatrixXd &v2){

    return v1(0, 0) * v2(0, 0) + v1(1, 0) * v2(1, 0) + v1(2, 0) * v2(2, 0);
}

MatrixXd PHOTOGRAM_gs_first_vect(MatrixXd &u1, MatrixXd &v2){    //Gram Shmidt

    return v2 - PHOTOGRAM_proj(u1, v2);
}

MatrixXd PHOTOGRAM_gs_second_vect(MatrixXd &u1, MatrixXd &u2, MatrixXd &v3){    //Gram Shmidt

    return v3 - PHOTOGRAM_proj(u1, v3) - PHOTOGRAM_proj(u2, v3);
}

void PHOTOGRAM_find_vector_bases(MatrixXd &u_norm, MatrixXd &v_norm, MatrixXd &n_norm)
{
    //Arbitrary vector (should not be aligned with n_norm)
    MatrixXd v_1_arbitrary = PHOTOGRAM_Vector_normalized(1, 0, 0); //n_norm normally pointing down
    MatrixXd v_2_arbitrary = PHOTOGRAM_Vector_normalized(0, 1, 0); //n_norm normally pointing down

    //First basis vector:
    MatrixXd u1 = n_norm;

    //Second basis vector:
    MatrixXd u2 = PHOTOGRAM_gs_first_vect(u1, v_1_arbitrary);

    //Third basis vector:
    MatrixXd u3 = PHOTOGRAM_gs_second_vect(u1, u2, v_2_arbitrary);


    u_norm = u2;
    v_norm = u3;
}

Vector3f PHOTOGRAM_solve_linear_eq(MatrixXd &P0, MatrixXd &C0, MatrixXd &d, MatrixXd &u, MatrixXd &v){

    Vector3f V;
    V(0) = P0(0,0) - C0(0,0);
    V(1) = P0(1,0) - C0(1,0);
    V(2) = P0(2,0) - C0(2,0);

    Matrix3f M;

    M(0, 0) = d(0, 0); M(0, 1) = -u(0, 0); M(0, 2) = -v(0, 0);
    M(1, 0) = d(1, 0); M(1, 1) = -u(1, 0); M(1, 2) = -v(1, 0);
    M(2, 0) = d(2, 0); M(2, 1) = -u(2, 0); M(2, 2) = -v(2, 0);

    return M.inverse() * V;
}

MatrixXd PHOTOGRAM_calculate_intersection_point(MatrixXd &d, double landa, MatrixXd &c){
    return c + landa * d;
}

MatrixXd PHOTOGRAM_create_transform_matrix(){
    // Create Rotation and Translation Matrix => Transform Matrix
    MatrixXd rotation_matrix = MatrixXd::Zero(4,4);
    MatrixXd translation_matrix = MatrixXd::Zero(4,4);
    MatrixXd transform_matrix = MatrixXd::Zero(4,4);

    PHOTOGRAM_rot_mx_camera_to_inertial(rotation_matrix);
    PHOTOGRAM_translation_mx_camera_to_inertial(translation_matrix);
    transform_matrix = translation_matrix * rotation_matrix;
    return transform_matrix;
}

int PHOTOGRAM_verify_normal_vector(MatrixXd &n_camera, MatrixXd &transform_matrix){

    //Keep only normal vector pointing up (in inertial frame)

    MatrixXd n_inertial = PHOTOGRAM_transform_camera_to_inertial(n_camera, transform_matrix);

    //Z component has to be negative and larger than other compoenents:
    if(fabs(n_inertial(2, 0)) > fabs(n_inertial(0, 0))        &&
            fabs(n_inertial(2, 0)) > fabs(n_inertial(1, 0)))
        return 1;
    else
        return 0;
}

void PHOTOGRAM_correct_normal_vector(MatrixXd &n_inertial){

    //n_inertial(1, 0) = -n_inertial(1, 0);
    n_inertial(2, 0) = fabs(n_inertial(2, 0));
}

MatrixXd PHOTOGRAM_input_transformations(MatrixXd &n_camera, MatrixXd &d_camera, MatrixXd &transform_matrix, MatrixXd &n_inertial, MatrixXd &u_inertial, MatrixXd &v_inertial){

    MatrixXd E_inertial = PHOTOGRAM_Point(0, 0, 0);
    MatrixXd E_camera = PHOTOGRAM_Point(0, 0, 0);
    MatrixXd C0_inertial = PHOTOGRAM_Point(0, 0, 0);
    MatrixXd P0_inertial = PHOTOGRAM_Point(0, 0, 0);
    MatrixXd d_inertial = PHOTOGRAM_Vector(unit, 0, 0);
    //MatrixXd n_inertial = PHOTOGRAM_Vector(unit, 0, 0);

    // TERM 1: C0_camera - position of the camera in inertial frame
    C0_inertial = PHOTOGRAM_transform_camera_to_inertial(E_camera, transform_matrix);

    // TERM 2: d (direction_center_elipse)
    // d represents the direction vector starting at the camera center in the
    // direction of the point of interest (e.g. center of the smaller ellipses)
    d_inertial = PHOTOGRAM_transform_camera_to_inertial(d_camera, transform_matrix);

    // TERM 3: P0
    // P0 represents the center of the main ellipse. It is computed by starting
    // in the centre of the sphere, moving up in the direction of the n vector
    n_inertial = PHOTOGRAM_transform_camera_to_inertial(n_camera, transform_matrix);
    PHOTOGRAM_correct_normal_vector(n_inertial);

    P0_inertial = PHOTOGRAM_gets_center_drawing_plane(n_inertial, E_inertial);

    // TERM 4,5: U,V using Gram-Schmidt
    //MatrixXd u_inertial = PHOTOGRAM_Vector(unit, 0, 0);
    //MatrixXd v_inertial = PHOTOGRAM_Vector(unit, 0, 0);
    PHOTOGRAM_find_vector_bases(u_inertial, v_inertial, n_inertial);

    if (!PHOTOGRAM_verify_orthogonality(u_inertial, v_inertial) ||
        !PHOTOGRAM_verify_orthogonality(n_inertial, v_inertial) ||
        !PHOTOGRAM_verify_orthogonality(n_inertial, u_inertial)) {
        cout<<"N,U,V not orthogonal"<<endl;
    }

    // Solve Linear equation
    Vector3f coefficients = PHOTOGRAM_solve_linear_eq(P0_inertial, C0_inertial, d_inertial, u_inertial, v_inertial);

    // Get the point
    MatrixXd I = PHOTOGRAM_Point(0, 0, 0);
    I = PHOTOGRAM_calculate_intersection_point(d_inertial, coefficients(0), C0_inertial);
    return I;
}


