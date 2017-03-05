#include "algorithm.h"
#include <iostream>
#include "CppUTest/CommandLineTestRunner.h"

using namespace std;
using namespace Eigen;


TEST_GROUP(Algorithm)
{
};


TEST(Algorithm, test_deg2rad)
{
    double angle = 45;
    double result = deg2rad(angle);
    DOUBLES_EQUAL(result, 0.785398, 0000.1);
}

TEST(Algorithm, test_X_img_px_to_mm_and_centered)
{
    double point = 1000;
    double result = X_img_px_to_mm_and_centered(point);
    DOUBLES_EQUAL(result, 3.768, 0.001);
}

TEST(Algorithm, test_Y_img_px_to_mm_and_centered)
{
    double point = 1000;
    double result = Y_img_px_to_mm_and_centered(point);
    DOUBLES_EQUAL(result, 4.56, 0.001);
}

TEST(Algorithm, test_gs_algorithm)
{
    MatrixXd v1(1,4);
    v1 << 1, 2, 3, 4;
    MatrixXd v2(1,4);
    v2 << 4, 5, 6, 7;
    double result = gs_coefficient(v1, v2);
    DOUBLES_EQUAL(result, 2, 0.0001);
}

TEST(Algorithm, test_proj)
{
    MatrixXd v1(1, 4);
    v1 << 1, 2, 3, 4;
    MatrixXd v2(1, 4);
    v2 << 4, 5, 6, 7;
    MatrixXd result = proj(v1, v2);
    double v1_0 = 2 * 1;
    double v1_1 = 2 * 2;
    double v1_2 = 2 * 3;
    double v1_3 = 2 * 4;
    DOUBLES_EQUAL(result(0,0), v1_0, 0.001);
    DOUBLES_EQUAL(result(0,1), v1_1, 0.001);
    DOUBLES_EQUAL(result(0,2), v1_2, 0.001);
    DOUBLES_EQUAL(result(0,3), v1_3, 0.001);
}

TEST(Algorithm, test_gs)
{
    MatrixXd u_arbitrary(1, 4);
    u_arbitrary << 1, 0, 0, 0;
    MatrixXd b(1, 4);
    b << -0.19518, -0.09759, 0.9759, 0;

    MatrixXd result;
    result = gs(b, u_arbitrary);

    double v1_0 = 0.961905;
    double v1_1 = -0.0190476;
    double v1_2 = 0.190476;
    double v1_3 = 0;

    DOUBLES_EQUAL(result(0,0), v1_0, 0.001);
    DOUBLES_EQUAL(result(0,1), v1_1, 0.001);
    DOUBLES_EQUAL(result(0,2), v1_2, 0.001);
    DOUBLES_EQUAL(result(0,3), v1_3, 0.001);

}

int main(int ac, char** av)
{
   return CommandLineTestRunner::RunAllTests(ac, av);
}



