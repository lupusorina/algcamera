#include "algorithm.h"
#include "CppUTest/CommandLineTestRunner.h"

TEST_GROUP(ExampleSuite)
{
};


TEST(ExampleSuite, test_deg2rad)
{
    double angle = 45;
    double result = deg2rad(angle);
    CHECK_EQUAL(result, 0.785398);
}

int main(int ac, char** av)
{
   return CommandLineTestRunner::RunAllTests(ac, av);
}
