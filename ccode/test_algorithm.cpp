#include "CppUTest/CommandLineTestRunner.h"
#include "algorithm.h"




int main(int ac, char** av)
{
   return CommandLineTestRunner::RunAllTests(ac, av);
}
