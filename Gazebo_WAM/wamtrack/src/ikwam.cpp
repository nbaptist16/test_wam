#include "ikwam/ikwam.hpp"
#include <iostream>

#include <eigen3/Eigen/Dense>
#include <vector>
using namespace std;
using namespace Eigen;

namespace ikwam
{
    //

    // joint configs OR joint velos (for desired ee twist)
    JointConfig::JointConfig()
    {
        j0 = 0.0;
        j1 = 0.0;
        j2 = 0.0;
        j3 = 0.0;
        j4 = 0.0;
        j5 = 0.0;
        j6 = 0.0;
        j7 = 0.0;
    }

    JointConfig::JointConfig(double j0in, double j1in, double j2in, double j3in, double j4in, double j5in, double j6in, double j7in)
    {
        j0 = j0in;
        j1 = j1in;
        j2 = j2in;
        j3 = j3in;
        j4 = j4in;
        j5 = j5in;
        j6 = j6in;
        j7 = j7in;
    }

    // law of cosines function

    // two-argument arctangent function

    // Newton-Raphson root finding function

    // Inv velo kinematics function (for finding)
        // desired twist, V_d, and Jacobian, J)
}