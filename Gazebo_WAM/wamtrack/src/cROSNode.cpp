/// \file
/// \brief converts end-effector pose to joint positions
///
/// PARAMETERS:
///     max_xdot (double): maximum translational velocity of the robot
///     goalp (TODO): input goal pose\
/// PUBLISHES:
///     turtle1/cmd_vel (geometry_msgs/Twist): publishes a twist to the robot to control linear and angular velocity
///     joint_traj (trajectory_msgs/JointTrajectory): publishes the resulting joint state from ik
/// SUBSCRIBES:
///     turtle1/pose (turtlesim/Pose): subcribes to /pose to read robot's position
///     TODO or srv\
/// SERVICES:
///     /start (bool): used to execute the turtle's movements by calling member functions and helper services such as:
///     /input (TODO): or sub\

#include "ros/ros.h"

#include "cmath"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
// #include "math.h"
// #include "service_server.h"
#include "sstream"
#include "std_msgs/String.h"
// #include "std_srvs/Empty.h"
// #include "turtlesim/Pose.h"
// #include "turtlesim/SetPen.h"
// #include "turtlesim/TeleportAbsolute.h"
// #include "trect/start.h"
// #include <trect/Start.h>
// #include "../include/turtle_rect.hpp"
// #include "trect/turtle_rect.hpp"
// #include "sensor_msgs/JointState.h"

#include "trajectory_msgs/JointTrajectory"

namespace Wamn  // oh wamn. Oh Wamn. OH. WAMN!
{
    Ikwam::Ikwam()
    {
        //call math library
    }
}