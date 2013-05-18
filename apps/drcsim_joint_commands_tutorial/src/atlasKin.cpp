#include <math.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <sensor_msgs/JointState.h>
#include <osrf_msgs/JointCommands.h>
#include <eigen3/Eigen/Dense>
#include "atlasKin.h"

using namespace Eigen;



