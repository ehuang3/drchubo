/* Teleop test */

#include <iostream>
#include <fstream>
#include "Teleop.h"
#include <getopt.h>

using namespace Eigen;
int main(int argc, char **argv)
{
	const char *teleopDeviceName = "fastrak"; //name of teleop device

    Vector3d SensorOrigin;
    Eigen::Matrix3d RotInitial;
    

	 //create teleop object
	Teleop teleop(teleopDeviceName);
	while(1)
	teleop.getPose(SensorOrigin, RotInitial, 4, true); //get initial pose, 2 sensors

}
