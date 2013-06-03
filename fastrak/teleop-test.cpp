/* Teleop test */

#include <iostream>
#include <fstream>
#include "Teleop.h"
#include <getopt.h>







/*
void   somatic__multi_transform__init
                     (Somatic__MultiTransform         *message);
size_t somatic__multi_transform__get_packed_size
                     (const Somatic__MultiTransform   *message);
size_t somatic__multi_transform__pack
                     (const Somatic__MultiTransform   *message,
                      uint8_t             *out);
size_t somatic__multi_transform__pack_to_buffer
                     (const Somatic__MultiTransform   *message,
                      ProtobufCBuffer     *buffer);

Somatic__Vector *
       somatic__vector__unpack
                     (ProtobufCAllocator  *allocator,
                      size_t               len,
                      const uint8_t       *data);



void   somatic__multi_transform__free_unpacked
                     (Somatic__MultiTransform *message,
                      ProtobufCAllocator *allocator);

*/






using namespace Eigen;

 /**
  * @function: main(int argc, char **argv)
  * @brief: Main function that loops reading the sensors and commands Atlas's joints based on position of the fastrak sensors.
 */

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


//somatic multitransform unpack
//typedef struct _Somatic__MultiTransform Somatic__MultiTransform;
//free it afterwards, use default allocater