#include "Teleop.h"
#include <google/protobuf-c/protobuf-c.h>
#include "somatic.h"
#include "somatic.pb-c.h"
#include <iostream>


Teleop::Teleop(const char *teleopDeviceName) : TRACKER_NAME(teleopDeviceName)
{
    initTeleop(teleopDeviceName);
}

Teleop::~Teleop(void)
{
    ach_close(&chan_teleop);
}

flag_t Teleop::initTeleop(const char *teleopDeviceName, bool assert)
{
    if(0 == strcmp(teleopDeviceName,"liberty"))
    {
        NUM_OF_SENSORS = 8;
    }
    else if(0 == strcmp(teleopDeviceName,"fastrak"))
    {
        NUM_OF_SENSORS = 4;
    }
    else
    {
        syslog(LOG_ERR, "Error!. \"%s\" is not a proper tracker name! Please enter \"liberty\" or \"fastrak\".\n", teleopDeviceName);
        exit(EXIT_FAILURE); 
    }

//    teleop.sensorData = new float[NUM_OF_SENSORS][NUM_OF_DATA];
//    std::cout << "sizeof(sensorData): " << sizeof(teleop.sensorData)/sizeof(float) << std::endl;
    
    ach_status_t r = ach_open( &chan_teleop, TRACKER_NAME, NULL );

    if( ACH_OK != r )
    {
        syslog(LOG_ERR, "Unable to open \"%s\" channel: (%d) %s!\n",
            TRACKER_NAME, r, ach_result_to_string((ach_status_t)r));
        return CHAN_OPEN_FAIL;
    }
    syslog(LOG_INFO, "The ach channel \"%s\" opened successfully!\n", TRACKER_NAME);

    teleopScale = 1.0;
    setTeleopScale(1.0);    

    return SUCCESS;    
}

void Teleop::setTeleopScale( double scale ) { teleopScale = scale; }
double Teleop::getTeleopScale() { return teleopScale; };

flag_t Teleop::getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor, bool update )
{
    ach_status_t r = ACH_OK;
    int s;

    if(0 == strcmp(TRACKER_NAME,"liberty"))
    {
        liberty_t teleop;
    }
    else if(0 == strcmp(TRACKER_NAME,"fastrak"))
    {
        fastrak_t teleop;
    }

    if(update) {
        size_t fs;
        if(0 == strcmp(TRACKER_NAME, "liberty"))
        {
            r = ach_get( &chan_teleop, libertyData.sensorData, sizeof(libertyData.sensorData), &fs, NULL, ACH_O_LAST );
            if ( r == ACH_OK ) {
                assert( sizeof(libertyData.sensorData) == fs  );
            } else {
                syslog(LOG_ERR, "The ach_get() call failed: %s\n", ach_result_to_string(r));
            }
        }
        else if(0 == strcmp(TRACKER_NAME, "fastrak"))
        {
            
            Somatic__MultiTransform * contents;
            // Somatic__Vector * translation;
            // Somatic__Vector * rotation;
            // Somatic__Transform *transform;

            uint8_t data[2048];
            //syslog(LOG_INFO, "Got some contents!\n");
            //somatic_verbprintf( 2, "Got reading\n");
            r = ach_get( &chan_teleop, data, 2048*sizeof(uint8_t), &fs, NULL, ACH_O_LAST );
            //syslog(LOG_INFO, "Got some contents!\n");

            contents = somatic__multi_transform__unpack(NULL, fs, data);

            for(;;) {
                if(!contents)
                    break;
                if(!contents->tf)
                    break;
                if(!contents->tf[sensor])
                    break;
                if(!contents->tf[sensor]->translation)
                    break;
                if(!contents->tf[sensor]->translation->data)
                    break;
                
                double *trans = contents->tf[sensor]->translation->data;
                double *quatArray = contents->tf[sensor]->rotation->data;
                
                for(int i=0; i < 3; i++) {
                    position[i] = trans[i];
                }
                quat.w() = quatArray[0];
                quat.x() = quatArray[1];
                quat.y() = quatArray[2];
                quat.z() = quatArray[3];

                std::cout << "[Teleop.cpp] Translation = " << position.transpose() << std::endl;
                std::cout << "[Teleop.cpp] Quaternion = " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << std::endl;
                
                break;
            }

            if ( r == ACH_OK ) {
                //assert( sizeof(contents) == fs ); //We're just going to go ahead and ignore this...

            } else {
                syslog(LOG_ERR, "The ach_get() call failed: %s\n", ach_result_to_string(r));
            
            }
            if(contents)
                somatic__multi_transform__free_unpacked(contents, NULL); //Free the unpacked protobuf packet
        }

    }
 
    if( ACH_OK != r )
        return TRACKER_STALE;
    return SUCCESS;
}


flag_t Teleop::getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor, bool update )
{
    Eigen::Quaterniond quat;
    flag_t flag = getPose( position, quat, sensor, update );

    if( flag==SENSOR_OOB )
        return flag;

    rotation = quat.matrix();

    return flag;
}


flag_t Teleop::getPose( Eigen::Isometry3d &tf, int sensor, bool update )
{
    Eigen::Vector3d position;
    Eigen::Quaterniond quat;

    flag_t flag = getPose( position, quat, sensor, update );

    if( flag==SENSOR_OOB )
        return flag;

    tf = Eigen::Matrix4d::Identity();
    tf.translate( position );
    tf.rotate( quat );

    return flag;
}

