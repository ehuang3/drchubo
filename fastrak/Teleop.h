/**
 * @file: Teleop.h
 * @brief: Encapsulates any teleoperation device that gives translation and
 * quaternion data about its sensor for the ACH IPC channels.
 * This class can be used currently for either the Polhemus Liberty of Fastrak
 * sensors by giving the constructor a string with "liberty" or "fastrak".
 *
 * \Author Andrew Price
 */

#ifndef TELEOP_H
#define TELEOP_H

#include <iostream>
#include <syslog.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <stdint.h>
#include <stdio.h>

#include <ach.h>

#define EXIT_SUCCESS 0
#define EXIT_FAILURE 1

namespace teleop
{
    typedef enum
    {
        SUCCESS = 0,    ///< The function completed with no errors
        SENSOR_OOB,     ///< You requested data from a sensor which doesn't exist
        VALUE_OOB,      ///< Some generic value was out of acceptable bounds
        SHORT_VECTOR,   ///< The VectorXd you tried to use has too few entries
        LONG_VECTOR,    ///< The VectorXd you tried to use has too many entries
        ALL_STALE,      ///< Nothing was able to update for some reason
        TRACKER_STALE,  ///< Nothing was able to update for some reason
        CHAN_OPEN_FAIL, ///< A channel failed to open
    } flag_t;
}

using namespace teleop;

class Teleop
{
public:
    Teleop(const char *teleopDeviceName="liberty");
    ~Teleop();

    /* Function: initTeleop(bool assert=false)
     * Description: Opens teleop device ach channel
     * Return: Returns a SUCCESS or FAIL flag
    */
    flag_t initTeleop(const char *teleopDeviceName, bool assert=false);

    /* Function: setTeleopScale( double scale );
     * Description: Sets the scale of the Teleop readings
    */
    void setTeleopScale( double scale );

    /* Function: getTeleopScale();
     * Description: Gets the scale of the Teleop readings
    */
    double getTeleopScale();

    /* Function: getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor=1, bool update=true )
     * Description: Gets the sensor pose as a Eigen Vector3d for the position and a Quaternion of doubles for the orientation
     * Return: Returns a success/fail flag
    */
    flag_t getPose( Eigen::Vector3d &position, Eigen::Quaterniond &quat, int sensor=1, bool update=true );

    /* Function: getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor=1, bool update=true )
     * Description: Gets the sensor pose as a 3x1 vector for the position and a 3x3 matrix of doubles for the orientation
     * Return: Returns a success/fail flag
    */
    flag_t getPose( Eigen::Vector3d &position, Eigen::Matrix3d &rotation, int sensor=1, bool update=true );

    /* Function: getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true )
     * Description: Gets the sensor pose as a isometry3d tranformation matrix
     * Return: Returns a success/fail flag
    */
    flag_t getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true );


    static const int NUM_OF_DATA = 7;
    const char *TRACKER_NAME;
    int NUM_OF_SENSORS;
    ach_channel_t chan_teleop;
    double teleopScale;
    typedef struct
    {
        float sensorData[4][NUM_OF_DATA];
    } fastrak_t;

    typedef struct
    {
        float sensorData[8][NUM_OF_DATA];
    } liberty_t;

private:

    fastrak_t fastrakData;
    liberty_t libertyData;
};

#endif // TELEOP_H
