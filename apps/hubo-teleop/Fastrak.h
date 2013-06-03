/**
 * \file Fastrak.h
 * \brief Encapsulates Fastrack access for ACH IPC channels.
 *
 * \author Andrew Price
 */
#pragma once

#ifndef FASTRAK_H
#define FASTRAK_H

// ACH includes
#include <stdint.h>
#include <time.h>
#include <string.h>
#include <pthread.h>
#include <sys/stat.h>
#include <ach.h>

// Eigen includes
#include <Eigen/Core>
#include <Eigen/Geometry>

// Message Results
//#include "ft_flag_t.h"
//#include "daemonizer.h"

#include <stdio.h>

//#define FASTRAK_CHAN_NAME "fastrak"

/**
 * \enum ft_flag_t
 * \brief Enumerates possible return values for functions in the Fastrak class.
 */
typedef enum
{
	SUCCESS = 0,	///< The command returned successfully
	SENSOR_OOB,     ///< You requested data from a sensor which doesn't exist
	FASTRAK_STALE,  ///< The Fastrak values were not able to update for some reason
	CHAN_OPEN_FAIL, ///< A channel failed to open
} ft_flag_t;

/**
 * \struct fastrak_data_c_t
 * \brief Container for pose information [X,Y,Z,w,i,j,k] in Ach IPC channels.
 */
typedef struct
{
	float data[4][7];
} fastrak_data_c_t;

/**
 * \class Fastrak
 * \brief Encapsulates Fastrack access for Ach IPC channels.
 * 
 */
class Fastrak
{
public:
	/**
	 * \fn Fastrak
	 * \brief Constructor for Fastrak class
	 * \param [in] channel String containing the name of the Ach channel to attach to
	 * \param [in] assert Uses the Daemon-Assert to verify that the channel was opened. Currently unimplemented.
	 */
	Fastrak(std::string channel="fastrak", bool assert=false);
	~Fastrak(void);

	/**
	 * \fn initFastrak
	 * \brief Initializes the Ach connection; called by the constructor.
	 * \param [in] assert Uses the Daemon-Assert to verify that the channel was opened. Currently unimplemented.
	 */
	ft_flag_t initFastrak(bool assert=false);

	/**
	 * \fn achUpdate
	 * \brief Refreshes the data pulled from the Ach channel.
	 * Called by getPose if update == true.
	 * \return The status of the call.
	 */
	ach_status achUpdate();

	/**
	 * \fn setFastrakScale
	 * \brief Sets a new scale on the <X,Y,Z> component of the pose.
	 * Note that the scale works as x/scale, not x*scale. Also, setting the scale to 0 will be ignored.
	 * \param [in] scale New value for the scaling factor.
	 */
	void setFastrakScale( double scale );

	/**
	 * \fn getFastrakScale
	 * \brief Returns the current scale on the <X,Y,Z> component of the pose.
	 * Note that the scale works as x/scale, not x*scale. Also, setting the scale to 0 will be ignored.
	 * \return Current value for the scaling factor.
	 */
	double getFastrakScale();

	/**
	 * \fn getNumChannels
	 * \brief If dynamic # of channels, returns the current number of sensors connected to the Fastrak, otherwise, 4.
	 * \return Either number of channels or 4.
	 */
	int getNumChannels();

	/**
	 * \fn ft_flag_t Fastrak::getPose( Eigen::Vector3d& position, Eigen::Quaterniond& quat, int sensor=1, bool update=true )
	 * \brief Returns the position and orientation of the indexed Fastrak sensor
	 * \param [out] position Position Vector.
	 * \param [out] quat Orientation Quaternion.
	 * \param [in] sensor Sensor index to return.
	 * \param [in] update Forces object to reload data from the Ach channel.
	 * \return Status of the pose call
	 */
	ft_flag_t getPose( Eigen::Vector3d& position, Eigen::Quaterniond& quat, int sensor=1, bool update=true );

	/**
	 * \fn ft_flag_t getPose( Eigen::Vector3d& position, Eigen::Matrix3d& rotation, int sensor=1, bool update=true )
	 * \brief Returns the position and orientation of the indexed Fastrak sensor
	 * \param [out] position Position Vector.
	 * \param [out] rotation Orientation Rotation Matrix.
	 * \param [in] sensor Sensor index to return.
	 * \param [in] update Forces object to reload data from the Ach channel.
	 * \return Status of the pose call
	 */
	ft_flag_t getPose( Eigen::Vector3d& position, Eigen::Matrix3d& rotation, int sensor=1, bool update=true );

	/**
	 * \fn ft_flag_t getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true )
	 * \brief Returns the position and orientation of the indexed Fastrak sensor
	 * \param [out] tf 3D Pose of the sensor
	 * \param [in] sensor Sensor index to return.
	 * \param [in] update Forces object to reload data from the Ach channel.
	 * \return Status of the pose call
	 */
	ft_flag_t getPose( Eigen::Isometry3d &tf, int sensor=1, bool update=true );

private:
	double fastrakScale;
	ach_channel_t chan_fastrak;
	fastrak_data_c_t fastrak;
	std::string fastrak_chan_name;
	
};

#endif // FASTRAK_H

