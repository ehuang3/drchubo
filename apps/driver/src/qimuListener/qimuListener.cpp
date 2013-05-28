/**
 * @file /qlistener/listener.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/


#include <ros/ros.h>
#include <string>
#include <sstream>
#include <std_msgs/String.h>

#include "qimuListener.hpp"


qimuListener::qimuListener(int argc, char** argv ) :
	QNode(argc,argv,"qqimuListener")
	{}

void qimuListener::ros_comms_init() {
	ros::NodeHandle n;
	chatter_subscriber = n.subscribe("/atlas/imu", 1000, &qimuListener::chatterCallback, this);
}

void qimuListener::chatterCallback(const sensor_msgs::Imu::ConstPtr &msg) {
	ROS_INFO("I heard: [%f %f %f]", msg->orientation.x, msg->orientation.y, msg->orientation.z );
	logging.insertRows(0,1);
	std::stringstream logging_msg;
	logging_msg << "[ INFO] [" << ros::Time::now() << "]: I heard: " << msg->orientation.x<<"," <<msg->orientation.y<<","<< msg->orientation.z ;
	QVariant new_row(QString(logging_msg.str().c_str()));
	logging.setData(logging.index(0),new_row);
}

void qimuListener::run() {
	ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
