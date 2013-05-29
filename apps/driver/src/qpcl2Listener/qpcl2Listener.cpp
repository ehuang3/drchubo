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

#include "qpcl2Listener.hpp"


qpcl2Listener::qpcl2Listener(int argc, char** argv ) :
	QNode(argc,argv,"qpcl2Listener")
	{}

void qpcl2Listener::ros_comms_init() {
	ros::NodeHandle n;
	pcl2_subscriber = n.subscribe("/multisense_sl/camera/points2", 1, &qpcl2Listener::pcl2Callback, this);
}

void qpcl2Listener::pcl2Callback(const sensor_msgs::PointCloud2ConstPtr &msg) {
	ROS_INFO("I heard: [%d %d %d]", msg->height, msg->width, msg->row_step );
	logging.insertRows(0,1);
	std::stringstream logging_msg;
	logging_msg << "[ INFO] [" << ros::Time::now() << "]: I heard: " << msg->height<<"," <<msg->width<<","<< msg->row_step ;
	QVariant new_row(QString(logging_msg.str().c_str()));
	logging.setData(logging.index(0),new_row);
}

void qpcl2Listener::run() {
	ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
