/*
 * @file VisualDataNode.cpp
 * @brief Ros communication central!
 * @date 2013, May 28th
 */
#include <ros/ros.h>
#include <string>
#include <sstream>
#include <std_msgs/String.h>

#include "VisualDataNode.hpp"

/**
 * @function VisualDataNode
 * @brief Constructor
 */
VisualDataNode::VisualDataNode(int argc, char** argv ) :
  QNode( argc,argv,"VisualDataNode" )
{}

/**
 * @function ros_comms_init
 * @brief Init the communication
 */
void VisualDataNode::ros_comms_init() {
  ros::NodeHandle n;
  pcl2_subscriber = n.subscribe("/multisense_sl/camera/points2", 1, 
				&VisualDataNode::pcl2_Callback, this );
}

/**
 * @function pcl2_Callback
 * @brief Called when a new pcl2 message is gotten
 */
void VisualDataNode::pcl2_Callback(const sensor_msgs::PointCloud2ConstPtr &_msg) {

	ROS_INFO("I heard: [%d %d %d]", _msg->height, _msg->width, _msg->row_step );
	/*
	logging.insertRows(0,1);
	std::stringstream logging_msg;
	logging_msg << "[ INFO] [" << ros::Time::now() << "]: I heard: " << msg->height<<"," <<msg->width<<","<< msg->row_step ;
	QVariant new_row(QString(logging_msg.str().c_str()));
	logging.setData(logging.index(0),new_row);
	*/
}

/**
 * @function run
 * @brief Run ROS loop (spin)
 */
void VisualDataNode::run() {
  ros::spin();
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
