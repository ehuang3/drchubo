/**
 * @file /qlistener/listener.hpp
 * @brief Ros communication central!
 * @date February 2011
 **/
#ifndef qpcl2LISTENER_NODE_HPP_
#define qpcl2LISTENER_NODE_HPP_

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "common/qnode.hpp"
#include <sensor_msgs/PointCloud2.h>

/**
 * @class Listener
 */
class qpcl2Listener : public QNode {

public:
	qpcl2Listener(int argc, char** argv);
	virtual ~qpcl2Listener() {}
	void run();
	void ros_comms_init();
private:
	void pcl2Callback(const sensor_msgs::PointCloud2ConstPtr &msg);
	ros::Subscriber pcl2_subscriber;
};

#endif /* qpcl2LISTENER_NODE_HPP_ */
