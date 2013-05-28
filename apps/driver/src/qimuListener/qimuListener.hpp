/**
 * @file /qlistener/listener.hpp
 * @brief Ros communication central!
 * @date February 2011
 **/
#ifndef qimuLISTENER_NODE_HPP_
#define qimuLISTENER_NODE_HPP_

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "common/qnode.hpp"
#include <sensor_msgs/Imu.h>

/**
 * @class Listener
 */
class qimuListener : public QNode {

public:
	qimuListener(int argc, char** argv);
	virtual ~qimuListener() {}
	void run();
	void ros_comms_init();
private:
	void chatterCallback(const sensor_msgs::Imu::ConstPtr &msg);
	ros::Subscriber chatter_subscriber;
};

#endif /* qimuLISTENER_NODE_HPP_ */
