/**
 * @file /qlistener/listener.hpp
 * @brief Ros communication central!
 * @date February 2011
 **/
#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "common/qnode.hpp"

/**
 * @class Listener
 */
class Listener : public QNode {

public:
	Listener(int argc, char** argv);
	virtual ~Listener() {}
	void run();
	void ros_comms_init();
private:
	void chatterCallback(const std_msgs::String::ConstPtr &msg);
	ros::Subscriber chatter_subscriber;
};

#endif /* LISTENER_NODE_HPP_ */
