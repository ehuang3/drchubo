/**
 * @file VisualDataNode.h
 */
#ifndef _VISUAL_DATA_NODE_H_
#define _VISUAL_DATA_NODE_H_

#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include "common/qnode.hpp"
#include <sensor_msgs/PointCloud2.h>

/**
 * @class VisualDataNode
 */
class VisualDataNode : public QNode {

public:
  VisualDataNode(int argc, char** argv);
  virtual ~VisualDataNode() {}
  void run();
  void ros_comms_init();

 private:
  void pcl2_Callback( const sensor_msgs::PointCloud2ConstPtr &_msg );
  ros::Subscriber pcl2_subscriber;
};

#endif /* _VISUAL_DATA_NODE_H_ */
