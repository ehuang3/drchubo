/**
 * @file grabPointcloud.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Display stuff - PCL 1.5 in ROS, some things are missing, bang! :(
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <boost/thread.hpp>

ros::Publisher pub;
sensor_msgs::PointCloud2ConstPtr grabbed_pc;


/**
 * @function cloud_cb
 * @brief Callback
 */
void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& _input ) {

  // .. Do data processing
  grabbed_pc = _input;

  // Check data we get
  printf("Height of data received: %d \n", grabbed_pc->height );
  printf("Width of data received: %d \n", grabbed_pc->width );
  printf("Point step: %d \n", grabbed_pc->point_step );
  printf("Size of float: %d \n", sizeof(float) );
  printf("Row step in bytes: %d \n", grabbed_pc->row_step );
  printf("Data[0]: %d \n", grabbed_pc->data[10] );
  // Publish the data
  pub.publish( grabbed_pc );

}

/**
 * @function main
 */
int main( int argc, char** argv ) {

  // Initialize ROS
  ros::init( argc, argv, "grabPointcloud" );
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe( "/multisense_sl/camera/points2", 1, cloud_cb );
  
  // Create a ROS publisher for the output PCL pointcloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("output_pointcloud", 1);

  // Keep open
  while( true ) {
    // Spin
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}
