/**
 * @file driverInterface.cpp
 */
#include "cvImageWidget.h"

// Qt4 stuff
#include <QDialog>
#include <QApplication>
#include <QMainWindow>

// Ros / OpenCV / PCL stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

static const char gWindowName[] = "Atlas View";
cv::Mat gView;

// Qt4
QMainWindow* gWindow;
CVImageWidget* gImageWidget;

/**
 * @function cloud_cb
 * @brief Callback for sensor_sl message
 */
void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& _input ) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
  pcl::fromROSMsg( *_input, *cloud );

  gView = cv::Mat( cloud->height, cloud->width, CV_8UC3 );
  gImageWidget->showImage( gView );  
  gWindow->show();
  
}

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  // Qt initialization / creation of window
  QApplication app( argc, argv );
  gWindow = new QMainWindow();

  // Create the image widget
  gImageWidget = new CVImageWidget();
  gWindow->setCentralWidget( gImageWidget );

  // Exec?
  printf("Before exec \n");
  app.exec();
  printf("After exec \n");

  // Initialize ROS
  printf("Initialize ROS \n");
  ros::init( argc, argv, "driverInterface" );
  ros::NodeHandle mNh;

  // Create a ROS subscriber for the input from SL sensor
  ros::Subscriber sub = mNh.subscribe( "/multisense_sl/camera/points2", 1, cloud_cb );

  // Keep running
  while( true ) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  


}
