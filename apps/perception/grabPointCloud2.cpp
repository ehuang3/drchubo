/**
 * @file grabPointcloud.cpp
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const char WINDOW[] = "SL Sensor";

/**
 * @function cloud_cb
 * @brief Callback
 */
void cloud_cb( const sensor_msgs::PointCloud2ConstPtr& _input ) {


  // Check data we get
  int height = _input->height;
  int width = _input->width;
  int row_step = _input->row_step;
  int point_step = _input->point_step;


  // Create an image to show
  cv::Mat img;
  img = cv::Mat( height, width, CV_8UC3 );

  float x; float y; float z; float rgb;
  int r; int g; int b;
  uint8_t* x_raw = reinterpret_cast<uint8_t*>( &x );
  uint8_t* y_raw = reinterpret_cast<uint8_t*>( &y );
  uint8_t* z_raw = reinterpret_cast<uint8_t*>( &z );
  uint8_t* rgb_raw = reinterpret_cast<uint8_t*>( &rgb );

  // Grab
  for( uint32_t row = 0; row < height; ++row ) {
    const uint8_t* row_data = &( _input->data[row*row_step] );
    for( uint32_t col = 0; col < width; ++col ) {
      const uint8_t* msg_data = row_data + col*point_step;

      // For x, y, z and rgb
      memcpy( x_raw, msg_data + 0, 4 );
      memcpy( y_raw, msg_data + 4, 4 );
      memcpy( z_raw, msg_data + 8, 4 );
      memcpy( rgb_raw, msg_data + 12, 4 );

      // Store
      uint32_t rgbD = *reinterpret_cast<int*>( rgb_raw );

      r = ( rgbD >> 16) & 0x0000ff;
      g = ( rgbD >> 8) & 0x0000ff;
      b = ( rgbD ) & 0x0000ff;

      img.at<cv::Vec3b>(row, col)[0] = b;
      img.at<cv::Vec3b>(row, col)[1] = g;
      img.at<cv::Vec3b>(row, col)[2] = r;

    }
  }

  cv::imshow( WINDOW, img );
  cv::waitKey(3);
    

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
  
  // Create a window
  cv::namedWindow( WINDOW );

  // Keep open
  while( true ) {
    // Spin
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }
}
