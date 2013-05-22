/**
 * @file trajTesterDemo.cpp
 */
#include "trajTester.h"

/**
 * @function main
 */
int main( int argc, char* argv[] ) {
  
  ros::init( argc, argv, "trajTesterDemo" );

  trajTester tt;
  tt.demo_1();

  ros::spin();
}


