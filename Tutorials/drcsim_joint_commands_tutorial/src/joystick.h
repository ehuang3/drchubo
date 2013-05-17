#ifndef joystick_H
#define joystick_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class joystick{
public:
joystick():X(0), Y(0), Z(0){}
void callback(const sensor_msgs::Joy::ConstPtr& joy);
float X;
float Y;
float Z;
};

#endif
