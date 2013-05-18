#include <sensor_msgs/Joy.h>
#include "joystick.h"

using namespace std;

void joystick::callback(const sensor_msgs::Joy::ConstPtr& joy){
  cout << "joystick::callback" << endl;
  //goal = larm.A06; 
  cout << "Yay" << endl;
  if (joy->buttons[12] == 1)
    //cout << "Triangle" << endl;
    X = 0.001;
  if (joy->buttons[13] == 1)
    //cout << "Circle" << endl;
    Y =  -0.001;
  if (joy->buttons[14] == 1)
    //cout << "X" << endl;
    X = -0.001;
  if (joy->buttons[15] == 1)
    //cout << "Square" << endl;
    Y = 0.001;

  else{cout << "Fuck" <<endl;}
}
