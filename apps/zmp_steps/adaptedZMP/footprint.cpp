/**
 * @file footprint.cpp
 */
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <iomanip>

#include "footprint.h"

/**
 * @function Footprint
 * @brief Constructor
 */
Footprint::Footprint(Eigen::Matrix4d t, bool is_left) {

  this->transform = t;
  this->is_left = is_left;
}

/**
 * @function Footprint
 * @brief Constructor
 */
Footprint::Footprint( double x, 
		      double y, 
		      double theta, 
		      bool is_left ) {

  this->transform = Eigen::Matrix4d::Identity();
  this->transform.block(0,3,3,1) = Eigen::Vector3d( x, y, 0 );
  Eigen::AngleAxisd rotation( theta, Eigen::Vector3d(0.0,0.0,1.0) );
  this->transform.block(0,0,3,3) = rotation.matrix();

  this->is_left = is_left;
}

/**
 * @function Footprint
 * @brief Yet another constructor
 */
Footprint::Footprint(){
    Footprint( 0.0, 0.0, 0.0, false );
}


/**
 * @function x
 * @brief Get x
 */
double Footprint::x() const { 
  return this->transform(0,3); 
}

/**
 * @function y
 * @brief Get y
 */
double Footprint::y() const{ 
  return this->transform(1,3); 
}

/**
 * @function theta
 * @brief Get theta 
 */
double Footprint::theta() const {
  Eigen::Vector3d forward = this->transform.block(0,0,3,3) * Eigen::Vector3d(1.0, 0.0, 0.0);
  return atan2(forward.y(), forward.x());
}

/**
 * @function getTransform
 * @brief Get the transform
 */
Eigen::Matrix4d Footprint::getTransform() const {
    return transform;
}

/**
 * @function setTransform
 * @brief Set transform
 */
void Footprint::setTransform( Eigen::Matrix4d transform ){
    this->transform = transform;
}

/**
 * @function getMidTransform 
 * @brief
 */
Eigen::Matrix4d Footprint::getMidTransform( double width ) const {
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block(0,0,3,1) = Eigen::Vector3d( 0, is_left?-width:width, 0 );
  return getTransform() * trans;
}


/**
 * @function getRotMat
 */
Eigen::Matrix3d Footprint::getRotMat() const {
  return transform.block(0,0,3,3);
}

/**
 * @function getTranslation
 */
Eigen::Vector3d Footprint::getTranslation() const {
  return transform.block(0,3,3,1);
}

/**
 * @function is_even
 * @brief Helper is even?
 */
bool is_even(int i) {
    return (i%2) == 0;
}

/**
 * @function is_odd
 * @brief Helper is odd?
 */
bool is_odd(int i) {
    return !is_even(i);
}

/**
 * @function walkline
 * @brief
 */
std::vector<Footprint> walkLine(double distance,
				double width,
				double max_step_length,
				Footprint stance_foot) {
  return walkCircle( 1e13,
		     distance,
		     width,
		     max_step_length,
		     3.14,
		     stance_foot );
}

/**
 * @function walkCircle
 * @brief 
 */
std::vector<Footprint> walkCircle( double radius,
				   double distance,
				   double width,
				   double max_step_length,
				   double max_step_angle,
				   Footprint stance_foot ) {
  assert(distance > 0);
  assert(width > 0);
  assert(max_step_length > 0);
  assert(max_step_angle >= -3.14159265359);
  assert(max_step_angle < 3.14159265359);
  
  bool left_is_stance_foot = stance_foot.is_left;
  
  // select stance foot, fill out transforms
  // Since the coordinates of the footprint will be given in world frame
  Eigen::Matrix4d stanceT = stance_foot.transform;

  // The transformations generated for swing.cpp are in global frame, no in feet frame (DH)
  // so we have to convert them
  Eigen::Matrix4d ToFootFrame = Eigen::Matrix4d::Identity();
  ToFootFrame(0,0) = 0; ToFootFrame(0,1) = 0; ToFootFrame(0,2) = -1;
  ToFootFrame(1,0) = 0; ToFootFrame(1,1) = 1; ToFootFrame(1,2) = 0;
  ToFootFrame(2,0) = 1; ToFootFrame(2,1) = 0; ToFootFrame(2,2) = 0;

  // And multiply for the transf of the origin of the footprints (middle of both feet)
  Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
  trans.block(0,3,3,1) = Eigen::Vector3d(0, left_is_stance_foot?-width:width, 0);

  Eigen::Matrix4d T_circle_to_world = stanceT*ToFootFrame*trans;

  // For some reason abs acts crazy so I have to do this
  double _radius_;
  if( radius > 0 ) { _radius_ = radius; }
  else { _radius_ = -1*radius; }

  double alpha = distance / _radius_;
  double outer_dist = (_radius_ + width) * alpha;
  int K_step_length = ceil(outer_dist / max_step_length);
  int K_angle = ceil(alpha / max_step_angle);
  int K = std::max(K_step_length, K_angle);
  double dTheta = alpha/K * (radius > 0 ? 1 : -1 );

  // init results list
  std::vector<Footprint> result;
  
  // fill out results
  for(int i = 2; i < K + 1; i++) {
    double theta_i = dTheta * (i - 1);
    if (is_even(i) xor left_is_stance_foot) { // i odd means this step is for the stance foot
      result.push_back(Footprint((radius - width) * sin(theta_i),
				 radius - ((radius - width) * cos(theta_i)),
				 theta_i,
				 true));
    }
    else {
      result.push_back(Footprint((radius + width) * sin(theta_i),
				 radius - ((radius + width) * cos(theta_i)),
				 theta_i,
				 false));
    }
  }
  
  // fill out the last two footsteps
  double theta_last = dTheta * K;
  if (is_even(K) xor left_is_stance_foot) { // K even means we end on the stance foot
    result.push_back(Footprint((radius + width) * sin(theta_last),
			       radius - ((radius + width) * cos(theta_last)),
			       theta_last,
			       false));
    result.push_back(Footprint((radius - width) * sin(theta_last),
			       radius - ((radius - width) * cos(theta_last)),
			       theta_last,
			       true));
  }
  else {
    result.push_back(Footprint((radius - width) * sin(theta_last),
			       radius - ((radius - width) * cos(theta_last)),
			       theta_last,
			       true));
    result.push_back(Footprint((radius + width) * sin(theta_last),
			       radius - ((radius + width) * cos(theta_last)),
			       theta_last,
			       false));
  }
  
  // run through results transforming them back into the original frame of reference
  for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {

    Eigen::Matrix4d temp = it->transform;
    it->transform = T_circle_to_world * temp;
    it->transform_w = temp;

    std::cout << "Footprint: "<< it->x()<< ","<<it->y()<<", "<<it->transform(2,3) <<" theta: "<< it->theta() << std::endl;
    std::cout << "Transform: \n"<< it->transform<<"\n Transform_w: \n"<< it->transform_w << std::endl;
    }
  
  result.insert(result.begin(), stance_foot);
  
  // return the result
  return result;
}

/**
 * @function turnInPlace
 */
std::vector<Footprint> turnInPlace( double desired_angle, /// The desired angle
				    double width, /// The desired angle
				    double max_step_angle, /// The maximum HALF angle between successive steps
				    Footprint from /// Where we start from. Note that this exact foot will be repeated in the output
				    ) {
  
  assert(max_step_angle >= -3.14159265359);
  assert(max_step_angle <   3.14159265359);
  assert(desired_angle >=  -3.14159265359);
  assert(desired_angle <    3.14159265359);
  assert(from.theta() >=   -3.14159265359);
  assert(from.theta() <     3.14159265359);

  double goal_angle = desired_angle - from.theta();
  if(goal_angle >= 3.14159265359) goal_angle -= 3.14159265359;
  else if( goal_angle < -3.14159265359) goal_angle -= 3.14159265359;
  double eps = 1e-10;
  return walkCircle(eps, eps*goal_angle, width, 1000, max_step_angle, from);
}

/**
 * @function compensate
 */
Eigen::Matrix4d compensate( double width, bool is_left ) {

  Eigen::Matrix4d c = Eigen::Matrix4d::Identity();
  c.block(0,3,3,1) = Eigen::Vector3d(0, is_left?-width:width, 0);
  return c;
}

/**
 * @function walkTo
 */
std::vector<Footprint> walkTo(
			      double width, /// The maximum HALF angle between successive steps
			      double max_step_length, /// The maximum HALF allowed length the robot may step
			      double max_step_angle, /// The maximum HALF angle between successive steps
			      Footprint from, /// Where we start from. Note that this exact foot will be repeated in the output
			      Footprint to /// Where we should end at. Note that this exact foot will be repeated in the output
			      ) {

  /* double walk_angle = to. */
  Eigen::Matrix4d T_delta = from.getMidTransform(width).inverse() * to.getMidTransform(width);

  /* to.y(), to.x() */
  Eigen::Vector3d trans = T_delta.block(0,3,3,1);
  double walk_angle = atan2( trans.y(), trans.x() );
  
  std::vector<Footprint> turn1 = turnInPlace( walk_angle, width, max_step_angle, from );
  Footprint f1 = turn1.back();
  turn1.pop_back();
  
  double walk_length = trans.norm();
  std::vector<Footprint> turn2 = walkLine(walk_length, width, max_step_length, f1);
  Footprint f2 = turn2.back();
  turn2.pop_back();
  
  std::vector<Footprint> turn3 = turnInPlace(to.theta(), width, max_step_angle, f2);
  
  std::vector<Footprint> total = turn1;
  total.insert(total.end(), turn2.begin(), turn2.end());
  total.insert(total.end(), turn3.begin(), turn3.end());
  return total;
}
