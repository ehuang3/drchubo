/**
 * @function gait-timer.h
 */

#ifndef _GAIT_TIMER_H_
#define _GAIT_TIMER_H_

#include "atlas_zmp.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

// Weird
//typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > EIGEN_V_VEC2D;
#define EIGEN_V_VEC2D std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >

struct step_t {
  Eigen::Vector2d pos;
  stance_t support;
};

struct step_traj_t {
  EIGEN_V_VEC2D zmpref;
  std::vector<stance_t> stance;
};

class GaitTimer {
 public:
 GaitTimer()
   : single_support_time(0.70),
    double_support_time(0.05),
    startup_time(1.0),
    shutdown_time(1.0),
    duty_factor(0),
    zmp_dist_gain(0),
    dist_gain(0),
    theta_gain(0),
    height_gain(0) {
  }
  
  double single_support_time; // minimum time for single support
  double double_support_time; // minimum time for double support
  double startup_time;
  double shutdown_time;
  
  // duty factor = support / support + transfer
  // (duty factor - 0.5) * 2 is the % of total time
  // spent in double support phase
  double duty_factor;
  
  double zmp_dist_gain;
  
  double dist_gain; // time gain based on distance
  double theta_gain;
  double height_gain;
  
  void set_duty_cycle(double df, double step_time);
  void set_duty_single(double df, double single_time);
  void set_duty_double(double df, double double_time);
  
  size_t seconds_to_ticks(double s);
  
  size_t compute_startup();
  size_t compute_shutdown();
  size_t compute_double(double dist);
  size_t compute_single(double dist, double theta, double height);
  
  friend std::ostream& operator<<(std::ostream& out, GaitTimer& t);
};

#endif /** _GAIT_TIMER_H_ */
