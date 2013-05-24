/**
 * @file gait-time.cpp
 */

#include "gait-timer.h"
#include <iostream>

///////////////////////////////////////////////////////////////
// some private constants

const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};



///////////////////////////////////////////////////////////////
// actual code


void GaitTimer::set_duty_cycle(double df, double step_time) {
  duty_factor = df;
  double_support_time = (df - 0.5) * 2 * step_time;
  single_support_time = (1 - df) * step_time;
}

void GaitTimer::set_duty_single(double df, double single_time) {
  set_duty_cycle(df, single_time/(1-df));
}

void GaitTimer::set_duty_double(double df, double double_time) {
  set_duty_cycle(df, double_time/((df-0.5)*2));
}

size_t GaitTimer::seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}

size_t GaitTimer::compute_startup() {
  return seconds_to_ticks(startup_time);
}
size_t GaitTimer::compute_shutdown() {
  return seconds_to_ticks(shutdown_time);
}
size_t GaitTimer::compute_double(double dist) {
  double double_time = double_support_time;
  double_time += zmp_dist_gain * dist;
  return seconds_to_ticks(double_time);
}
size_t GaitTimer::compute_single(double dist, double theta, double height) {
  double single_time = single_support_time;
  single_time += height_gain * height;
  single_time += dist_gain * dist;
  single_time += theta_gain * theta;
  return seconds_to_ticks(single_time);
}

std::ostream& operator<<(std::ostream& out, GaitTimer& t) {
  return
    out << "step-timer:\n"
	<< "start up time: " << t.startup_time << " s\n"
	<< "shutdown time: " << t.shutdown_time << " s\n"
	<< "single support time: " << t.single_support_time << " s\n"
	<< "double support time: " << t.double_support_time << " s\n"
	<< "duty factor set: " << t.duty_factor << "\n"
	<< "duty factor calc: " << (t.single_support_time + t.double_support_time)/(2*t.single_support_time + t.double_support_time) << "\n"
	<< "dist gain:   " << t.dist_gain << " s/m\n"
	<< "theta gain:  " << t.theta_gain << " s/m\n"
	<< "height gain: " << t.height_gain << " s/m\n";
}
