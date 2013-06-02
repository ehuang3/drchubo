#ifndef _HUBO_ZMP_H_
#define _HUBO_ZMP_H_

#include <stdlib.h>

#ifdef HAVE_HUBO_ACH
#include <hubo.h>
#else
#include "src/hubo_joint_count.h"
#endif

enum stance_t {
  DOUBLE_LEFT  = 0,
  DOUBLE_RIGHT = 1,
  SINGLE_LEFT  = 2,
  SINGLE_RIGHT = 3,
};

typedef struct zmp_traj_element {
  double angles[HUBO_JOINT_COUNT];
  // XYZ pos/vel/accel in frame of stance ANKLE
  // (translated up ~10cm from foot -- i.e. z coord is 10cm less)
  double com[3][3];
  double zmp[2]; // XY of zmp in frame of stance ANKLE
  double forces[2][3]; // right/left predicted normal forces
  double torque[2][3]; // right/left predicted moments XYZ
  // TODO: add orientation for IMU
  stance_t stance;
} zmp_traj_element_t;

enum {
  TRAJ_FREQ_HZ = 200,
  MAX_TRAJ_SIZE = 2000,
};

typedef struct zmp_traj {
  zmp_traj_element_t traj[MAX_TRAJ_SIZE];
  size_t count;
} zmp_traj_t;

#define HUBO_CHAN_ZMP_TRAJ_NAME "hubo-zmp-traj"

#endif


