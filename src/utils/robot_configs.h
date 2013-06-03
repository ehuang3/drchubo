#pragma once

// This file defines several macros for working with Atlas and Hubo in a consistent manner.

//#define ATLAS_SANDIA_CONFIG
//#define ATLAS_IROBOT_CONFIG
//#define ATLAS_CONFIG
#define HUBO_CONFIG

#ifdef ATLAS_CONFIG

#include <atlas/atlas_state.h>
#include <atlas/atlas_jacobian.h>
#include <atlas/atlas_kinematics.h>
#ifndef ROBOT_URDF
#define ROBOT_URDF "models/atlas/atlas_world.urdf"
#endif
#ifndef __ROBOT_NAME
#define __ROBOT_NAME "atlas"
#endif
#define __ROBOT_HEAD "head"
#define __ROBOT_BODY "pelvis"
#define __ROBOT_LEFT_HAND "l_hand"
#define __ROBOT_RIGHT_HAND "r_hand"
#define __ROBOT_LEFT_FOOT "l_foot"
#define __ROBOT_RIGHT_FOOT "r_foot"
#define ROBOT_JACOBIAN_T atlas::atlas_jacobian_t
#define ROBOT_STATE_T atlas::atlas_state_t
#define ROBOT_KINEMATICS_T atlas::atlas_kinematics_t

#elif defined HUBO_CONFIG

#include <hubo/hubo_state.h>
#include <hubo/hubo_jacobian.h>
#include <hubo/hubo_kinematics.h>
#ifndef ROBOT_URDF
#define ROBOT_URDF "models/drchubo-master/hubo_world.urdf"
#endif
#ifndef __ROBOT_NAME
#define __ROBOT_NAME "golem_hubo"
#endif
#define __ROBOT_HEAD "Body_NKP"
#define __ROBOT_BODY "Body_Hip"
#define __ROBOT_LEFT_HAND "Body_LWR"
#define __ROBOT_RIGHT_HAND "Body_RWR"
#define __ROBOT_LEFT_FOOT "Body_LAR"
#define __ROBOT_RIGHT_FOOT "Body_RAR"
#define ROBOT_JACOBIAN_T hubo::hubo_jacobian_t
#define ROBOT_STATE_T hubo::hubo_state_t
#define ROBOT_KINEMATICS_T hubo::hubo_kinematics_t

#else
#ifndef ROBOT_URDF
#define ROBOT_URDF ""
#endif
#ifndef __ROBOT_NAME
#define __ROBOT_NAME ""
#endif
#define __ROBOT_HEAD ""
#define __ROBOT_BODY ""
#define __ROBOT_LEFT_HAND ""
#define __ROBOT_RIGHT_HAND ""
#define __ROBOT_LEFT_FOOT ""
#define __ROBOT_RIGHT_FOOT ""
#define ROBOT_JACOBIAN_T is_not_defined_ROBOT_JACOBIAN_T
#define ROBOT_STATE_T is_not_defined_ROBOT_STATE_T
#define ROBOT_KINEMATICS_T is_not_defined_ROBOT_KINEMATICS_T

#endif

static const char* ROBOT_NAME = __ROBOT_NAME;
static const char* ROBOT_HEAD = __ROBOT_HEAD;
static const char* ROBOT_BODY = __ROBOT_BODY;
static const char* ROBOT_LEFT_HAND = __ROBOT_LEFT_HAND;
static const char* ROBOT_RIGHT_HAND = __ROBOT_RIGHT_HAND;
static const char* ROBOT_LEFT_FOOT = __ROBOT_LEFT_FOOT;
static const char* ROBOT_RIGHT_FOOT = __ROBOT_RIGHT_FOOT;
