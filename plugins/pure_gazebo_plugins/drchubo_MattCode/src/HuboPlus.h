/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _HUBOPLUS_H_
#define _HUBOPLUS_H_

#include "src/fakerave.h"
#include "src/HuboKin.h"

#ifdef HAVE_HUBO_ACH
#include <hubo.h>
#else
#include "hubo_joint_count.h"
#endif

class HuboPlus {
public:  

  enum ManipIndex {
    MANIP_L_FOOT,
    MANIP_R_FOOT,
    MANIP_L_HAND,
    MANIP_R_HAND,
    NUM_MANIPULATORS,
  };

  enum IKMode {
    IK_MODE_FREE,    // you can do whatever you want to these joint angles
    IK_MODE_FIXED,   // joint angles already specified, do not mess with them
    IK_MODE_BODY,    // manipulator specified relative to body
    IK_MODE_WORLD,   // manipulator specified relative to world
    IK_MODE_SUPPORT, // manipulator specfied relative to world, and holding up robot
  };
  
  static const char* ikModeString(int i);
  
  class KState {
  public:

    fakerave::quat       body_rot;
    fakerave::vec3       body_pos;

    fakerave::RealArray  jvalues;

    fakerave::Transform3 xform() const;
    void setXform(const fakerave::Transform3& x);

  };

  fakerave::KinBody kbody;
  fakerave::BodyLookup bl;
  fakerave::JointLookup jl;
  fakerave::ManipulatorLookup ml;

  HK::HuboKin hkin;

  fakerave::vec3 defaultFootPos;
  fakerave::vec3 defaultComPos;
  fakerave::quat footRot;
  
  fakerave::real footAnkleDist;

  fakerave::IndexArray huboJointOrder;

  size_t DEFAULT_COM_ITER;
  fakerave::real DEFAULT_COM_PTOL;
  
  HuboPlus(const std::string& filename);
  
  void render(const fakerave::Transform3Array& xforms,
              const fakerave::Vec4Array* overrideColors=0) const;
  
  void initIK(size_t mi,
              const fakerave::Transform3& desired,
              fakerave::RealArray& jvalues) const;

  bool manipIK(size_t mi,
               const fakerave::Transform3& desired,
               fakerave::RealArray& jvalues,
               fakerave::Transform3Array& work,
               bool global=true) const;

  static const bool* allGlobalIK() {
    static const bool g[4] = { true, true, true, true };
    return g;
  }

  static const bool* noGlobalIK() {
    static const bool g[4] = { false, false, false, false };
    return g;
  }
  
  bool stanceIK( KState& state,
                 const fakerave::Transform3 manipXforms[NUM_MANIPULATORS],
                 const IKMode mode[NUM_MANIPULATORS],
                 const bool globalIK[NUM_MANIPULATORS],
                 fakerave::Transform3Array& work,
                 bool* ikvalid=0 ) const;

  bool comIK( KState& state,
              const fakerave::vec3& com,
              const fakerave::Transform3 manipXforms[NUM_MANIPULATORS],
              const IKMode mode[NUM_MANIPULATORS],
              const bool globalIK[NUM_MANIPULATORS],
              fakerave::Transform3Array& work,
              fakerave::real ascl=10,
              fakerave::real fscl=0,
              bool* ikvalid=0 ) const;

  fakerave::real nonSupportMass(const IKMode mode[2]) const;

  void computeGroundReaction(const fakerave::vec3& comPos,
			     const fakerave::vec3& comAccel,
			     const fakerave::Transform3 footXforms[2],
			     const IKMode mode[2],
			     fakerave::vec3 forces[2],
			     fakerave::vec3 torques[2]) const;

  void computeGroundReaction(const fakerave::real mass,
			     const fakerave::vec3& comPos,
			     const fakerave::vec3& comAccel,
			     const fakerave::Transform3& footXform,
			     fakerave::vec3* force,
			     fakerave::vec3* torque) const;

  
  
};

#endif
