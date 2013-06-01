/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "HuboPlus.h"
#include <mzcommon/glstuff.h>
#include <mzcommon/TimeUtil.h>

#define assert_equal(x, y) _assert_equal((x), (y), #x, #y, __FILE__, __LINE__)





using namespace fakerave;
using namespace HK;

  enum IKMode {
    IK_MODE_FREE,    // you can do whatever you want to these joint angles
    IK_MODE_FIXED,   // joint angles already specified, do not mess with them
    IK_MODE_BODY,    // manipulator specified relative to body
    IK_MODE_WORLD,   // manipulator specified relative to world
    IK_MODE_SUPPORT, // manipulator specfied relative to world, and holding up robot
  };
  
const char* HuboPlus::ikModeString(int i) {
  switch (i) {
  case IK_MODE_FREE:    return "free";
  case IK_MODE_FIXED:   return "fixed";
  case IK_MODE_BODY:    return "body";
  case IK_MODE_WORLD:   return "world";
  case IK_MODE_SUPPORT: return "support";
  default: return "[INVALID]";
  }
}


static inline void _assert_equal(real x, real y, 
                                 const char* xstr, 
                                 const char* ystr,
                                 const char* file,
                                 int line) {
  if (fabs(x-y) > 1e-12) {
    std::cerr << file << ":" << line << ": " 
              << xstr << "=" << x << ", " 
              << ystr << "=" << y << "\n";
    abort();
  }
}

const char* mnames[4] = {
  "leftFootManip",
  "rightFootManip",
  "leftHandManip",
  "rightHandManip",
};

std::string flipy(const std::string& name) {
  size_t idx = name.find('L');
  if (idx != std::string::npos) {
    std::string n = name;
    n[idx] = 'R';
    return n;
  } else {
    return name;
  }
}

Transform3 flipy(const Transform3& t) {

  mat4_t<real> m = mat4_t<real>::identity();
  m(1,1) = -1;

  return Transform3(m * t.matrix() * m);

}

vec3 flipy(const vec3& v) {
  return vec3(v.x(), -v.y(), v.z());
}

void compare(const vec3& l, const vec3& r) {
  assert( (l - r).norm() < 1e-8 );
}

void compare(const Transform3& l, const Transform3& r) {
  vec3 dp, dq;
  deltaTransform(l, r, dp, dq);
  assert( dp.norm() < 1e-8 && dq.norm() < 1e-8 );
}

void ensureMirror(const Joint& jl, const Joint& jr) {

  vec3 al = jl.anchor;
  vec3 ar = jr.anchor;

  compare(al, flipy(ar));
  compare(jl.axis, jr.axis);

  if (!jl.axis.y()) {
    if (jl.limits[0] < jl.limits[1]) {
      assert(jl.limits[0] == -jr.limits[1]);
      assert(jl.limits[1] == -jr.limits[0]);
    }
  }

}

void ensureMirror(const Body& bl, const Body& br) {

  Transform3 xl = bl.xform;
  Transform3 xr = br.xform;

  compare(xl, flipy(xr));

}

class AnchorLookup {
public:
  const KinBody& kbody;
  const Transform3Array& xforms;
  AnchorLookup(const KinBody& k, const Transform3Array& x): kbody(k), xforms(x) {}
  vec3 operator()(const std::string& n) {
    return kbody.jointAnchor(xforms, kbody.lookupJoint(n));
  }
};

HuboPlus::HuboPlus(const std::string& filename): 
  bl(kbody), jl(kbody), ml(kbody)

{
  
  DEFAULT_COM_ITER = 10000;
  DEFAULT_COM_PTOL = 1e-3;

  kbody.loadXML(filename);

  for (size_t i=0; i<4; ++i) {
    assert( ml(mnames[i]) == i );
  }

  for (size_t j=0; j<kbody.joints.size(); ++j) {
    const Joint& jleft = kbody.joints[j];
    size_t jj = jl(flipy(jleft.name));
    if (jj != j && jj != size_t(-1)) {
      const Joint& jright = kbody.joints[jj];
      ensureMirror(jleft, jright);
    }
  }

  for (size_t b=0; b<kbody.bodies.size(); ++b) {
    const Body& bleft = kbody.bodies[b];
    size_t bb = bl(flipy(bleft.name));
    if (bb != b && bb != size_t(-1)) {
      const Body& bright = kbody.bodies[bb];
      ensureMirror(bleft, bright);
    }
  }

  Transform3Array xforms;
  RealArray jvalues(kbody.joints.size(), 0.0);

  kbody.transforms(jvalues, xforms);

  defaultFootPos = kbody.manipulatorFK(xforms, 0).translation();
  defaultComPos = kbody.com(xforms);

  // now set some lengths and stuff
  HK::HuboKin::KinConstants& kc = hkin.kc;

  AnchorLookup ja(kbody, xforms);

  // FIX
  kbody.alignJoint(jl("LAR"), xforms, ja("LHR"), vec3(0,1,0), false);
  kbody.alignJoint(jl("RAR"), xforms, ja("RHR"), vec3(0,1,0), false);
  //kbody.alignJoint(jl("HPY"), xforms, vec3(0), vec3(0,1,0), false);

  real desiredMass = 43.5;
  real totalMass = kbody.totalMass();
  std::cerr << "MASS WAS " << totalMass << "\n";
  kbody.adjustTotalMass(desiredMass / totalMass);
  std::cerr << "MASS IS " << kbody.totalMass() << "\n";

  IKMode mode[2] = { IK_MODE_WORLD, IK_MODE_WORLD };
  std::cerr << "FULL BODY MASS = " << nonSupportMass(mode) << "\n";
  mode[0] = IK_MODE_SUPPORT;
  std::cerr << "SINGLE SUPPORT MASS = " << nonSupportMass(mode) << "\n";
  mode[1] = IK_MODE_SUPPORT;
  std::cerr << "DOUBLE SUPPORT MASS = " << nonSupportMass(mode) << "\n";
  
  assert_equal( ja("LHY").y(), ja("LHR").y() );
  assert_equal( ja("LHY").y(), ja("LAR").y() );
  assert_equal( ja("LHY").x(), ja("LHP").x() );
  assert_equal( ja("LHY").x(), ja("LKP").x() );
  assert_equal( ja("LHY").x(), ja("LAP").x() );

  kbody.alignJoint(jl("LSP"), xforms, ja("LSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LSR"), xforms, ja("LSP"), vec3(1,0,0), true);
  kbody.alignJoint(jl("LSY"), xforms, ja("LSP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("LEP"), xforms, ja("LSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LWY"), xforms, ja("LWP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("LWP"), xforms, ja("LSR"), vec3(0,1,0), true);

  kbody.alignJoint(jl("RSP"), xforms, ja("RSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RSR"), xforms, ja("RSP"), vec3(1,0,0), true);
  kbody.alignJoint(jl("RSY"), xforms, ja("RSP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("REP"), xforms, ja("RSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RWY"), xforms, ja("RWP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("RWP"), xforms, ja("RSR"), vec3(0,1,0), true);

  kbody.alignJoint(jl("LHY"), xforms, ja("LHR"), vec3(0,0,1), true);
  kbody.alignJoint(jl("LHR"), xforms, ja("LHY"), vec3(1,0,0), true);
  kbody.alignJoint(jl("LHP"), xforms, ja("LHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LKP"), xforms, ja("LHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LAP"), xforms, ja("LHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LAR"), xforms, ja("LHY"), vec3(1,0,0), true);

  kbody.alignJoint(jl("RHY"), xforms, ja("RHR"), vec3(0,0,1), true);
  kbody.alignJoint(jl("RHR"), xforms, ja("RHY"), vec3(1,0,0), true);
  kbody.alignJoint(jl("RHP"), xforms, ja("RHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RKP"), xforms, ja("RHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RAP"), xforms, ja("RHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RAR"), xforms, ja("RHY"), vec3(1,0,0), true);

  // Are joint offsets contributing to disagreement between FK's?
  // It seems like not...
  if (0)  {
    for (size_t ji=0; ji<kbody.joints.size(); ++ji) {
      kbody.transforms(jvalues, xforms);
      vec3 anchor = kbody.jointAnchor(xforms, ji);
      size_t bi = kbody.joints[ji].body2Index;
      vec3 origin = xforms[bi].translation();
      vec3 diff = anchor - origin;
      kbody.offsetBody(bi, diff);
    }
  }

  kbody.transforms(jvalues, xforms);

  for (size_t j=0; j<kbody.joints.size(); ++j) {
    const Joint& jleft = kbody.joints[j];
    size_t jj = jl(flipy(jleft.name));
    if (jj != j && jj != size_t(-1)) {
      const Joint& jright = kbody.joints[jj];
      ensureMirror(jleft, jright);
    }
  }

  for (size_t b=0; b<kbody.bodies.size(); ++b) {
    const Body& bleft = kbody.bodies[b];
    size_t bb = bl(flipy(bleft.name));
    if (bb != b && bb != size_t(-1)) {
      const Body& bright = kbody.bodies[bb];
      ensureMirror(bleft, bright);
    }
  }


  kc.leg_l1 = ja("LSP").z() - ja("HPY").z();
  kc.leg_l2 = ja("LHY").y();
  kc.leg_l3 = ja("HPY").z() - ja("LHR").z();
  kc.leg_l4 = ja("LHR").z() - ja("LKP").z();
  kc.leg_l5 = ja("LKP").z() - ja("LAR").z();
  kc.leg_l6 = ja("LAR").z() - defaultFootPos.z();

  // handle the limits 
  const IndexArray& jidx = kbody.manipulators[1].jointIndices;
  for (size_t i=0; i<jidx.size(); ++i) {
    real& lo = kc.leg_limits(i,0);
    real& hi = kc.leg_limits(i,1);
    const Joint& j = kbody.joints[jidx[i]];
    if (j.limits[0] < j.limits[1]) {
      lo = j.limits[0];
      hi = j.limits[1];
    } else {
      lo = -M_PI/2;
      hi = M_PI/2;
    }
  }

  std::cerr << "leg_l1 = " << kc.leg_l1 << "; // neck -> waist Z\n";
  std::cerr << "leg_l2 = " << kc.leg_l2 << "; // waist -> hip Y\n";
  std::cerr << "leg_l3 = " << kc.leg_l3 << "; // waist -> hip Z\n";
  std::cerr << "leg_l4 = " << kc.leg_l4 << "; // hip -> knee Z\n";
  std::cerr << "leg_l5 = " << kc.leg_l5 << "; // knee -> ankle Z\n";
  std::cerr << "leg_l6 = " << kc.leg_l6 << "; // ankle to foot Z\n\n";

  std::cerr << "leg_limits << \n";
  for (size_t i=0; i<jidx.size(); ++i) {
    std::cerr << "  " << kc.leg_limits(i, 0) << ", " << kc.leg_limits(i, 1);
    if (i+1 == jidx.size()) {
      std::cerr << ";\n\n";
    } else {
      std::cerr << ",\n";
    }
  }

  real jy = kc.leg_l2;
  real jz = kc.leg_l1 + kc.leg_l3 + kc.leg_l4 + kc.leg_l5 + kc.leg_l6;

  footAnkleDist = kc.leg_l6;
  std::cerr << "FOOT ANKLE DIST = " << footAnkleDist << "\n";

  size_t lfoot = bl("Body_LAR");
  size_t rfoot = bl("Body_RAR");
  vec3 t0 = xforms[lfoot].transformInv(vec3(0, jy, -jz));

  real s2 = sqrt(2)/2;
  footRot = quat(0, s2, 0, s2);

  kbody.manipulators[0].xform = Transform3(t0);

  t0 = xforms[rfoot].transformInv(vec3(0, -jy, -jz));
  kbody.manipulators[1].xform = Transform3(t0);


  const char* hnames[42] = {
    "HPY",
    0, 0, 0,
    "LSP", "LSR", "LSY", "LEP", "LWY", 0, "LWP",
    "RSP", "RSR", "RSY", "REP", "RWY", 0, "RWP",
    0,
    "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
    0,
    "RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0,
  };

  for (size_t i=0; i<42; ++i) {
    size_t ji = size_t(-1);
    if  (hnames[i]) {
      ji = kbody.lookupJoint(hnames[i]);
      if (ji == size_t(-1)) {
	std::cerr << "Error: joint " << hnames[i] << " not found!\n";
      }
      assert(ji != size_t(-1));
    }
    huboJointOrder.push_back(ji);
  }



}


void HuboPlus::render(const Transform3Array& xforms,
                      const Vec4Array* overrideColors) const {
  
  kbody.render(xforms,
               vec4(0.5,0.5,0.5,1), 
               overrideColors);

}

bool HuboPlus::manipIK(size_t mi,
                       const Transform3& desired,
                       RealArray& jvalues,
                       Transform3Array& xforms,
                       bool global) const {

  if (global && (mi == MANIP_L_FOOT || mi == MANIP_R_FOOT)) { 
    
    Transform3 dd = desired * Transform3(footRot);
    mat4_t<real> m = dd.matrix();
    Isometry3d B;

    for (int i=0; i<4; ++i) { 
      for (int j=0; j<4; ++j) {
        B(i,j) = m(i,j);
      }
    }

    const Manipulator& manip = kbody.manipulators[mi];
    const IndexArray& jidx = manip.jointIndices;

    Vector6d qprev, q;
    stdvec2mat(jvalues, qprev, jidx);
    
    int side = (mi == MANIP_L_FOOT ? HuboKin::SIDE_LEFT : HuboKin::SIDE_RIGHT);
    
    hkin.legIK(q, B, qprev, side);

    mat2stdvec(q, jvalues, jidx);
    

    vec3 dp, dq;

    if (1) {

      kbody.transforms(jvalues, xforms, &manip.activeBodies);
      Transform3 fk = kbody.manipulatorFK(xforms, mi);
      deltaTransform(desired, fk, dp, dq);
      
    } else {

      hkin.legFK(B, q, side);

      for (int i=0; i<4; ++i) { 
        for (int j=0; j<4; ++j) {
          m(i,j) = B(i, j);
        }
      }

      deltaTransform(dd, Transform3(m), dp, dq);

    }
    //std::cout << "dp.norm() = " << dp.norm() << "\n";
    //std::cout << "dq.norm() = " << dq.norm() << "\n";

    return (dp.norm() <= kbody.DEFAULT_PTOL && dq.norm() <= kbody.DEFAULT_QTOL);

  } else {
    
    if (global) { initIK(mi, desired, jvalues); }
    return kbody.manipulatorIK(mi, desired, jvalues, xforms);

  }

}

void HuboPlus::initIK(size_t mi,
                      const Transform3& desired,
                      RealArray& jvalues) const {

  const real deg = M_PI/180;
  const real lang = 30*deg;


  if (mi == MANIP_L_FOOT) {
    jvalues[ jl("LHP") ] = -lang;
    jvalues[ jl("LKP") ] = 2*lang;
    jvalues[ jl("LAP") ] = -lang;
  } else if (mi == MANIP_R_FOOT) {
    jvalues[ jl("RHP") ] = -lang;
    jvalues[ jl("RKP") ] = 2*lang;
    jvalues[ jl("RAP") ] = -lang;
  } else if (mi == MANIP_L_HAND) {
    jvalues[ jl("LSP") ] = lang;
    jvalues[ jl("LEP") ] = -2*lang;
    jvalues[ jl("LWP") ] = lang;
  } else if (mi == MANIP_R_HAND) {
    jvalues[ jl("RSP") ] = lang;
    jvalues[ jl("REP") ] = -2*lang;
    jvalues[ jl("RWP") ] = lang;
  } else {
    kbody.centerJoints(kbody.manipulators[mi].jointIndices, jvalues);
  }
  
}


bool HuboPlus::stanceIK( KState& state,
                              const Transform3 manipXforms[NUM_MANIPULATORS],
                              const IKMode mode[NUM_MANIPULATORS],
                              const bool shouldInitIK[NUM_MANIPULATORS],
                              Transform3Array& work,
                              bool* ikvalid ) const {

  bool allOK = true;

  kbody.transforms(state.jvalues, work);

  for (int i=0; i<NUM_MANIPULATORS; ++i) {

    Transform3 desired = manipXforms[i];
    bool doIK = false;
    
    switch (mode[i]) {
    case IK_MODE_BODY:
      doIK = true;
      break;
    case IK_MODE_WORLD:
    case IK_MODE_SUPPORT:
      doIK = true;
      desired = state.xform().inverse() * desired;
      break;
    default:
      doIK = false;
      break;
    }

    bool valid = true;

    if (doIK) {

      Transform3 cur = kbody.manipulatorFK(work,i);
      
      if ( (cur.translation() - desired.translation()).norm() > kbody.DEFAULT_PTOL ||
           quat::dist(cur.rotation(), desired.rotation()) > kbody.DEFAULT_QTOL) {

        if (shouldInitIK[i]) { initIK(i, desired, state.jvalues); }
        
        valid = manipIK(i, desired, state.jvalues, work);

      }
      
    }

    if (!valid) { allOK = false; }
    if (ikvalid) { ikvalid[i] = valid; }

  }

  return allOK;

}

#define debug if (0) std::cerr

bool HuboPlus::comIK( KState& state,
                      const vec3& dcom,
                      const Transform3 manipXforms[NUM_MANIPULATORS],
                      const IKMode mode[NUM_MANIPULATORS],
                      const bool globalIK[NUM_MANIPULATORS],
                      Transform3Array& work,
                      real ascl,
                      real fscl,
                      bool* ikvalid ) const {

  bool ok = false;

  MatX gT, gpT, gxT, fxT, fpT, lambda, gxxpT(6, 3), deltap;
  MatX gfT, deltaf;

  const real alpha = 0.5;
  
  IndexArray pdofs;
  for (size_t i=DOF_POS_X; i<=DOF_ROT_Z; ++i) {
    pdofs.push_back(i);
  }

  IndexArray fdofs;
  for (int i=0; i<4; ++i) {
    if (fscl && mode[i] == IK_MODE_FREE) {
      const IndexArray& jidx = kbody.manipulators[i].jointIndices;
      for (size_t j=0; j<jidx.size(); ++j) {
        fdofs.push_back(jidx[j]);
      }
    }
  }

  for (size_t iter=0; iter<DEFAULT_COM_ITER; ++iter) {

    // try doing IK
    ok = stanceIK( state, manipXforms, mode, globalIK, work, ikvalid );


    // compute the COM pos
    kbody.transforms(state.jvalues, work);

    vec3 com = state.xform() * kbody.com(work);

    // get the error
    vec3 comerr = dcom - com;

    if (comerr.norm() < DEFAULT_COM_PTOL) {
      debug << "breaking after " << iter << " iterations\n";
      break; 
    } else {
      ok = false;
    }

        
    if (ascl == 0) {

      state.body_pos += alpha * comerr;

    } else {
      
      // get jacobians ftw
      kbody.comJacobian(work, pdofs, gpT);
      gpT.block(3, 0, 3, 3) *= ascl;
      

      debug << "gpT=" << gpT << "\n\n";

      if (!fdofs.empty()) {
        kbody.comJacobian(work, fdofs, gfT);
        debug << "gfT=" << gfT << "\n\n";
      }

      gxxpT.setZero();

      for (int i=0; i<4; ++i) {

        if (mode[i] == IK_MODE_WORLD || mode[i] == IK_MODE_SUPPORT) {

          const std::string& name = kbody.manipulators[i].name;

          kbody.comJacobian(work, kbody.manipulators[i].jointIndices, gxT);
          kbody.manipulatorJacobian(work, i, pdofs, fpT);
          kbody.manipulatorJacobian(work, i, fxT);
          lambda = fxT.colPivHouseholderQr().solve(gxT);

          fpT.block(3, 0, 3, 6) *= ascl;

          debug << "gxT[" << name << "]=\n" << gxT << "\n\n";
          debug << "fpT[" << name << "]=\n" << fpT << "\n\n";
          debug << "fxT[" << name << "]=\n" << fxT << "\n\n";
          debug << "lambda[" << name << "]=\n" << lambda << "\n\n";
          gxxpT += fpT * lambda;

        }

      }

      gT = gpT - gxxpT;
      Eigen::Vector3d cerr(comerr[0], comerr[1], comerr[2]);
      deltap = alpha * gT * cerr;

      debug << "gxxpT = \n" << gxxpT << "\n\n";
      debug << "gT = \n" << gT << "\n\n";
      debug << "deltap = \n" << deltap.transpose() << "\n\n";


      vec3 dp(deltap(0), deltap(1), deltap(2));
      vec3 dq(deltap(3), deltap(4), deltap(5));

      state.body_pos += dp;
      state.body_rot = quat::fromOmega(-dq) * state.body_rot;


    }

    if (!fdofs.empty()) {
      Eigen::Vector3d cerr(comerr[0], comerr[1], comerr[2]);
      deltaf = fscl * gfT * cerr;
      debug << "deltaf = \n" << deltaf.transpose() << "\n\n";
      for (size_t i=0; i<fdofs.size(); ++i) {
        state.jvalues[fdofs[i]] += deltaf(i);
      }
    }

  }

  return ok;

}
                           

Transform3 HuboPlus::KState::xform() const {
  return Transform3(body_rot, body_pos);
}

void HuboPlus::KState::setXform(const Transform3& t) {
  body_pos = t.translation();
  body_rot = t.rotation();
}


real HuboPlus::nonSupportMass(const IKMode mode[2]) const {

  real totalMass = 0;

  for (size_t bi=0; bi<kbody.bodies.size(); ++bi) {
    
    bool addMass = true;

    for (int f=0; f<2; ++f) {
      if (mode[f] == IK_MODE_SUPPORT) {
	const Manipulator& m = kbody.manipulators[f];
	for (int j=0; j<2; ++j) {
	  size_t ji = m.jointIndices[m.jointIndices.size()-j-1];
	  if (kbody.bodyDependsOnJoint(bi, ji)) {
	    addMass = false;
	  }
	}
      }
    }

    if (addMass) { totalMass += kbody.bodies[bi].mass; }

  }
  
  return totalMass;

}

const real g = 9.8;

void HuboPlus::computeGroundReaction(const vec3& comPos,
				     const vec3& comAccel,
				     const Transform3 footXforms[2],
				     const IKMode mode[2],
				     vec3 forces[2],
				     vec3 torques[2]) const {

  // Get the mass
  real m = nonSupportMass(mode);

  if (mode[0] == IK_MODE_SUPPORT && mode[1] == IK_MODE_SUPPORT) {

    // If we're in dual support

    // Find the displacement between the feet
    vec3 ry = footXforms[0].translation() - footXforms[1].translation();
    assert( fabs(ry.z()) < 1e-4 );

    // Let d be the distance between the feet
    real d = ry.norm();
    ry /= d;
    
    vec3 rz = vec3(0,0,1);
    vec3 rx = vec3::cross(ry,rz);
    assert( fabs( rx.norm() - 1.0 ) < 1e-8 );
    
    mat3 R;
    R.setRow(0, rx);
    R.setRow(1, ry);
    R.setRow(2, rz);
    
    // Set up a transformation from "world" space to the 
    // frame whose y axis points from right angle to left ankle
    // and whose origin is at the right ankle
    Transform3 forceFrame( quat::fromMat3(R), 
			   -R * footXforms[1].translation() );

    assert(comAccel.z() == 0);

    // COM position in the force frame
    vec3 cp = forceFrame * comPos;

    // Grab x and y coord of COM in force frame
    real x = cp.x();
    real c = cp.y();
    real h = cp.z();

    // COM acceleration the force frame
    vec3 ca = forceFrame.rotFwd() * comAccel;
    ca[2] += g;

    vec3 ff[2];

    real ratio = 1-c/d;

    ff[1][0] = m * (ratio*ca[0] + x*ca[1]/d);
    ff[1][1] = m * ratio * ca[1];
    ff[1][2] = m * (ratio*ca[2] + h*ca[1]/d);

    ff[0] = m*ca - ff[1];

    mat3 RT = R.transpose();
    
    for (int f=0; f<2; ++f) {

      mat3 forceToFoot = footXforms[f].rotInv() * RT;

      if (forces) { forces[f] = forceToFoot * ff[f]; }

      if (torques) {
	// Moment arm for each foot
	vec3 r = (f == 0) ? vec3(-x, d-c, -h) : -cp;
	torques[f] = forceToFoot * vec3::cross(r, ff[f]);
      }

    }

  } else if (mode[0] == IK_MODE_SUPPORT || mode[1] == IK_MODE_SUPPORT) {

    // If we're in single support

    int stance = (mode[1] == IK_MODE_SUPPORT ? 1 : 0);
    int swing = 1-stance;

    computeGroundReaction(m, comPos, comAccel, footXforms[stance], 
			  forces ? forces + stance : 0,
			  torques ? torques + stance : 0);

    if (forces) { forces[swing] = vec3(0); }
    if (torques) { torques[swing] = vec3(0); }

  } else {

    // no support
    if (forces) { forces[0] = forces[1] = vec3(0); }
    if (torques) { torques[0] = torques[1] = vec3(0); }
    
  }

    

}

void HuboPlus::computeGroundReaction(const real m,
				     const vec3& comPos,
				     const vec3& comAccel,
				     const Transform3& footXform,
				     vec3* force,
				     vec3* torque) const {

  assert(comAccel.z() == 0);

  // COM in frame of foot
  vec3 cp = footXform.transformInv(comPos);

  // COM accel in frame of foot
  vec3 ca = footXform.rotInv() * comAccel;
  ca[2] += g; // tack on gravity to the Z component

  // F = ma
  vec3 f = m*ca;


  if (force)  { *force = f; }
  if (torque) { 
    // moment arm for torque from foot
    vec3 r = -cp;
    *torque = vec3::cross(r, f); 
  }

}
