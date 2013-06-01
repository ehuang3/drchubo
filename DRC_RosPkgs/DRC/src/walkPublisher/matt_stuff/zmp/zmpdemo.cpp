#include "zmp/hubo-zmp.h"
#include "src/HuboPlus.h"
#include <math.h>
#include "mzcommon/MzGlutApp.h"
#include "mzcommon/TimeUtil.h"
#include <getopt.h>

#include "zmp/zmpwalkgenerator.h"
#include "zmp/footprint.h"

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}
/*TODO remove this
*/
const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};


class ZmpDemo: public MzGlutApp {
public:

  HuboPlus& hplus;
  KinBody& kbody;

  HuboPlus::KState state;
  vec3 forces[2];
  vec3 torques[2];
  vec3 actualCom;
  vec3 actualComVel;
  vec3 actualComAcc;

  Transform3Array xforms;

  const TrajVector& traj;
  
  size_t cur_index;

  int stance_foot;
  Transform3 stance_foot_xform;
  
  GLUquadric* quadric;

  bool animating;

  ZmpDemo(int argc, char** argv, HuboPlus& h, const TrajVector& t):
    MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE),
    hplus(h),
    kbody(h.kbody),
    traj(t),
    cur_index(-1)

  {

    initWindowSize(640, 480);
    createWindow("Hubo Demo");
    setupBasicLight(vec4f(1,1,1,0));

    double bz = 0.85;

    camera.aim(vec3f(3, 0, bz),
               vec3f(0, 0, bz),
               vec3f(0, 0, 1));

    camera.setPerspective();

    camera.setRotateType(GlCamera::ROTATE_2_AXIS);

    camera.setHomePosition();

    quadric = gluNewQuadric();

    animating = false;

    kbody.compileDisplayLists();

    state.jvalues.resize(kbody.joints.size());

    setTimer(40, 0);
    
    resetCurrent();

  }

  void resetCurrent() {

    cur_index = 0; 
    stance_foot_xform = Transform3();
    stance_foot = stance_foot_table[traj[0].stance];
    setStateFromTraj(traj[0]);


  }
    
  
  
  void setStateFromTraj(const zmp_traj_element_t& cur) {
    
    for (size_t hi=0; hi<hplus.huboJointOrder.size(); ++hi) {
      size_t ji = hplus.huboJointOrder[hi];
      if (ji != size_t(-1)) {
      	state.jvalues[ji] = cur.angles[hi];
      }
    }
    
    kbody.transforms(state.jvalues, xforms);

    Transform3 foot_fk = kbody.manipulatorFK(xforms, stance_foot);
    state.setXform(stance_foot_xform * foot_fk.inverse());
    
    for (int a=0; a<3; ++a) {
      actualCom[a] = cur.com[a][0];
      actualComVel[a] = cur.com[a][1];
      actualComAcc[a] = cur.com[a][2];
      for (int f=0; f<2; ++f) {
	forces[1-f][a] = cur.forces[f][a];
	torques[1-f][a] = cur.torque[f][a]; // TODO: FIXME: pluralization WTF?
      }
    }

    std::cerr << "current index: " << cur_index << "/" << traj.size() << ", stance=" << cur.stance << "\n";  
    
  }

  virtual void deltaCurrent(int delta) {

    int dmin = -cur_index;
    int dmax = traj.size()-1-cur_index;
    delta = std::max(dmin, std::min(delta, dmax));
    if (!delta) { return; }

    int dd = delta < 0 ? -1 : 1;
    delta *= dd;
    assert(delta > 0);

    for (int i=0; i<delta; ++i) {

      cur_index += dd;

      // see if the stance foot has swapped
      int new_stance_foot = stance_foot_table[traj[cur_index].stance];

      if (new_stance_foot != stance_foot) {
        stance_foot = new_stance_foot;
        stance_foot_xform = state.xform() * kbody.manipulatorFK(xforms, stance_foot);
      }

      setStateFromTraj(traj[cur_index]);

    }

  }

  virtual void display() {

    MzGlutApp::display();

    glMatrixMode(GL_MODELVIEW);

    glColor3ub(0, 127, 127);
    glNormal3f(0, 0, 1);
    glBegin(GL_LINES);
    double s = 0.5;
    double n = 10;
    double x = s*n;
    for (int i=-n; i<=n; ++i) {
      glVertex2d(x, i*s);
      glVertex2d(-x, i*s);
      glVertex2d(i*s, x);
      glVertex2d(i*s, -x);
    }
    glEnd();

    glPushMatrix();
    glstuff::mult_transform(state.xform());

    hplus.render(xforms);
    glClear(GL_DEPTH_BUFFER_BIT);

    hplus.kbody.renderSkeleton(xforms, quadric);

    // Force and torque arrows
    for (int f=0; f<2; ++f) {
      Transform3 fk = hplus.kbody.manipulatorFK(xforms, f);
      glPushMatrix();
      glstuff::mult_transform(fk);
      glTranslated(0, 0, hplus.footAnkleDist);
      double fscl = 1.0/400;
      glColor3ub(255,255,0);
      glstuff::draw_arrow(quadric, vec3(0), fscl*forces[f], 0.02);
      glColor3ub(255,128,0);
      glstuff::draw_arrow(quadric, vec3(0), fscl*torques[f], 0.02);

      glPopMatrix();
    }

    glPopMatrix();


    glPushMatrix();
    glstuff::mult_transform(stance_foot_xform);

    // CoM sphere
    glColor3ub(255, 0, 255);
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], actualCom[2]+hplus.footAnkleDist);
    gluSphere(quadric, 0.05, 32, 24);
    glPopMatrix();

    // CoM projection disk
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], 0.01);
    glColor3ub(127, 0, 127);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glRotated(180, 1, 0, 0);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glPopMatrix();

    // Acceleration arrow
    double fscl = 1.0/2.0;
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], actualCom[2]+hplus.footAnkleDist);
    glColor3ub(0, 255, 0);
    glstuff::draw_arrow(quadric, vec3(0), fscl*actualComAcc, 0.02);
    glPopMatrix();
    
    
    glPopMatrix();

    glutSwapBuffers();

  }

  virtual void timer(int value) {

    if (animating) {
      if (cur_index+1 < traj.size()) { 
	deltaCurrent(5);
	glutPostRedisplay();
      } else {
	animating = false;
      }
    }

    setTimer(40, 0);    

  }

  virtual void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case '-':
      animating = false;
      deltaCurrent(-1);
      glutPostRedisplay();
      break;
    case '+':
    case '=':
      animating = false;
      deltaCurrent(1);
      glutPostRedisplay();
      break;
    case 'r':
      animating = false;
      resetCurrent();
      glutPostRedisplay();
      break;
    case '\r':
    case '\n':
      animating = !animating;
      break;
    default: 
      MzGlutApp::keyboard(key, x, y);
    }


    
  }

};

/**
* @function: validateCOMTraj(Eigen::MatrixXd& comX, Eigen::MatrixXd& comY) 
* @brief: validation of COM output trajectory data
* @return: void
*/
void validateCOMTraj(Eigen::MatrixXd& comX, Eigen::MatrixXd& comY) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double comVel, comAcc;
    Eigen::Matrix3d comStateDiffs;
    double comStateMaxes[] = {0.0, 0.0, 0.0};
    const double comStateTol[] = {2.0, 5.0}; // m/s, m/s^2, m/s^3
    for (int n=0; n<(comX.rows()-1); n++) {
      // Calculate COM vel and acc norms from x and y components
      comVel = sqrt((comX(n+1,0)-comX(n,0))*(comX(n+1,0)-comX(n,0)) + (comY(n+1,0)-comY(n,0))*(comY(n+1,0)-comY(n,0)))/dt;
      comAcc = sqrt((comX(n+1,1)-comX(n,1))*(comX(n+1,1)-comX(n,1)) + (comY(n+1,1)-comY(n,1))*(comY(n+1,1)-comY(n,1)))/dt;
      // Update max state values
      if (comVel > comStateMaxes[0]) comStateMaxes[0] = comVel;
      if (comAcc > comStateMaxes[1]) comStateMaxes[1] = comAcc;
      // Check if any are over limit
      if (comVel > comStateTol[0]) {
          std::cerr << "COM velocity sample " << n+1 << "is larger than " << comStateTol[0] << "(" << comVel << ")\n";
      }
      if (comAcc > comStateTol[1]) {
          std::cerr << "COM acceleration of sample " << n+1 << "is larger than " << comStateTol[1] << "(" << comAcc << ")\n";
      }
    }
    std::cerr << "comMaxVel: " << comStateMaxes[0]
              << "\ncomMaxAcc: " << comStateMaxes[1] << std::endl;


}
  
/**
* @function: validateOutputData(TrajVector& traj)
* @brief: validation of joint angle output trajectory data
* @return: void
*/
void validateOutputData(TrajVector& traj) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double maxJointVel=0;
    double jointVel;
    const double jointVelTol = 6.0; // radians/s
    for (int n=0; n<(int)(traj.size()-1); n++) {
      for (int j=0; j<HUBO_JOINT_COUNT; j++) {  
        jointVel = (traj[n+1].angles[j] - traj[n].angles[j])/dt;
        if (jointVel > jointVelTol) {
          std::cerr << "change in joint " << j << "is larger than " << jointVelTol << "(" << jointVel << ")\n";
        }
        if (jointVel > maxJointVel) maxJointVel = jointVel;
      }
    }
    std::cerr << "maxJntVel: " << maxJointVel << std::endl;
}


void usage(std::ostream& ostr) {
  ostr << 
    "usage: zmpdemo [OPTIONS] HUBOFILE.xml\n"
    "\n"
    "OPTIONS:\n"
    "\n"
    "  -g, --show-gui                    Show a GUI after computing trajectories.\n"
    "  -A, --use-ach                     Send trajectory via ACH after computing.\n"
    "  -I, --ik-errors                   IK error handling: strict/sloppy\n"
    "  -w, --walk-type                   Set type: canned/line/circle\n"
    "  -D, --walk-distance               Set maximum distance to walk\n"
    "  -r, --walk-circle-radius          Set radius for circle walking\n"
    "  -c, --max-step-count=NUMBER       Set maximum number of steps\n"
    "  -y, --foot-separation-y=NUMBER    Half-distance between feet\n"
    "  -z, --foot-liftoff-z=NUMBER       Vertical liftoff distance of swing foot\n"
    "  -l, --step-length=NUMBER          Max length of footstep\n"
    "  -S, --walk-sideways               Should we walk sideways? (canned gait only)\n"
    "  -h, --com-height=NUMBER           Height of the center of mass\n"
    "  -a, --comik-angle-weight=NUMBER   Angle weight for COM IK\n"
    "  -Y, --zmp-offset-y=NUMBER         Lateral distance from ankle to ZMP\n"
    "  -X, --zmp-offset-x=NUMBER         Forward distance from ankle to ZMP\n"
    "  -T, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
    "  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
    "  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
    "  -d, --double-support-time=NUMBER  Double support time\n"
    "  -s, --single-support-time=NUMBER  Single support time\n"
    "  -R, --zmp-jerk-penalty=NUMBER     R-value for ZMP preview controller\n"
    "  -H, --help                        See this message\n";
    
}

double getdouble(const char* str) {
  char* endptr;
  double d = strtod(str, &endptr);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    usage(std::cerr);
    exit(1);
  }
  return d;
}

long getlong(const char* str) {
  char* endptr;
  long d = strtol(str, &endptr, 10);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    usage(std::cerr);
    exit(1);
  }
  return d;
}

enum walktype {
  walk_canned,
  walk_line,
  walk_circle
};

walktype getwalktype(const std::string& s) {
  if (s == "canned") {
    return walk_canned;
  } else if (s == "line") {
    return walk_line;
  } else if (s == "circle") {
    return walk_circle;
  } else {
    std::cerr << "bad walk type " << s << "\n";
    usage(std::cerr);
    exit(1);
  }
}

ZMPWalkGenerator::ik_error_sensitivity getiksense(const std::string& s) {
  if (s == "strict") {
    return ZMPWalkGenerator::ik_strict;
  } else if (s == "sloppy") {
    return ZMPWalkGenerator::ik_sloppy;
  } else if (s == "permissive") {
    return ZMPWalkGenerator::ik_swing_permissive;
  } else {
    std::cerr << "bad ik error sensitivity " << s << "\n";
    usage(std::cerr);
    exit(1);
  }
}

int main(int argc, char** argv) {

  if (argc < 2) {
    usage(std::cerr);
    return 1;
  }


  bool show_gui = false;
  bool use_ach = false;

  walktype walk_type = walk_canned;
  double walk_circle_radius = 5.0;
  double walk_dist = 20;

  double footsep_y = 0.085; // half of horizontal separation distance between feet
  double foot_liftoff_z = 0.05; // foot liftoff height

  double step_length = 0.05;
  bool walk_sideways = false;

  double com_height = 0.48; // height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0;

  double lookahead_time = 2.5;

  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;

  size_t max_step_count = 4;

  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller

  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_strict;

  const struct option long_options[] = {
    { "show-gui",            no_argument,       0, 'g' },
    { "use-ach",             no_argument,       0, 'A' },
    { "ik-errors",           required_argument, 0, 'I' },
    { "walk-type",           required_argument, 0, 'w' },
    { "walk-distance",       required_argument, 0, 'D' },
    { "walk-circle-radius",  required_argument, 0, 'r' },
    { "max-step-count",      required_argument, 0, 'c' },
    { "foot-separation-y",   required_argument, 0, 'y' },
    { "foot-liftoff-z",      required_argument, 0, 'z' },
    { "step-length",         required_argument, 0, 'l' },
    { "walk-sideways",       no_argument,       0, 'S' },
    { "com-height",          required_argument, 0, 'h' },
    { "comik-angle-weight",  required_argument, 0, 'a' },
    { "zmp-offset-y",        required_argument, 0, 'Y' },
    { "zmp-offset-x",        required_argument, 0, 'X' },
    { "lookahead-time",      required_argument, 0, 'T' },
    { "startup-time",        required_argument, 0, 'p' },
    { "shutdown-time",       required_argument, 0, 'n' },
    { "double-support-time", required_argument, 0, 'd' },
    { "single-support-time", required_argument, 0, 's' },
    { "zmp-jerk-penalty",    required_argument, 0, 'R' },
    { "help",                no_argument,       0, 'H' },
    { 0,                     0,                 0,  0  },
  };

  const char* short_options = "gAI:w:D:r:c:y:z:l:Sh:a:Y:X:T:p:n:d:s:R:H";

  int opt, option_index;


  while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
    switch (opt) {
    case 'g': show_gui = true; break;
    case 'A': use_ach = true; break;
    case 'I': ik_sense = getiksense(optarg); break;
    case 'w': walk_type = getwalktype(optarg); break;
    case 'D': walk_dist = getdouble(optarg); break;
    case 'r': walk_circle_radius = getdouble(optarg); break;
    case 'c': max_step_count = getlong(optarg); break;
    case 'y': footsep_y = getdouble(optarg); break;
    case 'z': foot_liftoff_z = getdouble(optarg); break;
    case 'l': step_length = getdouble(optarg); break;
    case 'S': walk_sideways = true; break;
    case 'h': com_height = getdouble(optarg); break;
    case 'a': com_ik_ascl = getdouble(optarg); break;
    case 'Y': zmpoff_y = getdouble(optarg); break;
    case 'X': zmpoff_x = getdouble(optarg); break;
    case 'T': lookahead_time = getdouble(optarg); break;
    case 'p': startup_time = getdouble(optarg); break;
    case 'n': shutdown_time = getdouble(optarg); break;
    case 'd': double_support_time = getdouble(optarg); break;
    case 's': single_support_time = getdouble(optarg); break;
    case 'R': zmp_jerk_penalty = getdouble(optarg); break;
    case 'H': usage(std::cout); exit(0); break;
    default:  usage(std::cerr); exit(1); break;
    }
  }


  const char* hubofile = 0;
  while (optind < argc) {
    if (!hubofile) {
      hubofile = argv[optind++];
    } else {
      std::cerr << "Error: extra arguments on command line.\n\n";
      usage(std::cerr);
      exit(1);
    }
  }
  if (!hubofile) { 
    std::cerr << "Please supply a huboplus file!\n\n";
    usage(std::cerr); 
    exit(1); 
  }

  HuboPlus hplus(hubofile);


  //////////////////////////////////////////////////////////////////////
  // build initial state

  // the actual state
  ZMPWalkGenerator walker(hplus,
			  ik_sense,
                          com_height,
                          zmp_jerk_penalty,
			  zmpoff_x,
			  zmpoff_y,
                          com_ik_ascl,
                          single_support_time,
                          double_support_time,
                          startup_time,
                          shutdown_time,
                          foot_liftoff_z,
			  lookahead_time
    );
  ZMPReferenceContext initContext;

  // helper variables and classes
  const KinBody& kbody = hplus.kbody;
  const JointLookup& jl = hplus.jl;
  double deg = M_PI/180; // for converting from degrees to radians

  // fill in the kstate
  initContext.state.body_pos = vec3(0, 0, 0.85);
  initContext.state.body_rot = quat();
  initContext.state.jvalues.resize(kbody.joints.size(), 0.0);
  initContext.state.jvalues[jl("LSR")] =  15*deg;
  initContext.state.jvalues[jl("RSR")] = -15*deg;
  initContext.state.jvalues[jl("LSP")] =  20*deg;
  initContext.state.jvalues[jl("RSP")] =  20*deg;
  initContext.state.jvalues[jl("LEP")] = -40*deg;
  initContext.state.jvalues[jl("REP")] = -40*deg;
  
  // build and fill in the initial foot positions

  Transform3 starting_location(quat::fromAxisAngle(vec3(0,0,1), 0));
  initContext.feet[0] = Transform3(starting_location.rotation(), starting_location * vec3(0, footsep_y, 0));
  initContext.feet[1] = Transform3(starting_location.rotation(), starting_location * vec3(0, -footsep_y, 0));

  // fill in the rest
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(zmpoff_x, 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(0.0, 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = 0.0;
  initContext.pY = 0.0;

  // apply COM IK for init context
  walker.applyComIK(initContext);

  /*
  walker.traj.resize(1);
  walker.refToTraj(initContext, walker.traj.back());
  */


  walker.initialize(initContext);

  
  //////////////////////////////////////////////////////////////////////
  // build ourselves some footprints
  
  Footprint initLeftFoot = Footprint(initContext.feet[0], true);
  Footprint initRightFoot = Footprint(initContext.feet[1], false);

  std::vector<Footprint> footprints;

  switch (walk_type) {
  case walk_circle: {

    double circle_max_step_angle = M_PI / 12.0; // maximum angle between steps TODO: FIXME: add to cmd line??
  
    footprints = walkCircle(walk_circle_radius,
			    walk_dist,
			    footsep_y,
			    step_length,
			    circle_max_step_angle,
			    &initLeftFoot,
			    &initRightFoot,
			    false);

    break;

  }

  case walk_line: {

    footprints = walkLine(walk_dist, footsep_y,
			  step_length,
			  &initLeftFoot,
			  &initRightFoot,
			  false);

    break;

  }

  default: {

    double cur_x[2] = { 0, 0 };
    double cur_y[2] = { 0, 0 };

    cur_y[0] =  footsep_y;
    cur_y[1] = -footsep_y;
    
    for (size_t i=0; i<max_step_count; ++i) {
      bool is_left = i%2;
      if (walk_sideways && step_length < 0) { is_left = !is_left; }
      int swing = is_left ? 0 : 1;
      int stance = 1-swing;
      if (walk_sideways) {
	cur_y[swing] -= step_length;
      } else {
	if (i + 1 == max_step_count) {
	  cur_x[swing] = cur_x[stance];
	} else {
	  cur_x[swing] = cur_x[stance] + 0.5*step_length;
	}
      }
      footprints.push_back(Footprint(cur_x[swing], cur_y[swing], 0, is_left));
    }

    break;

  }
  }

  if (footprints.size() > max_step_count) {
    footprints.resize(max_step_count);
  }

  //////////////////////////////////////////////////////////////////////
  // and then build up the walker


  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);


  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }



  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);
  

  //////////////////////////////////////////////////////////////////////
  // have the walker run preview control and pass on the output

  walker.bakeIt();
  // validateOutputData(traj);

#ifdef HAVE_HUBO_ACH

  if (use_ach) {
    ach_channel_t zmp_chan;
    ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );

    zmp_traj_t trajectory;
    memset( &trajectory, 0, sizeof(trajectory) );

    int N;
    if( (int)walker.traj.size() > MAX_TRAJ_SIZE )
      N = MAX_TRAJ_SIZE;
    else
      N = (int)walker.traj.size();

    trajectory.count = N;
    for(int i=0; i<N; i++)
      memcpy( &(trajectory.traj[i]), &(walker.traj[i]), sizeof(zmp_traj_element_t) );

    ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
    fprintf(stdout, "Message put\n");
  }

#endif

  if (show_gui) {

    ZmpDemo demo(argc, argv, hplus, walker.traj);

    demo.run();

  }


  return 0;
}


// Local Variables:
// c-basic-offset: 2
// End:
