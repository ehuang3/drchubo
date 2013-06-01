#include "math.h"
#include "mzcommon/Transform3.h"
#include "zmp/footprint.h"
#include "src/fakerave.h"
#include "zmp/hubo-zmp.h"
#include "zmp/gait-timer.h"
#include "src/HuboPlus.h"





using namespace fakerave;

class ZMPReferenceContext {
public:
    stance_t stance; // single/double, left/right
    HuboPlus::IKMode ikMode[4]; // current IK settings for each limb

    Transform3 feet[2]; // or footprint i dont care
    Eigen::Vector3d comX, comY; // pos/vel/accel of each 
    double eX, eY; // integrator error for zmp controller
    double pX, pY; // where is the desired ZMP right now?
    
    HuboPlus::KState state; // complete state of the robot
};



class ZMPWalkGenerator {
public:
    
  // TODO: shove these into struct
  
  enum ik_error_sensitivity {
    ik_strict, // default
    ik_swing_permissive, // allows ik errors on swing foot when above 0.5 * step_height
    ik_sloppy, // never ever ever ever run this on the robot
    
  };

    ZMPWalkGenerator(HuboPlus& _hplus,
		     ik_error_sensitivity ik_sense,
                     double com_height,
                     double zmp_R,
		     double zmpoff_x,
		     double zmpoff_y,
                     double com_ik_angle_weight,
                     double min_single_support_time,
                     double min_double_support_time,
                     double walk_startup_time,
                     double walk_shutdown_time,
                     double step_height,
		     double lookahead_time
        );
    
    const HuboPlus& hplus;

    ik_error_sensitivity ik_sense;
    double com_height;
    double zmp_R; // jerk penalty on ZMP controller
    double zmpoff_x;
    double zmpoff_y;
    double com_ik_angle_weight;
    double min_single_support_time;
    double min_double_support_time;
    double walk_startup_time;
    double walk_shutdown_time;
    double step_height;
    double lookahead_time;
    // tons of constants

    size_t first_step_index;
    
    bool haveInitContext;
    ZMPReferenceContext initContext; 
    std::vector<ZMPReferenceContext> ref;

    std::vector<zmp_traj_element_t> traj; // the entire fullbody trajectory so far
    
    // INVARIANT: traj.back() agrees 100% with current




    // these all modify the current context but do not immediately affect traj
    void initialize(const ZMPReferenceContext& current);
    const ZMPReferenceContext& getLastRef();
    // these will add walk contexts to the back of ref and the new
    // contexts don't have comX, comY, eX, eY however, the kstate will
    // have body orientation set correctly and upper body joints
    void stayDogStay(size_t stay_ticks);
    void addFootstep(const Footprint& fp);
    void bakeIt();
  
    void applyComIK(ZMPReferenceContext& ref);
    void refToTraj(const ZMPReferenceContext& ref,
		   zmp_traj_element_t& traj);


    
    // helper
    double sigmoid(double x);
    
    // this runs the ZMP preview controller on the entire reference
    // trajectory to fill in comX, comY, eX, eY for every dang thing.
    void runZMPPreview();
    // this runs the COM IK on every dang thing in reference to fill
    // in the kstate

    void runCOMIK();
    // this dumps everything into traj
    void dumpTraj();
};

// Local Variables:
// mode: c++
// End:
