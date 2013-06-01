#include "src/ZmpPreview.h"
#include "src/HuboPlus.h"
#include "zmp/zmpwalkgenerator.h"
#include "zmp/gait-timer.h"
#include "zmp/swing.h"

ZMPWalkGenerator::ZMPWalkGenerator(HuboPlus& _hplus,
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
				   double lookahead_time) :
    hplus(_hplus),
    ik_sense(ik_sense),
    com_height(com_height),
    zmp_R(zmp_R),
    zmpoff_x(zmpoff_x),
    zmpoff_y(zmpoff_y),
    com_ik_angle_weight(com_ik_angle_weight),
    min_single_support_time(min_single_support_time),
    min_double_support_time(min_double_support_time),
    walk_startup_time(walk_startup_time),
    walk_shutdown_time(walk_shutdown_time),
    step_height(step_height),
    lookahead_time(lookahead_time),
    haveInitContext(false)
{
}

// these all modify the current context but do not immediately affect traj

void ZMPWalkGenerator::initialize(const ZMPReferenceContext& current) {
    ref.clear();
    traj.clear();
    first_step_index = -1;
    initContext = current;
    haveInitContext = true; // TODO: FIXME: CHECK THIS IN addFootstep and stayDogStay
}

/**
 * @function: ZMPReferenceContext& getLastRef()
 * @brief: gets the last ZMPReferenceContext from ref or the initContext
 */
const ZMPReferenceContext& ZMPWalkGenerator::getLastRef() {
    return ref.empty() ? initContext : ref.back();
}


/**
 * @function: stayDogStay(size_t stay_ticks)
 * @brief: these will add walk contexts to the back of ref and the new
 *         contexts don't have comX, comY, eX, eY however, the kstate will
 *         have body orientation set correctly and upper body joints
 * @return: void
 */
void ZMPWalkGenerator::stayDogStay(size_t stay_ticks) {

  ZMPReferenceContext cur = getLastRef();

  vec3 zmp_start(cur.pX, cur.pY, 0);
  
  Transform3 midfoot( quat::slerp(cur.feet[0].rotation(),
				  cur.feet[1].rotation(),
				  0.05),
		      0.5 * (cur.feet[0].translation() +
			     cur.feet[1].translation() ) );
		      

  vec3 zmp_end = midfoot * vec3(zmpoff_x, zmpoff_y, 0);


  stance_t double_stance = DOUBLE_LEFT;

  // TODO: get from timer!!
  size_t shift_ticks = TRAJ_FREQ_HZ * min_double_support_time;
  if (shift_ticks > stay_ticks) { shift_ticks = stay_ticks; }

  for (size_t i=0; i<shift_ticks; ++i) {

    double u = double(i) / double(shift_ticks - 1);
    double c = sigmoid(u);

    cur.stance = double_stance;

    vec3 cur_zmp = zmp_start + (zmp_end - zmp_start) * c;

    cur.pX = cur_zmp.x();
    cur.pY = cur_zmp.y();

    ref.push_back(cur);

  }

  for (size_t i=shift_ticks; i<stay_ticks; ++i) {
    ref.push_back(cur);
  }

}


double ZMPWalkGenerator::sigmoid(double x) {
    return 3*x*x - 2*x*x*x;
}


void ZMPWalkGenerator::addFootstep(const Footprint& fp) {
    // initialize our timer
    GaitTimer timer;
    timer.single_support_time = min_single_support_time;
    timer.double_support_time = min_double_support_time;
    timer.startup_time = walk_startup_time;
    timer.startup_time = walk_shutdown_time;

    // grab the initial position of the zmp
    const ZMPReferenceContext start_context = getLastRef();

    // figure out the stances for this movement
    stance_t double_stance = fp.is_left ? DOUBLE_RIGHT : DOUBLE_LEFT;
    stance_t single_stance = fp.is_left ? SINGLE_RIGHT : SINGLE_LEFT;
    if (step_height == 0) { single_stance = double_stance; }

    // figure out swing foot and stance foot for accessing
    int swing_foot = fp.is_left ? 0 : 1;
    int stance_foot = 1-swing_foot;

    // figure out where our body is going to end up if we put our foot there
    quat body_rot_end = quat::slerp(start_context.feet[swing_foot].rotation(),
                                    fp.transform.rotation(),
                                    0.5);

    // figure out the start and end positions of the zmp
    vec3 zmp_end = start_context.feet[stance_foot] * vec3(zmpoff_x, zmpoff_y, 0);
    vec3 zmp_start = vec3(start_context.pX, start_context.pY, 0);


    // figure out how far the swing foot will be moving
    double dist = (fp.transform.translation() - start_context.feet[swing_foot].translation()).norm();

    // turns out that extracting plane angles from 3d rotations is a bit annoying. oh well.
    vec3 rotation_intermediate =
        fp.transform.rotFwd() * vec3(1.0, 0.0, 0.0) +
        start_context.feet[swing_foot].rotFwd() * vec3(1.0, 0.0, 0.0);
    double dist_theta = atan2(rotation_intermediate.y(), rotation_intermediate.x()); // hooray for bad code!

    // figure out how long we spend in each stage
    // TODO:
    //  dist for double support should be distance ZMP moves
    //  no theta or step height needed for double support
    //
    //  dist for single support should be distance SWING FOOT moves
    //  that also includes change in theta, and step height
    size_t double_ticks = timer.compute_double(dist);
    size_t single_ticks = timer.compute_single(dist, dist_theta, step_height);

    //size_t double_ticks = TRAJ_FREQ_HZ * min_double_support_time;
    //size_t single_ticks = TRAJ_FREQ_HZ * min_single_support_time;

    for (size_t i = 0; i < double_ticks; i++) {
        // sigmoidally interopolate things like desired ZMP and body
        // rotation. we run the sigmoid across both double and isngle
        // support so we don't try to whip the body across for the
        // split-second we're in double support.
        double u = double(i) / double(double_ticks - 1);
        double c = sigmoid(u);

	double uu = double(i) / double(double_ticks + single_ticks - 1);
	double cc = sigmoid(uu);
        
        ZMPReferenceContext cur_context = start_context;
        cur_context.stance = double_stance;


        vec3 cur_zmp = zmp_start + (zmp_end - zmp_start) * c;
        cur_context.pX = cur_zmp.x();
        cur_context.pY = cur_zmp.y();

        cur_context.state.body_rot = quat::slerp(start_context.state.body_rot, 
                                                 body_rot_end, cc);


        ref.push_back(cur_context);
    }

    double swing_foot_traj[single_ticks][3];
    double swing_foot_angle[single_ticks];

    // note: i'm not using the swing_foot_angle stuff cause it seemed to not work
    swing2Cycloids(start_context.feet[swing_foot].translation().x(),
                   start_context.feet[swing_foot].translation().y(),
                   start_context.feet[swing_foot].translation().z(),
                   fp.transform.translation().x(),
                   fp.transform.translation().y(),
                   fp.transform.translation().z(),
                   single_ticks,
                   fp.is_left,
                   step_height,
                   swing_foot_traj,
                   swing_foot_angle);

    
    quat foot_start_rot = start_context.feet[swing_foot].rotation();
    quat foot_end_rot = fp.transform.rotation();

    // we want to have a small deadband at the front and end when we rotate the foot around
    // so it has no rotational velocity during takeoff and landing
    size_t rot_dead_ticks = 0.1 * TRAJ_FREQ_HZ; 

    if (single_ticks < 4*rot_dead_ticks) {
      rot_dead_ticks = single_ticks / 4;
    }
    
    assert(single_ticks > rot_dead_ticks * 2);

    size_t central_ticks = single_ticks - 2*rot_dead_ticks;

    for (size_t i = 0; i < single_ticks; i++) {

	double uu = double(i + double_ticks) / double(double_ticks + single_ticks - 1);
	double cc = sigmoid(uu);

	double ru;

	if (i < rot_dead_ticks) {
	  ru = 0;
	} else if (i < rot_dead_ticks + central_ticks) {
	  ru = double(i - rot_dead_ticks) / double(central_ticks - 1);
	} else {
	  ru = 1;
	}


        ref.push_back(getLastRef());
        ZMPReferenceContext& cur_context = ref.back();
        cur_context.stance = single_stance;

        cur_context.state.body_rot = quat::slerp(start_context.state.body_rot, 
                                                 body_rot_end, cc);


	cur_context.feet[swing_foot].setRotation(quat::slerp(foot_start_rot, 
							     foot_end_rot,
							     ru));

	cur_context.feet[swing_foot].setTranslation(vec3(swing_foot_traj[i][0],
							 swing_foot_traj[i][1],
							 swing_foot_traj[i][2]));
    }

    // finally, update the first step variable if necessary
    if (first_step_index == size_t(-1)) { // we haven't taken a first step yet
        first_step_index = ref.size() - 1;
    }
}


/**
 * @function: bakeIt()
 * @brief: clears a trajectory, runs ZMP Preview Controller
 *         run COM IK, dumps out trajectory.
 * @return: void
 */
void ZMPWalkGenerator::bakeIt() {
    traj.clear();
    runZMPPreview();
    runCOMIK();
    dumpTraj();

    // TODO: for thinkin ahead
    //initContext = ref[first_step_index];

    ref.clear();
    haveInitContext = false;

}

/** @function: runZMPPreview()
 * @brief: run ZMP preview controller on entire reference and creates trajectory for COM pos/vel/acc in X and Y
 * @precondition: we have zmp reference values for x and y, initContext com and integrator error
 *               for initialization.
 * @postcondition: now we have set comX, comY, eX, eY for everything in ref.
 * @return: void
 */
void ZMPWalkGenerator::runZMPPreview() {

    Eigen::Vector3d comX = initContext.comX; // initialize comX states to initial ZMPReferenceContext
    Eigen::Vector3d comY = initContext.comY; // initialize comY states to initial ZMPReferenceContext
    double eX = initContext.eX; //
    double eY = initContext.eY; //
    Eigen::ArrayXd zmprefX(ref.size());
    Eigen::ArrayXd zmprefY(ref.size());

    // initialize the zmp preview controller
    ZmpPreview preview(1.0/TRAJ_FREQ_HZ, com_height, lookahead_time*TRAJ_FREQ_HZ, zmp_R);

    // put all the zmp refs into eigen arrays in order to pass into the preview controller
    for(size_t i=0; i<ref.size(); i++) {
        zmprefX(i) = ref[i].pX;
        zmprefY(i) = ref[i].pY;
    }


    // generate COM position for each tick using zmp preview update
    for(size_t i = 0; i < ref.size(); i++) {
        ZMPReferenceContext& cur = ref[i];
        // run zmp preview controller to update COM states and integrator error
        preview.update(comX, eX, zmprefX.block(i, 0, ref.size()-i, 1));
	preview.update(comY, eY, zmprefY.block(i, 0, ref.size()-i, 1));
        cur.comX = comX; // set the ref comX pos/vel/acc for this tick
        cur.comY = comY; // set the ref comY pos/vel/acc for this tick
        cur.eX = eX;                 // set the X error for this tick
        cur.eY = eY;                 // set the Y error for this tick
    }

}

/**
 * @function: runCOMIK()
 * @brief: this runs the COM IK on every dang thing in reference to fill in the kstate
 * @precondition: reference is fully filled in
 * @postcondition: kstate is fully filled in
 * @return: void
 */
void ZMPWalkGenerator::runCOMIK() {

    for(std::vector<ZMPReferenceContext>::iterator cur = ref.begin(); cur != ref.end(); cur++) {

      applyComIK(*cur);

    }
}

void ZMPWalkGenerator::applyComIK(ZMPReferenceContext& cur) {

  Transform3Array body_transforms;

  // set up target positions
  Transform3 desired[4] = {cur.feet[0], cur.feet[1], Transform3(), Transform3()};
  vec3 desiredCom = vec3(cur.comX[0], cur.comY[0], com_height + hplus.footAnkleDist);
  // My guess, and why I changed it from addition to
  // subtraction: the bottom of the foot is footAnkleDist away
  // from the "end" of the end effector, so to get our actual
  // desired com_height we have to set our desired end effector
  // position a bit closer to the com than we would otherwise.
  // TODO: this needs to be checked by someone that actually
  // knows what they're talking about.

  // set up mode
  // single right is the only time the left foot is raised and vice versa
  // TODO: find a way to cut out the HuboPlus junk, it's unsightly
  cur.ikMode[HuboPlus::MANIP_L_FOOT] = (cur.stance == SINGLE_RIGHT) ? HuboPlus::IK_MODE_WORLD : HuboPlus::IK_MODE_SUPPORT;
  cur.ikMode[HuboPlus::MANIP_R_FOOT] = (cur.stance == SINGLE_LEFT) ? HuboPlus::IK_MODE_WORLD : HuboPlus::IK_MODE_SUPPORT;
  cur.ikMode[HuboPlus::MANIP_L_HAND] = HuboPlus::IK_MODE_FIXED;
  cur.ikMode[HuboPlus::MANIP_R_HAND] = HuboPlus::IK_MODE_FIXED;

  // only moving left foot in current code

  // set up ikvalid
  bool ikvalid[4];

  // and run IK. Everything we need goes straight into cur.state! cool.
  bool ok = hplus.comIK(cur.state,
			desiredCom,
			desired,
			cur.ikMode,
			HuboPlus::noGlobalIK(),
			body_transforms,
			com_ik_angle_weight,
			0,
			ikvalid);

  // TODO freak out if not ok
  if (ik_sense != ik_sloppy && !ok) {

    hplus.kbody.transforms(initContext.state.jvalues, body_transforms);
    Transform3 actual[4];
    vec3 dp[4], dq[4];
    hplus.kbody.transforms(cur.state.jvalues, body_transforms);
    vec3 actualCom = cur.state.xform() * hplus.kbody.com(body_transforms);

    bool permissive_ok = (actualCom - desiredCom).norm() < hplus.DEFAULT_COM_PTOL;

    // loop through legs and arms to calculate results
    for (int i=0; i<4; ++i) {

      // are we in world or supporting?
      if (cur.ikMode[i] != HuboPlus::IK_MODE_FIXED &&
	  cur.ikMode[i] != HuboPlus::IK_MODE_FREE) {

	// get the FK for the manipulator
	actual[i] = hplus.kbody.manipulatorFK(body_transforms, i);

	// if relative to world, offset
	if (cur.ikMode[i] == HuboPlus::IK_MODE_WORLD ||
	    cur.ikMode[i] == HuboPlus::IK_MODE_SUPPORT) {
	  actual[i] = cur.state.xform() * actual[i];
	}

	// compute desired - actual
	deltaTransform(desired[i], actual[i], dp[i], dq[i]);

	if (!ikvalid[i]) {
	  // TODO: hacky - this assumes walking on flat ground
	  // TODO: hacky - this hardcodes distance tolerances

	  if ( ! ( ik_sense == ik_swing_permissive &&
		   cur.ikMode[i] == HuboPlus::IK_MODE_WORLD &&
		   desired[i].translation().z() > 0.03 &&
		   dp[i].norm() < 0.002 &&
		   dq[i].norm() < 4 * M_PI/180) ) {
	    permissive_ok = false;
	  }
	}

      }
    }

    if (ik_sense == ik_swing_permissive) {
      ok = permissive_ok;
    }

    if (!ok) {

      std::cerr << "IK FAILURE!\n\n";
      std::cerr << "  body:        " << cur.state.xform() << "\n";
      std::cerr << "  desired com: " << desiredCom << "\n";
      std::cerr << "  actual com:  " << actualCom << "\n\n";
      for (int i=0; i<4; ++i) {
	if (cur.ikMode[i] != HuboPlus::IK_MODE_FIXED &&
	    cur.ikMode[i] != HuboPlus::IK_MODE_FREE) {
	  std::cerr << "  " << hplus.kbody.manipulators[i].name << ":\n";
	  std::cerr << "    mode:    " << HuboPlus::ikModeString(cur.ikMode[i]) << "\n";
	  std::cerr << "    valid:   " << ikvalid[i] << "\n";
	  std::cerr << "    desired: " << desired[i] << "\n";
	  std::cerr << "    actual:  " << actual[i] << "\n";
	  std::cerr << "    dp:      " << dp[i] << " with norm " << dp[i].norm() << "\n";
	  std::cerr << "    dq:      " << dq[i] << " with norm " << dq[i].norm() << "\n";
	  std::cerr << "\n";
	}
      }
      exit(1);

    }

  } // finish freaking out

  // and we're done!

}

/**
 * @function: dumpTraj()
 * @brief: picks everything important out of ref which is now fully specified and creates a trajectory
 * @precondition: we have ref fully filled in
 * @postcondition: forces and torque are calculated and everything is transformed into stance ankle reference frame
 * @return: void
 */
void ZMPWalkGenerator::dumpTraj() {
  for(std::vector<ZMPReferenceContext>::iterator cur_ref = ref.begin(); cur_ref != ref.end(); cur_ref++) {
    // make output
    traj.push_back(zmp_traj_element_t());
    refToTraj(*cur_ref, traj.back());
  }
}

void ZMPWalkGenerator::refToTraj(const ZMPReferenceContext& cur_ref,
				 zmp_traj_element_t& cur_out) {



  // copy stance into output
  cur_out.stance = cur_ref.stance;

  // copy joint angles from reference to output
  for (size_t phys_i=0; phys_i < hplus.huboJointOrder.size(); phys_i++) {
    // map simulation joint number to physical joint number
    size_t sim_i = hplus.huboJointOrder[phys_i];
    if (sim_i != size_t(-1)) { // if it's valid, copy
      cur_out.angles[phys_i] = cur_ref.state.jvalues[sim_i];
    }
  }

  // compute expected forces and torques and copy into output
  vec3 forces[2];
  vec3 torques[2];
  Transform3 desired[4] = {cur_ref.feet[0], cur_ref.feet[1], Transform3(), Transform3()};
  // compute ground reaction forces and torques
  hplus.computeGroundReaction( vec3(cur_ref.comX(0), cur_ref.comY(0), com_height), // com position, we hope
			       vec3(cur_ref.comX(2), cur_ref.comY(2), 0), // com acceleration
			       desired, // foot locations
			       cur_ref.ikMode, // foot modes
			       forces,  // forces output
			       torques); // torques output
  for (size_t axis=0; axis<3; axis++) {  // here's one of the places we have to swap handedness
    cur_out.forces[1][axis] = forces[0][axis];  // left foot forces
    cur_out.torque[1][axis] = torques[0][axis]; // left foot torques
    cur_out.forces[0][axis] = forces[1][axis]; // right foot forces
    cur_out.torque[0][axis] = torques[1][axis]; // right foot torques
  }


  // transform zmp and com into stance ankle reference frame, copy into output
  Transform3 stance_foot_trans = cur_ref.stance == SINGLE_LEFT || cur_ref.stance == DOUBLE_LEFT
    ? cur_ref.feet[0] : cur_ref.feet[1];
  vec3 zmp_in_world(cur_ref.pX, cur_ref.pY, 0);
  vec3 zmp_in_stance = stance_foot_trans.transformInv(zmp_in_world);

  cur_out.zmp[0] = zmp_in_stance.x();
  cur_out.zmp[1] = zmp_in_stance.y();

  for (size_t deriv = 0; deriv < 3; deriv++) {

    vec3 com_in_world(cur_ref.comX[deriv], cur_ref.comY[deriv], deriv == 0 ? com_height : 0);

    vec3 com_in_stance = ( deriv == 0 ?
			   stance_foot_trans.transformInv(com_in_world) :
			   stance_foot_trans.rotInv() * com_in_world );

    cur_out.com[0][deriv] = com_in_stance.x();
    cur_out.com[1][deriv] = com_in_stance.y();
    cur_out.com[2][deriv] = com_in_stance.z();

  }



}
