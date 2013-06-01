/**
 * @file drchubo_AnimationPush
 */
#include <map>
#include "gazebo.hh"
#include "common/common.hh"
#include "physics/physics.hh"
#include "../../src/utils/data_paths.h"


std::string gJointNames[35] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
			       "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
			       "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
			       "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
			       "drchubo::TSY", "drchubo::NKY", "drchubo::NKP",
			       "drchubo::LF1", "drchubo::LF2", "drchubo::LF3",
			       "drchubo::RF1", "drchubo::RF2", "drchubo::RF3"};
int gNumJoints = 35;

namespace gazebo
{
  /**
   * @class drchubo_AnimationPush
   */
  class drchubo_AnimationPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      printf( "*** Loading animation *** \n" );

      // Set gravity to false, otherwise the robot slowly falls
      _model->SetGravityMode( false );

      std::map<std::string, common::NumericAnimationPtr> joint_anim;
      common::NumericKeyFrame * joint_key; 


      // Fill joint initialization
      double T = 3.0;
      int numTrajPoints = 500;

      for( int i = 0; i < gNumJoints; ++i ) {
	joint_anim[ gJointNames[i] ].reset( new common::NumericAnimation( "anim", T, true ) );	
      }


      // Start storing
      double t = 0;
      double dt = T / (double) numTrajPoints;
      for( int i = 0; i < numTrajPoints; ++i ) {

	for( int j = 0; j < gNumJoints; ++j ) {
	  joint_key = joint_anim[ gJointNames[j] ]->CreateKeyFrame(t);
	  if( j == 0 ) { // LSP (raise arm)
	    joint_key->SetValue(-1.0 ); 
	  }
	  else if( j == 1 ) { // LSR (away from body)
	    joint_key->SetValue( 0.35 );
	  }
	  else if( j == 2 ) { // LSY (turn forearm towards inside)
	    joint_key->SetValue( -1.5 );
	  }
	  else if( j == 3 ) { // LEP
	    joint_key->SetValue( -1.57*( (double)i/(double)numTrajPoints ) );
	  }
	  else {
	    joint_key->SetValue(0);
	  }
	}

	t += dt;
      }

	
      // Attach the animation to the model
      _model->SetJointAnimation( joint_anim );
      printf("*** End loading animation *** \n");

      //************************
      // WORLD ANIMATION  	
      //************************
      gazebo::common::PoseAnimationPtr pose_anim( new gazebo::common::PoseAnimation("test", T, true) );
      gazebo::common::PoseKeyFrame *pose_key;

      t = 0;

      for( int i = 0; i < numTrajPoints; ++i ) {

	pose_key = pose_anim->CreateKeyFrame(t);

	pose_key->SetTranslation( math::Vector3( 0.0, 0, 0.95) );
	pose_key->SetRotation(math::Quaternion(0, 0, 0.0));

	// Advance one time step
	t+=dt;
      }
      
      _model->SetAnimation( pose_anim );      
      printf( "** End loading Pose animation **\n" );
    
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(drchubo_AnimationPush)
}
