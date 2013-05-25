/**
 * @file drchubo_AnimationPush
 */
#include <map>
#include "gazebo.hh"
#include "common/common.hh"
#include "physics/physics.hh"
#include "../../src/utils/data_paths.h"


std::string gJointNames[26] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
			       "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
			       "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
			       "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR"};
int gNumJoints = 26;

namespace gazebo
{
  /**
   * @class drchubo_Animation
   */
  class drchubo_Animation : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      printf("Loading animation \n");
      _model->SetGravityMode( false );
      std::map<std::string, common::NumericAnimationPtr> anim;
      double T = 10.0;
      int numPoints = 500;
      double dt = T / (double) numPoints;
      // For testing we only will use shoulder pitch and yaw
      anim["drchubo::LSR"].reset( new common::NumericAnimation( "anim", T, true) );
      //anim["drchubo::LSY"].reset( new common::NumericAnimation( "anim", T, true) );

      // Start storing
      common::NumericKeyFrame * key; 
      double t = 0;
//      for( int i = 0; i < numPoints; ++i ) {

	// Start with arm raised
	key = anim[ "drchubo::LSR" ]->CreateKeyFrame(0);
	key->SetValue( 0.2 );	  

	key = anim[ "drchubo::LSR" ]->CreateKeyFrame(T);
//	double x = ((double)i/(double)numPoints)*0.6;
	key->SetValue( 0.6 );	  
	t += dt;
//      }



	
      // Attach the animation to the model
      _model->SetJointAnimation( anim );
      printf("End loading animation \n");

      /////////////////////////////////
      // WORLD ANIMATION  		
      gazebo::common::PoseAnimationPtr anim2(new gazebo::common::PoseAnimation("test", T, true));

      gazebo::common::PoseKeyFrame *key2;

      key2 = anim2->CreateKeyFrame(0);
      key2->SetTranslation(math::Vector3( 0, 0, 1.25));
      key2->SetRotation(math::Quaternion(0, 0, 0.0));

      key2 = anim2->CreateKeyFrame(5.0);
      key2->SetTranslation(math::Vector3( 0.0, 0.0, 1.25));
      key2->SetRotation(math::Quaternion(0, 0, 0, 0.0));

      key2 = anim2->CreateKeyFrame(10.0);
      key2->SetTranslation(math::Vector3( 0.0, 0.0, 1.25));
      key2->SetRotation(math::Quaternion(0, 0, 0, 0.0));



      _model->SetAnimation(anim2);

	////////////////////




    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(drchubo_Animation)
}
