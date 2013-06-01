/**
 * @file drchubo_Animation.cpp
 */
#include <map>
#include "gazebo.hh"
#include "common/common.hh"
#include "physics/physics.hh"
#include "../../src/utils/data_paths.h"

std::string gJointNames[28] = {"drchubo::LSP", "drchubo::LSR", "drchubo::LSY", "drchubo::LEP", "drchubo::LWY", "drchubo::LWP", "drchubo::LWR",
			       "drchubo::RSP", "drchubo::RSR", "drchubo::RSY", "drchubo::REP", "drchubo::RWY", "drchubo::RWP", "drchubo::RWR",
			       "drchubo::LHY", "drchubo::LHR", "drchubo::LHP", "drchubo::LKP", "drchubo::LAP", "drchubo::LAR",
			       "drchubo::RHY", "drchubo::RHR", "drchubo::RHP", "drchubo::RKP", "drchubo::RAP", "drchubo::RAR",
			       "drchubo::NKY", "drchubo::NKP"};
int gNumJoints = 28;

namespace gazebo
{
  /**
   * @class drchubo_Animation
   */
  class drchubo_Animation : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
    {
      printf( "*** Loading animation ***\n" );

      // Set gravity to false, otherwise the robot slowly falls
      _model->SetGravityMode( false );
      std::map<std::string, common::NumericAnimationPtr> joint_anim;
      common::NumericKeyFrame *joint_key;

      // Fill joint initialization
      double T = 25.0;
      for( int i = 0; i < gNumJoints; ++i ) {
	joint_anim[ gJointNames[i] ].reset( new common::NumericAnimation( "anim", T, true) );
      }

      //**********************************
      // Read file with trajectories
      //**********************************
      float ang[gNumJoints]; int ind;
      int numTrajPoints = 4900;
      FILE* pFile;

      pFile = fopen(VRC_DATA_PATH "trajs/traj.txt", "r");

      if( pFile == NULL ) {
	printf("Did not find file, exiting! \n");
	return;
      }

      double t;
      double dt = T / (double)numTrajPoints;

      t = 0;
      for( int i = 0; i < numTrajPoints; ++i ) {
	
	fscanf( pFile, "%d %f %f %f %f %f %f", &ind, &ang[0], &ang[1], &ang[2], &ang[3], &ang[4], &ang[5] ); 
	ang[6] = 0.0;
	fscanf( pFile, "%f %f %f %f %f %f", &ang[7], &ang[8], &ang[9], &ang[10], &ang[11], &ang[12] );
	ang[13] = 0.0;
	fscanf( pFile, "%f %f %f %f %f %f", &ang[14], &ang[15], &ang[16], &ang[17], &ang[18], &ang[19] );
	fscanf( pFile, "%f %f %f %f %f %f \n", &ang[20], &ang[21], &ang[22], &ang[23], &ang[24], &ang[25] );
	
	// Create a key frame for each trajectory point read
	for( int j = 0; j < gNumJoints; ++j ) {
	  joint_key = joint_anim[ gJointNames[j] ]->CreateKeyFrame(t);
	  joint_key->SetValue( ang[j] );	  
	}
	// Advance one time step
	t+=dt;
      }

      fclose( pFile );

      // Attach the animation to the model
      _model->SetJointAnimation( joint_anim );
      printf("End loading Joint animation \n");
      
      //*********************
      // POSE ANIMATION
      //*********************
      gazebo::common::PoseAnimationPtr pose_anim( new gazebo::common::PoseAnimation("test", T, true) );
      gazebo::common::PoseKeyFrame *pose_key;

      t = 0;
      float posx, posy, posz;


      pFile = fopen(VRC_DATA_PATH "trajs/com.txt", "r");

      for( int i = 0; i < numTrajPoints; ++i ) {

        fscanf( pFile, "%f %f %f \n", &posx, &posy, &posz );  
	pose_key = pose_anim->CreateKeyFrame(t);
        std::cout<<"xyz["<<i<<"] : "<<posx<<","<<posy<<","<<posz << std::endl;
	pose_key->SetTranslation(math::Vector3( posx, posy, posz + 0.39));
	pose_key->SetRotation(math::Quaternion(0, 0, 0.0));

	// Advance one time step
	t+=dt;
      }
      
      fclose( pFile );

      _model->SetAnimation( pose_anim );      
      printf( "** End loading Pose animation **\n" );
    }

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(drchubo_Animation)
}
