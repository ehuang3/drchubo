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

      std::map<std::string, common::NumericAnimationPtr> anim;
      double T = 10.0;

      // Fill joint init
      for( int i = 0; i < gNumJoints; ++i ) {
	anim[ gJointNames[i] ].reset( new common::NumericAnimation( "anim", T, true) );
      }


      // Read file with trajectories
      float ang[gNumJoints]; int ind;
      int numTrajPoints = 900;
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
	  common::NumericKeyFrame * key = anim[ gJointNames[j] ]->CreateKeyFrame(t);
	  key->SetValue( ang[j] );	  
	}
	// Advance one time step
	t+=dt;

      }

//////////////////////////////
// Save stance
    std::vector<int> mTrajStance;
    mTrajStance.resize(0);

    // Read file with trajectories and stance
    FILE* sFile;
    sFile = fopen(VRC_DATA_PATH "trajs/stance.txt", "r");
    
    if( sFile == NULL ) {
      printf("Did not find either traj or stance file, exiting! \n");
      return;
    }
    
    for( int i = 0; i < numTrajPoints; ++i ) {
      // Read stance
      int stance;
      fscanf( sFile, "%d \n", &stance );
      
      mTrajStance.push_back( stance );
    }
    
    fclose( sFile );
//////////////////////////////////
	
      // Attach the animation to the model
      _model->SetJointAnimation( anim );
      printf("End loading animation \n");

      /////////////////////////////////
      // WORLD ANIMATION  		
      gazebo::common::PoseAnimationPtr anim2(
          new gazebo::common::PoseAnimation("test", 1000.0, true));

      gazebo::common::PoseKeyFrame *key2;

      t = 0;
      double xdist = 0;
      for( int i = 0; i < numTrajPoints; ++i ) {

          key2 = anim2->CreateKeyFrame(t);
	// Single does not move
          if( mTrajStance[i] == 2 || mTrajStance[i] == 3 ) {

	  }
          else {
           xdist += dt;
          }

            key2->SetTranslation(math::Vector3( xdist, 0, 1.0));
            key2->SetRotation(math::Quaternion(0, 0, 0.0));

	// Advance one time step
	t+=dt;

      }

      _model->SetAnimation(anim2);

	////////////////////




    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(drchubo_Animation)
}
