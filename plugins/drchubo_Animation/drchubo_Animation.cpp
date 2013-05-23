#include <map>
#include "gazebo.hh"
#include "common/common.hh"
#include "physics/physics.hh"

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
      double T = 5.0;

      // Fill joint init
      for( int i = 0; i < gNumJoints; ++i ) {
	anim[ gJointNames[i] ].reset( new common::NumericAnimation( "anim", T, true) );
      }


      // Read file with trajectories
      float ang[gNumJoints]; int ind;
      int numTrajPoints = 1;
      FILE* pFile;
      pFile = fopen("traj.txt", "r");

      if( pFile == NULL ) {
	printf("Did not find file, exiting! \n");
	return;
      }

      double t;

      double dt = T / (double)numTrajPoints;

      t = 0;
      for( int i = 0; i < numTrajPoints; ++i ) {

	fscanf( pFile, "%d %f %f %f %f %f %f %f", &ind, &ang[0], &ang[1], &ang[2], &ang[3], &ang[4], &ang[5], &ang[6] );
	fscanf( pFile, "%f %f %f %f %f %f %f", &ang[7], &ang[8], &ang[9], &ang[10], &ang[11], &ang[12], &ang[13] );
	fscanf( pFile, "%f %f %f %f %f %f", &ang[14], &ang[15], &ang[16], &ang[17], &ang[18], &ang[19] );
	fscanf( pFile, "%f %f %f %f %f %f \n", &ang[20], &ang[21], &ang[22], &ang[23], &ang[24], &ang[25] );

	// Create a key frame for each trajectory point read
	for( int j = 0; j < gNumJoints; ++j ) {
	  common::NumericKeyFrame * key = anim[ gJointNames[j] ]->CreateKeyFrame(t);
	  key->SetValue( ang[j] );	  
	  printf("val[%d]=%f\n", j, ang[j]);
	}
	// Advance one time step
	t+=dt;

      }
	
      // Attach the animation to the model
      _model->SetJointAnimation( anim );
      printf("End loading animation \n");

    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(drchubo_Animation)
}
