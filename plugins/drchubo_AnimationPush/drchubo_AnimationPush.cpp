#include "common/CommonTypes.hh"
#include "common/Animation.hh"
#include "common/KeyFrame.hh"
#include "physics/Model.hh"
#include "gazebo.hh"

namespace gazebo
{
  class AnimatePose : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      gazebo::common::PoseAnimationPtr anim(
          new gazebo::common::PoseAnimation("test", 1000.0, true));

      gazebo::common::PoseKeyFrame *key;

      int numTrajPoints = 900;
      double T = 10.0;
      double t;
      double dt = T / (double)numTrajPoints;

      t = 0;
      for( int i = 0; i < numTrajPoints; ++i ) {

          key = anim->CreateKeyFrame(t);
          key->SetTranslation(math::Vector3(t, 0, 0));
          key->SetRotation(math::Quaternion(0, 0, 0.0));
	// Advance one time step
	t+=dt;

      }
	





      _parent->SetAnimation(anim);
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatePose)
}
