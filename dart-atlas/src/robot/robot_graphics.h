#pragma once
#include <renderer/LoadOpengl.h>

namespace kinematics {
	class Skeleton;
	class BodyNode;
}
namespace renderer { class RenderInterface; }
namespace robot
{

class robot_graphics_t {
public:
	void renderCOM(kinematics::Skeleton *_robot, renderer::RenderInterface *_ri);
	void renderJoints(kinematics::Skeleton *_robot, renderer::RenderInterface *_ri);
	void renderCOM(kinematics::BodyNode *_node, renderer::RenderInterface *_ri);
	void renderJoints(kinematics::BodyNode *_node, renderer::RenderInterface *_ri);
};

}
