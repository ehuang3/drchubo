#pragma once

#include <renderer/LoadOpengl.h>

namespace kinematics {
	class Skeleton;
	class BodyNode;
}
namespace renderer {
	class RenderInterface;
}

namespace atlas {

class AtlasGraphics {
public:
	void renderCOM(kinematics::Skeleton *_atlas, renderer::RenderInterface *_ri);
	void renderJoints(kinematics::Skeleton *_atlas, renderer::RenderInterface *_ri);
private:
	void renderCOM(kinematics::BodyNode *_node, renderer::RenderInterface *_ri);
	void renderJoints(kinematics::BodyNode *_node, renderer::RenderInterface *_ri);
};

}
