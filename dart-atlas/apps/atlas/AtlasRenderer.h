#pragma once

namespace dynamics {
	class SkeletonDynamics;
}

namespace renderer {
	class RenderInterface;
}

namespace atlas {

class AtlasRenderer {
public:
	void drawSkeleton(dynamics::SkeletonDynamics *_atlas, renderer::RenderInterface* _ri);
};

}