#pragma once

#include <cstdio>
#include <stdarg.h>
#include "yui/Win3D.h"
#include "simulation/SimWindow.h"

namespace dynamics{
	class SkeletonDynamics;
	class ContactDynamics;
}

class MyWindow : public simulation::SimWindow {
public:

	/// The constructor - set the position of the skeleton
	MyWindow(): SimWindow() {
		mTrans[1] = 200.f;
		mZoom = 0.3;
	}
	virtual ~MyWindow() {}

	/// Draw the skeleton
	virtual void drawSkels();

	/// Move the joints with the {1,2,3} keys and '-' to change direction
	//virtual void keyboard(unsigned char key, int x, int y);
};
