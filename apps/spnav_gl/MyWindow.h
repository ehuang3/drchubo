#pragma once

#include <cstdio>
#include <stdarg.h>
#include "yui/Win3D.h"
#include "simulation/SimWindow.h"

// vrc stuff
#include <robot/robot_state.h>

namespace dynamics{
	class SkeletonDynamics;
	class ContactDynamics;
}

class MyWindow : public simulation::SimWindow {
public:

	/// The constructor - set the position of the skeleton
	MyWindow(): SimWindow() {
		mTrans[1] = 0.f;
		mZoom = 0.3;
	}
	virtual ~MyWindow() {}

    // pthread hook
    static void* start_routine(void *arg);

	/// Draw the skeleton
	virtual void drawSkels();

	/// Move the joints with the {1,2,3} keys and '-' to change direction
	//virtual void keyboard(unsigned char key, int x, int y);

    void setCurrentState(robot::robot_state_t* stateCurrent) { current_state = stateCurrent; }
    void setTargetState(robot::robot_state_t* stateTarget) { target_state = stateTarget; }

    // void addRobotState(robot::robot_state_t* robot) { robotStates.push_back(robot); }

private:
    robot::robot_state_t * current_state;
    robot::robot_state_t * target_state;

    std::vector<robot::robot_state_t*> robotStates;
};
