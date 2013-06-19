/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * 
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <vector>
#include <Tabs/GRIPTab.h>

namespace planning { class Controller; }

class drchubo_demo : public GRIPTab
{
public:
  drchubo_demo() {};
  drchubo_demo( wxWindow * parent, wxWindowID id = -1, 
		const wxPoint & pos = wxDefaultPosition, 
		const wxSize & size = wxDefaultSize, 
		long style = wxTAB_TRAVERSAL );
  virtual ~drchubo_demo() {};

  virtual void GRIPEventSimulationBeforeTimestep();
  virtual void GRIPEventSceneLoaded();

  void onButtonSetStart(wxCommandEvent & _evt);
  void onButtonSetGoal(wxCommandEvent & _evt);
  void onButtonSetPredefStart(wxCommandEvent & _evt);
  void onButtonSetPredefGoal(wxCommandEvent & _evt);
  void onButtonRelocateObjects(wxCommandEvent & _evt);
  void onButtonShowStart(wxCommandEvent & _evt);
  void onButtonShowGoal(wxCommandEvent & _evt);
  void onButtonPlan(wxCommandEvent & _evt);

  planning::Controller* mController;
  
  int mRobotIndex;
  std::string mRobotName;
  std::vector<int> mArmDofs;
  Eigen::VectorXd mStartConf;
  Eigen::VectorXd mGoalConf;
  Eigen::VectorXd mPredefStartConf;
  Eigen::VectorXd mPredefGoalConf;

  DECLARE_DYNAMIC_CLASS(drchubo_demo)
  DECLARE_EVENT_TABLE()
};

