/**
 * @file drchubo_demoApp.h
 * @brief Creates application for pushDemoTab
 * @author A. Huaman Q.
 */
#include "GRIPApp.h"
#include "drchubo_demo.h"

extern wxNotebook* tabView;

class drchubo_demoApp : public GRIPApp {
  virtual void AddTabs() {
    tabView->AddPage(new drchubo_demo(tabView), wxT("DRCHubo Demo"));
  }
};

IMPLEMENT_APP( drchubo_demoApp )
