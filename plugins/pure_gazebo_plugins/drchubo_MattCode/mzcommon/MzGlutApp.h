/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _MZGLUTAPP_H_
#define _MZGLUTAPP_H_

#include "GlCamera.h"

#ifdef __APPLE__
#include <Glut/glut.h>
#else
#include <GL/glut.h>
#endif

#include <cstdlib>

class MzGlutApp {
public:

  enum {
    DEFAULT_PARAMS = GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB,
    NICE_PARAMS = DEFAULT_PARAMS | GLUT_MULTISAMPLE
  };

  int width;
  int height;
  GlCamera camera;
  GlCamera::MouseMode mmode;

  virtual ~MzGlutApp();

  void initWindowPosition(int x, int y);
  void initWindowSize(int w, int h);
  void createWindow(const char* name);
  void setTimer(unsigned int msec, int value);
  void setIdle();
  void clearIdle();

  void setupBasicLight(const vec4f& position=vec4f(-1,1,1,0));

  virtual void run();

  virtual void display();
  virtual void reshape(int width, int height);
  virtual void keyboard(unsigned char key, int x, int y);
  virtual void special(int key, int x, int y);
  virtual void mouse(int button, int state, int x, int y);
  virtual void motion(int x, int y);
  virtual void timer(int value);
  virtual void idle();
  virtual void viewChanged(bool interactive);

  virtual GlCamera::MouseMode btn2cam(int button) const;

  static MzGlutApp& getInstance();

protected:
  MzGlutApp(int argc, char** argv, int displaymode=0);

private:

  static MzGlutApp* _instance;

};

#endif
