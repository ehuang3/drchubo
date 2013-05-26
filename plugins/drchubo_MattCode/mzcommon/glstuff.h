/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _GLSTUFF_H_
#define _GLSTUFF_H_

#ifdef HAVE_GLEW
#include <GL/glew.h>
#else
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#endif

#include "Transform3.h"

namespace glstuff {

  void check_opengl_errors(const char* context);

  size_t pwr2(size_t x);

  inline void get(GLenum which, GLdouble* data) {
    glGetDoublev(which, data);
  }

  inline void get(GLenum which, GLfloat* data) {
    glGetFloatv(which, data);
  }

  inline void translate(GLfloat x, GLfloat y, GLfloat z) {
    glTranslatef(x,y,z);
  }

  inline void translate(GLdouble x, GLdouble y, GLdouble z) {
    glTranslated(x,y,z);
  }

  inline void translate(const vec3f& v) { glTranslatef( v[0], v[1], v[2] ); }
  inline void translate(const vec3d& v) { glTranslated( v[0], v[1], v[2] ); }

  inline void color(const vec3f& v) { glColor3fv( v.v ); }
  inline void color(const vec3d& v) { glColor3dv( v.v ); }

  inline void color(const vec4f& v) { glColor4fv( v.v ); }
  inline void color(const vec4d& v) { glColor4dv( v.v ); }

  inline void normal(const vec3f& v) { glNormal3fv( v.v ); }
  inline void normal(const vec3d& v) { glNormal3dv( v.v ); }

  inline void vertex(const vec3f& v) { glVertex3fv( v.v ); }
  inline void vertex(const vec3d& v) { glVertex3dv( v.v ); }

  template <class real, class GLreal>
  inline void mat4_to_gl(const mat4_t<real>& mat, GLreal data[16]) {

    data[0]  = mat(0,0);
    data[1]  = mat(1,0);
    data[2]  = mat(2,0);
    data[3]  = mat(3,0);

    data[4]  = mat(0,1);
    data[5]  = mat(1,1);
    data[6]  = mat(2,1);
    data[7]  = mat(3,1);

    data[8]  = mat(0,2);
    data[9]  = mat(1,2);
    data[10] = mat(2,2);
    data[11] = mat(3,2);

    data[12] = mat(0,3);
    data[13] = mat(1,3);
    data[14] = mat(2,3);
    data[15] = mat(3,3);

  }
  
  template <class real, class GLreal>
  inline void gl_to_mat4(const GLreal data[16], mat4_t<real>& mat) {

    mat(0,0) = data[0];
    mat(1,0) = data[1];
    mat(2,0) = data[2];
    mat(3,0) = data[3];

    mat(0,1) = data[4];
    mat(1,1) = data[5];
    mat(2,1) = data[6];
    mat(3,1) = data[7];

    mat(0,2) = data[8];
    mat(1,2) = data[9];
    mat(2,2) = data[10];
    mat(3,2) = data[11];

    mat(0,3) = data[12];
    mat(1,3) = data[13];
    mat(2,3) = data[14];
    mat(3,3) = data[15];

  }

  inline void load_matrix(GLfloat* data)  { glLoadMatrixf(data); }
  inline void load_matrix(GLdouble* data) { glLoadMatrixd(data); }

  inline void mult_matrix(GLfloat* data)  { glMultMatrixf(data); }
  inline void mult_matrix(GLdouble* data) { glMultMatrixd(data); }
  
  template <class real>
  inline void get_mat4(GLenum which, mat4_t<real>& mat) {
    real data[16];
    get(which, data);
    gl_to_mat4(data, mat);
  }

  template <class real>
  inline void load_mat4(const mat4_t<real>& mat) {
    real data[16];
    mat4_to_gl(mat, data);
    load_matrix(data);
  }

  template <class real>
  inline void mult_mat4(const mat4_t<real>& mat) {
    real data[16];
    mat4_to_gl(mat, data);
    mult_matrix(data);
  }


  template <class real>
  inline void load_transform(const Transform3_t<real>& tx) {
    load_mat4(tx.matrix());
  }

  template <class real>
  inline void mult_transform(const Transform3_t<real>& tx) {
    mult_mat4(tx.matrix());
  }

  enum { 
    DEFAULT_SLICES = 32,
    DEFAULT_CSTACKS = 4,
    DEFAULT_SSTACKS = 24
  };

  template<class real>
  inline bool setup_cylinder(const vec3_t<real>& p0,
                             const vec3_t<real>& p1,
                             real& len) {
    
    vec3_t<real> diff = p1-p0;
    len = diff.norm();

    if (len <= 1e-9) {
      translate(p0);
      return false;
    }

    vec3_t<real> rz = diff / len;
  
    vec3_t<real> rx = vec3_t<real>::cross(vec3_t<real>(0,1,0), rz);
    real xn = rx.norm();
  
    if (xn > 0.1) {
      rx /= xn;
    } else {
      rx = vec3_t<real>::cross(vec3_t<real>(1,0,0), rz);
      rx /= rx.norm();
    }
  
    vec3_t<real> ry = vec3_t<real>::cross(rz, rx);
  
    mat4_t<real> m;
    for (int i=0; i<3; ++i) {
      m(i,0) = rx[i];
      m(i,1) = ry[i];
      m(i,2) = rz[i];
      m(i,3) = p0[i];
    }
  
    mult_mat4(m);

    return true;

  }


  template <class real>
  inline void draw_cylinder(GLUquadric* q,
                            const vec3_t<real>& p0,
                            const vec3_t<real>& p1, 
                            real rad,
                            int slices=DEFAULT_SLICES,
                            int cstacks=DEFAULT_CSTACKS,
                            int cloops=1) {

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    real len;

    if (!setup_cylinder(p0, p1, len)) {
    
      return;
    
    } else {
    
      gluCylinder(q, rad, rad, len, slices, cstacks);
      gluQuadricOrientation(q, GLU_INSIDE);
      gluDisk(q, 0, rad, slices, cloops);

      translate(0, 0, len);
      gluQuadricOrientation(q, GLU_OUTSIDE);
      gluDisk(q, 0, rad, slices, cloops);


    }

    glPopMatrix();


  }

  template <class real>
  void draw_capsule(GLUquadric* q,
                    const vec3_t<real>& p0,
                    const vec3_t<real>& p1,
                    real rad,
                    int slices=DEFAULT_SLICES,
                    int sstacks=DEFAULT_SSTACKS,
                    int cstacks=DEFAULT_CSTACKS) {

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    real len;

    if (!setup_cylinder(p0, p1, len)) {
    
      gluSphere(q, rad, slices, sstacks);
    
    } else {
    
      gluSphere(q, rad, slices, sstacks);
      gluCylinder(q, rad, rad, len, slices, cstacks);
      translate(0, 0, len);
      gluSphere(q, rad, slices, sstacks);

    }

    glPopMatrix();
  
  }

  template <class real>
  inline void draw_arrow(GLUquadric* q,
                         const vec3_t<real>& p0,
                         const vec3_t<real>& p1,
                         real cyl_rad,
                         real head_rad=0, // defaults to 2*cyl_rad
                         real head_len=0, // defaults to 2*cyl_rad
                         int slices=DEFAULT_SLICES,
                         int cstacks=DEFAULT_CSTACKS,
                         int hstacks=1,
                         int hloops=1,
                         int cloops=1) {

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    real len;
    if (setup_cylinder(p0, p1, len)) { 

      if (!head_rad) { head_rad = 2*cyl_rad; }
      if (!head_len) { head_len = 4*cyl_rad; }

      real clen = len - 0.5 * head_len;
      real flange_inner = 0;

      if (clen > 0) {

        // cylinder
        gluCylinder(q, cyl_rad, cyl_rad, clen, slices, cstacks);

        // base disk
        gluQuadricOrientation(q, GLU_INSIDE);
        gluDisk(q, 0, cyl_rad, slices, cloops);
      
        flange_inner = cyl_rad;

      }

      translate(0, 0, clen);

      // flange
      gluDisk(q, flange_inner, head_rad, slices, hloops);
      gluQuadricOrientation(q, GLU_OUTSIDE);

      // cone
      gluCylinder(q, head_rad, 0, head_len, slices, hstacks);


    }
  
    glPopMatrix();
  
  }

}

#endif
