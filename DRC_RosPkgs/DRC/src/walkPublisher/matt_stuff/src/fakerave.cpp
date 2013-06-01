/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "fakerave.h"
#include <mzcommon/strutils.h>
#include <mzcommon/glstuff.h>
#include <Eigen/Dense>

namespace fakerave {

  Geom::Geom(): haveColor(false), list(0){ }

  Body::Body(): 
    offsetFromIndex(-1), 
    parentJointIndex(-1), 
    haveMass(false), 
    haveInertia(false) {}

  Joint::Joint(): haveAnchor(false), haveAxis(false), haveLimits(false) {
    limits[0] = 1;
    limits[1] = -1;
  }

  Manipulator::Manipulator(): drawScale(0.1) {}

  //////////////////////////////////////////////////////////////////////

  void KinBody::setFilename(const std::string& f) {
    filename = f;
    directory = directoryOf(f);
  }

  size_t KinBody::lookupBody(const std::string& name) const {
    for (size_t i=0; i<bodies.size(); ++i) {
      if (bodies[i].name == name) { return i; }
    }
    return -1;
  }

  size_t KinBody::lookupJoint(const std::string& name) const {

    for (size_t i=0; i<joints.size(); ++i) {
      if (joints[i].name == name) { return i; }
    }
    return -1;

  }

  size_t KinBody::lookupManipulator(const std::string& name) const {

    for (size_t i=0; i<manipulators.size(); ++i) {
      if (manipulators[i].name == name) { return i; }
    }
    return -1;

  }

  bool KinBody::isAncestorOf(size_t rootIndex, size_t leafIndex) const {

    assert(rootIndex < bodies.size());
    assert(leafIndex < bodies.size());

    if (bodies[leafIndex].offsetFromIndex >= bodies.size()) {
      return false;
    } else if (bodies[leafIndex].offsetFromIndex == rootIndex) {
      return true;
    } else {
      return isAncestorOf(rootIndex, bodies[leafIndex].offsetFromIndex);
    }

  }

  bool KinBody::bodyDependsOnJoint(const size_t bodyIndex,
                                   const size_t jointIndex) const {

    assert(bodyIndex < bodies.size());
    assert(jointIndex < joints.size());

    const Joint& j = joints[jointIndex];

    return (j.body2Index == bodyIndex || 
            isAncestorOf(j.body2Index, bodyIndex));

  }

  void KinBody::resolve(size_t& i, const std::string& n, bool allowBlank) {
    if (n.empty()) {
      assert(allowBlank);
      i = size_t(-1);
    } else {
      i = lookupBody(n);
      assert(i < bodies.size());
    }
  }
       
  void KinBody::transforms(const RealArray& jvalues,
                           Transform3Array& xforms,
                           const BoolArray* active) const {

    assert(joints.size() == jvalues.size());
    assert(!active || active->size() == bodies.size());

    ////////////////////////////////////////////////////////////////////
    // perform relative transformations

    xforms.clear();
    xforms.resize(bodies.size());

    // for each body
    for (size_t i=0; i<bodies.size(); ++i) {
      if (!active || (*active)[i]) {
        xforms[i] = bodies[i].xform;
      }
    }

    // for each joint
    for (size_t i=0; i<joints.size(); ++i) {
      
      const Joint& j = joints[i];

      size_t bi1 = j.body1Index;
      size_t bi2 = j.body2Index;

      if (!active || (*active)[bi2]) {

        size_t oi = j.offsetFromIndex;
      
        assert( bodies[bi2].offsetFromIndex == bi1 );

        vec3 t = j.anchor;
        quat q = quat::fromAxisAngle( j.axis, jvalues[i] );
      
        Transform3 jx = Transform3(t) * Transform3(q) * Transform3(-t);

        if (oi == bi1) {
          xforms[bi2] = jx * xforms[bi2];
        } else {
          xforms[bi2] = xforms[bi2] * jx;
        } 

      }

    }

    //////////////////////////////////////////////////////////////////////

    for (size_t k=0; k<bodyOrder.size(); ++k) {

      size_t i = bodyOrder[k];
      const Body& b = bodies[i];


      if (!active || (*active)[i]) {

        if (b.offsetFromIndex < bodies.size()) {

          xforms[i] = xforms[b.offsetFromIndex] * xforms[i];

        }

      }

    }

  }


  void KinBody::jointJacobian(const Transform3Array& xforms,
                              size_t bodyIndex,
                              const vec3& point,      // only used for jpos
                              bool pointInWorldFrame, // only used for jpos
                              size_t jointIndex,
                              vec3* jpos,
                              vec3* jrot) const {

    assert(xforms.size() == bodies.size());
    assert(bodyIndex < bodies.size());

    assert(jointIndex < joints.size() || 
           (jointIndex >= DOF_POS_X && jointIndex <= DOF_ROT_Z));

    if (jointIndex >= DOF_POS_X && jointIndex <= DOF_POS_Z) {

      size_t aidx = jointIndex - DOF_POS_X;
      vec3 axis(0); axis[aidx] = 1;
      
      if (jpos) { *jpos = axis; }
      if (jrot) { *jrot = vec3(0); }

    } else if (jointIndex >= DOF_ROT_X && jointIndex <= DOF_ROT_Z) {
      
      vec3 worldPoint = pointInWorldFrame ? point : xforms[bodyIndex] * point;

      size_t aidx = jointIndex - DOF_ROT_X;
      vec3 axis(0); axis[aidx] = 1;
      
      if (jpos) { *jpos = vec3::cross(axis, worldPoint); }
      if (jrot) { *jrot = axis; }

    } else if (!bodyDependsOnJoint(bodyIndex, jointIndex)) {

      if (jpos) { *jpos = vec3(0); }
      if (jrot) { *jrot = vec3(0); }

    } else {

      const Joint& j = joints[jointIndex];
      
      vec3 worldAxis = xforms[j.offsetFromIndex].rotFwd() * j.axis;

      if (jpos) {
        vec3 worldPoint = pointInWorldFrame ? point : xforms[bodyIndex] * point;
        vec3 worldAnchor = xforms[j.offsetFromIndex] * j.anchor;
        *jpos = vec3::cross( worldAxis, worldPoint - worldAnchor );
      }

      if (jrot) {
        *jrot = worldAxis;
      }

    }

  }

  vec3 KinBody::jointAnchor(const Transform3Array& xforms,
                               size_t ji) const {

    assert(xforms.size() == bodies.size());
    assert(ji < joints.size());

    const Joint& j = joints[ji];

    return xforms[j.offsetFromIndex] * j.anchor;

  }

  vec3 KinBody::jointAxis(const Transform3Array& xforms,
                             size_t ji) const {

    assert(xforms.size() == bodies.size());
    assert(ji < joints.size());

    const Joint& j = joints[ji];

    return xforms[j.offsetFromIndex].rotFwd() * j.axis;

  }

  // get full jacobian
  void KinBody::jacobian(const Transform3Array& xforms,
                         size_t bodyIndex,
                         const vec3& point,
                         bool pointInWorldFrame,
                         const IndexArray& jointIndices,
                         MatX& Jt) const {

    int nj = jointIndices.size();
    vec3 jpi(false), jri(false);

    if (Jt.rows() != nj || Jt.cols() != 6) { Jt = MatX(nj, 6); }

    for (int i=0; i<nj; ++i) {

      size_t ji = jointIndices[i];

      jointJacobian(xforms, bodyIndex, 
                    point, pointInWorldFrame,
                    ji, &jpi, &jri);

      
      dpdq2mat(jpi, jri, Jt.row(i));

    }
    
  }

  Transform3 KinBody::manipulatorFK(const Transform3Array& xforms,
                                    size_t manipIndex) const {
    
    const Manipulator& m = manipulators[manipIndex];
    return xforms[m.effectorIndex] * m.xform;

  }

  static inline void tweakDOF(Transform3& X, size_t jointIndex, real offset) {

    if (jointIndex >= DOF_POS_X && jointIndex <= DOF_POS_Z) {

      size_t aidx = jointIndex - DOF_POS_X;
      vec3 axis(0); axis[aidx] = 1;
      X.setTranslation(axis * offset);

    } else if (jointIndex >= DOF_ROT_X && jointIndex <= DOF_ROT_Z) {

      size_t aidx = jointIndex - DOF_ROT_X;
      vec3 axis(0); axis[aidx] = 1;
      X.setRotation(quat::fromEuler(axis * offset));

    } else { 

      assert( !"bad joint index in tweakDOF!" );

    }


  }

  void KinBody::manipulatorNumericalJacobian(size_t mi,
                                             RealArray& jvalues,
                                             Transform3Array& xforms,
                                             MatX& Jnt, 
                                             real h) const {

    assert( mi < manipulators.size() );
    
    size_t bodyIndex = manipulators[mi].effectorIndex;
    vec3 bodyPoint = manipulators[mi].xform.translation();

    numericalJacobian(jvalues, xforms,
                      bodyIndex, bodyPoint, false,
                      manipulators[mi].jointIndices,
                      Jnt, h);

  }

  void KinBody::numericalJacobian(RealArray& jvalues,
                                  Transform3Array& xforms,
                                  size_t bodyIndex,
                                  const vec3& point,
                                  bool pointInWorldFrame,
                                  const IndexArray& jidx,
                                  MatX& Jnt,
                                  real h) const {

    vec3 pointBody = pointInWorldFrame ? xforms[bodyIndex].inverse() * point : point;

    Transform3 Xb(pointBody);


    Jnt = MatX(jidx.size(), 6);

    // do numerical jacobian
    const real hscl = 0.5 / h;

    for (size_t i=0; i<jidx.size(); ++i) {
      
      real orig = jvalues[jidx[i]];
      const real delta[3] = { -1, 1, 0 };

      Transform3 t[3];
      Transform3 X;

      for (int x=0; x<3; ++x) {
        if (jidx[i] < jvalues.size()) {
          jvalues[jidx[i]] = orig + delta[x]*h;
        } else { 
          tweakDOF(X, jidx[i], delta[x]*h);
        }
        transforms(jvalues, xforms);
        t[x] = X * xforms[bodyIndex] * Xb;
      }

      vec3 dp, dq;

      deltaTransform(t[1], t[0], dp, dq);

      dp *= hscl;
      dq *= hscl;

      dpdq2mat(dp, dq, Jnt.row(i));
      
    }

  }

  void KinBody::manipulatorJacobian(const Transform3Array& xforms,
                                    size_t mi,
                                    const IndexArray& jointIndices,
                                    MatX& Jt) const {
    
    assert( mi < manipulators.size() );
    
    size_t bodyIndex = manipulators[mi].effectorIndex;
    vec3 bodyPoint = manipulators[mi].xform.translation();
    
    jacobian(xforms,
             bodyIndex, bodyPoint, false,
             jointIndices,
             Jt);

  }

  

  void KinBody::manipulatorJacobian(const Transform3Array& xforms,
                                    size_t mi,
                                    MatX& Jt) const {

    assert( mi < manipulators.size() );
    manipulatorJacobian(xforms, mi, 
                        manipulators[mi].jointIndices, Jt);
    
    
    
  }


  bool KinBody::clampAngle(real l, real u,
                           real& angle) {

    if (u <= l) { return true; }
  
    while (angle < l && angle + 2*M_PI <= u) { angle += 2*M_PI; }
    while (angle > u && angle - 2*M_PI >= l) { angle -= 2*M_PI; }
    
    if (angle < l) { 
      angle = l;
      return false;
    } else if (angle > u) {
      angle = u;
      return false;
    } else {
      return true;
    }

  }

  bool KinBody::clampToLimits(RealArray& jvalues) const {

    assert(jvalues.size() == joints.size());

    bool ok = true;
  
    for (size_t i=0; i<jvalues.size(); ++i) {
      const Joint& j = joints[i];
      real l = j.limits[0];
      real u = j.limits[1];
      ok = clampAngle(l, u, jvalues[i]) && ok;
    }

    return ok;

  }

  bool KinBody::clampToLimits(MatX& jmat, const IndexArray& jidx) const {

    assert( ((unsigned)jmat.rows() == jidx.size() && jmat.cols() == 1) ||
            ((unsigned)jmat.cols() == jidx.size() && jmat.rows() == 1) );


    bool ok = true;
  
    for (size_t i=0; i<jidx.size(); ++i) {
      size_t ji = jidx[i];
      assert(ji < joints.size());
      const Joint& j = joints[ji];
      real l = j.limits[0];
      real u = j.limits[1];
      ok = clampAngle(l, u, jmat(i)) && ok;
    }

    return ok;

  }

  KinBody::KinBody() {

    DEFAULT_PTOL = 5e-4;
    DEFAULT_QTOL = 1e-7;
    DEFAULT_ITER = 100;

  }

  void KinBody::centerJoints(const IndexArray& jidx,
                             RealArray& jvalues) const {

    assert( jvalues.size() == joints.size() );

    for (size_t i=0; i<jidx.size(); ++i) {

      size_t ji = jidx[i];
      assert(ji < joints.size());

      const Joint& j = joints[ji];
      if (j.limits[0] < j.limits[1]) {
        jvalues[ji] = 0.5*(j.limits[0] + j.limits[1]);
      } else {
        jvalues[ji] = 0;
      }
      
    }

  }

  void KinBody::zeroJoints(const IndexArray& jidx,
                           RealArray& jvalues) const {
    
    assert( jvalues.size() == joints.size() );
    
    for (size_t i=0; i<jidx.size(); ++i) {
      
      size_t ji = jidx[i];
      assert(ji < joints.size());
      jvalues[ji] = 0;
      
    }

  }

  void KinBody::resolve() {

    for (size_t i=0; i<bodies.size(); ++i) {

      Body& b = bodies[i];
      resolve(b.offsetFromIndex, b.offsetFromName, true);

    }

    for (size_t i=0; i<joints.size(); ++i) {


      Joint& j = joints[i];

      resolve(j.body1Index, j.body1Name, false);
      resolve(j.body2Index, j.body2Name, false);
      resolve(j.offsetFromIndex, j.offsetFromName, false);

      assert( j.offsetFromIndex == j.body1Index ||
              j.offsetFromIndex == j.body2Index );

      Body& b2 = bodies[j.body2Index];

      assert(b2.offsetFromIndex == j.body1Index);
      b2.parentJointIndex = i;

    }

    for (size_t i=0; i<manipulators.size(); ++i) {

      Manipulator& m = manipulators[i];

      resolve(m.effectorIndex, m.effectorName, false);
      resolve(m.baseIndex, m.baseName, false);

      assert( isAncestorOf(m.baseIndex, m.effectorIndex) );

      m.activeBodies.clear();
      m.activeBodies.resize(bodies.size(), false);

      for (size_t ji=0; ji<joints.size(); ++ji) {
        if (bodyDependsOnJoint(m.effectorIndex, ji)) {
          m.activeBodies[joints[ji].body1Index] = true;
          m.activeBodies[joints[ji].body2Index] = true;
        }
      }

    }

    //////////////////////////////////////////////////

    bodyOrder.clear();

    size_t remainingBodies = bodies.size();

    std::vector<bool> picked(bodies.size(), false);

    while (remainingBodies) {

      size_t pickIndex = -1;

      for (size_t i=0; i<bodies.size(); ++i) {
        const Body& b = bodies[i];
        if (picked[i]) { continue; }
        if (b.offsetFromIndex >= bodies.size() || picked[b.offsetFromIndex]) {
          pickIndex = i;
          break;
        }
      }

      if (pickIndex >= bodies.size()) {
        std::cout << "detected cycle in bodies!\n";
        exit(1);
      }

      bodyOrder.push_back(pickIndex);
      picked[pickIndex] = true;
      --remainingBodies;

    }

    ////////////////////////////////////////////////////////////

    for (size_t i=0; i<bodies.size(); ++i) {

      Body& b = bodies[i];

      for (size_t j=0; j<b.geoms.size(); ++j) {

        Geom& g = b.geoms[j];

        if (g.type == "cylinder") {

          int cslices = 32;
          g.render = TriMesh3::cylinder( g.radius, g.height, cslices );


          const real Rdata[9] = {
            1,  0, 0, 
            0,  0, 1, 
            0, -1, 0,
          };

          mat3 R(Rdata);

          g.render.applyTransform(Transform3(quat::fromMat3(R), 
                                                 vec3(0, -0.5*g.height, 0)));

          g.data = g.render;

        } else if (g.type == "sphere") {

          int sslices = 32;
          int sstacks = 24;

          g.render = TriMesh3::sphere( g.radius, sslices, sstacks );

          const real Rdata[9] = {
            1,  0, 0, 
            0,  0, 1, 
            0, -1, 0,
          };

          mat3 R(Rdata);

          g.render.applyTransform(Transform3(quat::fromMat3(R))); 

          g.data = g.render;


        } else if (g.type == "box") {

          g.render = TriMesh3::box(g.boxExtents[0],
                                       g.boxExtents[1],
                                       g.boxExtents[2]);

          g.data = g.render;

        }

      }


      for (size_t j=0; j<b.cgeoms.size(); ++j) {

        Geom& cg = b.cgeoms[j];
        if (cg.type == "auto_aabb" || cg.type == "auto_sphere") {

          // resolve points for box
          std::vector<vec3> allpoints;
          for (size_t k=0; k<b.geoms.size(); ++k) {
            const Geom& g = b.geoms[k];
            const TriMesh3& m = g.data.empty() ? g.render : g.data;
            for (size_t n=0; n<m.verts.size(); ++n) {
              allpoints.push_back(g.xform.transformFwd(m.verts[n]));
            }
          }

          if (cg.type == "auto_sphere") {

            Box3_t<real> bbox;
            
            for (size_t n=0; n<allpoints.size(); ++n) {
              bbox.addPoint(allpoints[n]);
            }

            vec3 vmean = bbox.center();

            cg.radius = 0;

            for (size_t n=0; n<allpoints.size(); ++n) {
              vec3 diff = allpoints[n] - vmean;
              cg.radius = std::max( cg.radius, diff.norm2() );
            }

            cg.radius = sqrt(cg.radius);

            cg.xform = Transform3(vmean);

            cg.type = "sphere";

          } else if (cg.type == "auto_aabb") {
            
            Box3_t<real> bbox;
            
            for (size_t n=0; n<allpoints.size(); ++n) {
              bbox.addPoint(allpoints[n]);
            }

            vec3 vmean = bbox.center();
            
            cg.boxExtents = bbox.p1 - bbox.p0;
            cg.xform = Transform3(vmean);
            cg.type = "box";

          }

        }
        
      }
    }

  }


  void KinBody::renderSkeleton(const Transform3Array& xforms,
                               GLUquadric* quadric,
                               const BoolArray* activeJoints) const {
    
    assert(xforms.size() == bodies.size());
    assert(!activeJoints || activeJoints->size() == joints.size());

    real l = 0.06;
    real r = 0.015;
    real m = 0.01;

    glMatrixMode(GL_MODELVIEW);


    glColor3ub(191,191,191);
    for (size_t bi=0; bi<bodies.size(); ++bi) {
      const Body& b = bodies[bi];
      if (b.haveMass) {
        glPushMatrix();
        glstuff::translate(xforms[bi] * b.comPos);
        gluSphere(quadric, m*cbrt(b.mass), 32, 24);
        glPopMatrix();
      }
    }

    // for each joint
    for (size_t ji=0; ji<joints.size(); ++ji) {
      
      const Joint& j = joints[ji];

      if (activeJoints && !(*activeJoints)[ji]) { continue; }

      size_t bi = j.offsetFromIndex;

      vec3 a = j.axis;

      real amax=0; 

      for (int i=0; i<3; ++i) {
        a[i] = fabs(a[i]);
        if (a[i] > amax) { amax = a[i]; }
      }
      
      const Transform3& t = xforms[bi];

      vec3 p0 = t.transformFwd( j.anchor - 0.5*l*j.axis );
      vec3 p1 = t.transformFwd( j.anchor + 0.5*l*j.axis );
                   
      glstuff::color(a);
      glstuff::draw_cylinder(quadric, p0, p1, r);
      
    }

    glColor3ub(0,0,0);
    glLineWidth(2.0);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    
    for (size_t ji=0; ji<joints.size(); ++ji) {

      if (activeJoints && !(*activeJoints)[ji]) { continue; }

      const Joint& j = joints[ji];


      const Transform3& to = xforms[j.offsetFromIndex];
      const Transform3& t1 = xforms[j.body1Index];
      const Transform3& t2 = xforms[j.body2Index];
      
      glstuff::vertex(t1.transformFwd(bodies[j.body1Index].comPos));
      glstuff::vertex(to.transformFwd(j.anchor));

      glstuff::vertex(to.transformFwd(j.anchor));
      glstuff::vertex(t2.transformFwd(bodies[j.body2Index].comPos));

    }

    for (size_t mi=0; mi<manipulators.size(); ++mi) {
      
      const Manipulator& m = manipulators[mi];

      size_t bi = m.effectorIndex;
      size_t ji = bodies[bi].parentJointIndex;

      if (ji>=joints.size() || (activeJoints && !(*activeJoints)[ji])) { 
        continue; 
      }

      const Transform3& tb = xforms[bi];

      glstuff::vertex(tb.transformFwd(bodies[bi].comPos));
      glstuff::vertex(tb * m.xform * vec3(0));
      
      

    }

    glEnd();

    

    glEnable(GL_LIGHTING);



  }

  void KinBody::render(const Transform3Array& xforms, 
                       const vec4& dcolor,
                       const Vec4Array* overrideColors) const {

    glMatrixMode(GL_MODELVIEW);

    for (size_t i=0; i<xforms.size(); ++i) {

      const Body& b = bodies[i];
    
      glPushMatrix();
      glstuff::mult_transform( xforms[i] );

      for (size_t j=0; j<b.geoms.size(); ++j) {

        const Geom& g = b.geoms[j];
        const TriMesh3& mesh = g.data;

        if (g.list || !mesh.empty()) {
          
          vec4 c(false);
          if (overrideColors && (*overrideColors)[i].x() >= 0) {
            c = (*overrideColors)[i];
          } else if (g.haveColor) { 
            c = g.diffuseColor;
          } else {
            c = dcolor;
          }

          glstuff::color(c);

          if (g.list) {
            
            glCallList(g.list);

          } else {

            glPushMatrix();
            glstuff::mult_transform( g.xform );
            mesh.renderGL();
            glPopMatrix();

          }

        } // if draw-worthy
        
      } // for each geom
        
      glPopMatrix();

    }

    for (size_t i=0; i<manipulators.size(); ++i) {

      const Manipulator& m = manipulators[i];
    
      size_t bi = m.effectorIndex;

      glPushMatrix();
    
      glstuff::mult_transform(xforms[bi]);
      glstuff::mult_transform(m.xform);

      drawAxes(m.drawScale);
    
      glPopMatrix();

    }

  }

  void KinBody::compileDisplayLists() {

    for (size_t i=0; i<bodies.size(); ++i) {

      Body& b = bodies[i];

      for (size_t j=0; j<b.geoms.size(); ++j) {

        Geom& g = b.geoms[j];
        const TriMesh3& mesh = g.render.empty() ? g.data : g.render;
        //const TriMesh3& mesh = g.data.empty() ? g.render : g.data;

        /*
        TriMesh3 mesh;

        if (!mesh_a.empty()) {
          Box3_t<real> bbox = mesh_a.computeBBox();
          vec3 dims = bbox.p1 - bbox.p0;
          mesh = TriMesh3::box(dims[0], dims[1], dims[2]);
          mesh.applyTransform( Transform3(0.5*(bbox.p1 + bbox.p0)) );
        }
        */

        if (!g.list && !mesh.empty()) {

          g.list = glGenLists(1);
          if (!g.list) { return; }

          glNewList(g.list, GL_COMPILE);
          glMatrixMode(GL_MODELVIEW);

          glPushMatrix();
          glstuff::mult_transform(g.xform);
          mesh.renderGL();
          glPopMatrix();

          glEndList();

          //std::cerr << "compiled display list for " << b.name << " geom " << j << "\n";

        }

      }

    }

  }

  void drawAxes(real scale) {

    glDisable(GL_LIGHTING);

    glBegin(GL_LINES);

    glColor3ub(255,0,0);
    glVertex3d(0, 0, 0);
    glVertex3d(scale, 0, 0);

    glColor3ub(0,255,0);
    glVertex3d(0, 0, 0);
    glVertex3d(0, scale, 0);

    glColor3ub(0,0,255);
    glVertex3d(0, 0, 0);
    glVertex3d(0, 0, scale);

    glEnd();

    glEnable(GL_LIGHTING);

  }

#define debug if(0) std::cerr

  bool KinBody::manipulatorIK(size_t mi,
                              const Transform3& desired,
                              RealArray& jvalues,
                              Transform3Array& xforms,
                              bool respectLimits,
                              real ptol,
                              real qtol,
                              size_t maxiter) const {

    if (ptol < 0) { ptol = DEFAULT_PTOL; }
    if (qtol < 0) { qtol = DEFAULT_QTOL; }
    if (maxiter == size_t(-1)) { maxiter = DEFAULT_ITER; }

    const Manipulator& m = manipulators[mi];

    const IndexArray& jidx = m.jointIndices;
    const BoolArray& active = m.activeBodies;

    MatX errvec(6,1), Jt, A, b, delta;
    real lambda = 1e2;

    real lastErr = -1;
    MatX lastParams, params;
  
    Transform3 fk;

    stdvec2mat(jvalues, params, jidx);
    assert((size_t)params.rows() == jidx.size() && params.cols() == 1);
    lastParams = params;

    bool limOK = true;

    if (respectLimits) { limOK = clampToLimits(params, jidx); }

    mat2stdvec(params, jvalues, jidx);

    transforms(jvalues, xforms, 0);

    bool rval = false;

#define IK_LEVENBERG_MARQUARDT
    
#ifdef IK_LEVENBERG_MARQUARDT
    Eigen::LLT<MatX> cholSolver(jidx.size());
#else
    Eigen::FullPivHouseholderQR<MatX> lstsqSolver(Jt.cols(), Jt.rows());
#endif
  
    for (size_t iter=0; iter<maxiter; ++iter) {

      debug << "at iter " << iter << "\n";

      // get the forward kinematics
      mat2stdvec(params, jvalues, jidx);
      transforms(jvalues, xforms, &active);
      fk = manipulatorFK(xforms, mi);

      // compute the error
      vec3 dp, dq;
      deltaTransform(desired, fk, dp, dq);

      real perr = dp.norm();
      real qerr = dq.norm();

      debug << "perr = " << perr << ", qerr = " << qerr << "\n";

      if (perr < ptol && qerr < qtol) {
        //std::cerr << "returning after " << iter << " iterations\n";
        rval = true; 
        break;
      } 

      if (iter + 1 == maxiter) {
        break;
      }

      dpdq2mat(dp, dq, errvec);
      debug << "errvec = " << errvec.transpose() << "\n";
      
      real err = errvec.squaredNorm();

      if (lastErr < 0) { lastErr = err; }

      debug << "err = " << sqrt(err) << ", lastErr-err = " << (lastErr-err) << "\n";
      debug << "lambda = " << lambda << "\n";

      if (err > lastErr) {
        params = lastParams;
        lambda = std::min(lambda * 10, 1e8);
        continue;
      }

      // compute the jacobian
      manipulatorJacobian(xforms, mi, Jt);

      /*
      MatX Jnt;
      manipulatorNumericalJacobian(mi, jvalues, xforms, Jnt);

      std::cout << "Jacobian max error: " << (Jnt-Jt).lpNorm<Eigen::Infinity>() << "\n";
      */


      debug << "params = " << params.transpose() << "\n";

#ifdef IK_LEVENBERG_MARQUARDT

      A = Jt * Jt.transpose();

      for (int i=0; i<A.rows(); ++i) { 
        A(i,i) *= (1 + lambda);
        //A(i,i) += lambda;
      }

      b = Jt * errvec;

      cholSolver.compute(A);
      delta = cholSolver.solve(b);


#else

      lstsqSolver.compute(Jt.transpose());
      delta = lstsqSolver.solve(errvec) * (1/lambda);

#endif
    
      lambda *= 0.5;
      lastErr = err;
      lastParams = params;
      params += delta;
    
    }

    
    if (respectLimits) { limOK = clampToLimits(params, jidx); }

    return rval && limOK;

  }

  real KinBody::totalMass() const {

    real totalMass = 0;

    for (size_t i=0; i<bodies.size(); ++i) {
      if (bodies[i].haveMass) {
        totalMass += bodies[i].mass;
      }
    }

    return totalMass;

  }
    
  void KinBody::adjustTotalMass(real fraction) {

    for (size_t i=0; i<bodies.size(); ++i) {
      bodies[i].mass *= fraction;
    }

  }

  vec3 KinBody::com(const Transform3Array& xforms) const {
    
    assert(xforms.size() == bodies.size());

    real totalMass = 0;
    vec3 rval(0);

    for (size_t i=0; i<bodies.size(); ++i) {
      
      if (bodies[i].haveMass) {
        const vec3& ci = xforms[i] * bodies[i].comPos;
        const real mi = bodies[i].mass;
        rval += mi*ci;
        totalMass += mi;
      }
      
    }

    return rval/totalMass;

  }

  void KinBody::comNumericalJacobian(RealArray& jvalues,
                                     Transform3Array& xforms,
                                     const IndexArray& jidx,
                                     MatX& Jnt,
                                     real h) const {

    assert(jvalues.size() == joints.size());
    Jnt = MatX(jidx.size(), 3);

    const real hscl = 0.5/h;

    for (size_t i=0; i<jidx.size(); ++i) {
      
      real orig = jvalues[jidx[i]];
      const real delta[3] = { -1, 1, 0 };

      vec3 c[3];
      Transform3 X;

      for (int x=0; x<3; ++x) {
        if (jidx[i] < jvalues.size()) {
          jvalues[jidx[i]] = orig + delta[x]*h;
        } else { 
          tweakDOF(X, jidx[i], delta[x]*h);
        }
        transforms(jvalues, xforms);
        c[x] = X * com(xforms);
      }

      vec3 dp = (c[1] - c[0]) * hscl;
      for (int a=0; a<3; ++a) {
        Jnt(i, a) = dp[a];
      }
      
    }


  }


  void KinBody::comJacobian(const Transform3Array& xforms,
                            const IndexArray& jointIndices,
                            MatX& Jt) const {
    
    int nj = jointIndices.size();
    if (Jt.rows() != nj || Jt.cols() != 3) { Jt = MatX(nj, 3); }
    
    // for each joint
    for (size_t j=0; j<jointIndices.size(); ++j) {

      vec3 jp_joint(0);
      real totalMass = 0;
      
      // for each body
      for (size_t i=0; i<bodies.size(); ++i) {
        const Body& b = bodies[i];
        if (b.mass) {
          vec3 jp(false);
          totalMass += b.mass;
          jointJacobian(xforms, i, b.comPos, false, jointIndices[j], &jp, 0);
          jp_joint += b.mass * jp;
        }
      }

      jp_joint /= totalMass;
      for (int a=0; a<3; ++a) {
        Jt(j,a) = jp_joint[a];
      }

    }

  }

  void KinBody::offsetBody(size_t bodyIndex, 
                           const vec3& diff) {

    assert(bodyIndex < bodies.size());


    Body& b = bodies[bodyIndex];

    b.xform.setTranslation(diff + b.xform.translation());
    b.comPos -= diff;

    for (size_t gi=0; gi<b.geoms.size(); ++gi) {
      b.geoms[gi].xform.setTranslation(b.geoms[gi].xform.translation() - diff);
    }
    for (size_t gi=0; gi<b.cgeoms.size(); ++gi) {
      b.cgeoms[gi].xform.setTranslation(b.cgeoms[gi].xform.translation() - diff);
    }

    for (size_t bi=0; bi<bodies.size(); ++bi) {
      Body& bc = bodies[bi];
      if (bc.offsetFromIndex == bodyIndex) {
        bc.xform.setTranslation(bc.xform.translation() - diff);
      }
    }

    for (size_t ji=0; ji<joints.size(); ++ji) {
      Joint& j = joints[ji];
      if (j.offsetFromIndex == bodyIndex) {
        j.anchor -= diff;
      }
    }

    for (size_t mi=0; mi<manipulators.size(); ++mi) {
      Manipulator& m = manipulators[mi];
      if (m.effectorIndex == bodyIndex) {
        m.xform.setTranslation(m.xform.translation() - diff);
      }
    }

  }

  void KinBody::offsetBody(size_t bodyIndex, 
                           const Transform3& pre) {

    // old = cur
    // new = pre * cur = cur * post
    // cur * post = pre * cur
    // post = cur_inv * pre * cur
    // post_inv = cur_inv * pre_inv * cur
    //
    // oldchld = cur * chld
    // newchld = cur * post * post_inv * chld
    //
    // oldanchor = cur * anchor
    // newanchor = cur * post * post_inv * anchor
    //
    // oldaxis = cur.rotFwd * axis
    // newaxis = (cur * post).rotFwd() * (post_inv.rotFwd() * axis)

    assert(bodyIndex < bodies.size());

    Body& b = bodies[bodyIndex];

    const Transform3 cur = b.xform;
    const Transform3 cur_inv = cur.inverse();
    const Transform3 pre_inv = pre.inverse();

    const Transform3 post_inv = cur_inv * pre_inv * cur;

    b.xform = pre * b.xform;
    b.comPos = post_inv * b.comPos;
    b.inertia = post_inv.rotFwd() * b.inertia * post_inv.rotInv();

    for (size_t gi=0; gi<b.geoms.size(); ++gi) {
      b.geoms[gi].xform = post_inv * b.geoms[gi].xform;
    }
    for (size_t gi=0; gi<b.cgeoms.size(); ++gi) {
      b.cgeoms[gi].xform = post_inv * b.cgeoms[gi].xform;
    }

    for (size_t bi=0; bi<bodies.size(); ++bi) {
      Body& bc = bodies[bi];
      if (bc.offsetFromIndex == bodyIndex) {
        bc.xform = post_inv * bc.xform;
      }
    }

    for (size_t ji=0; ji<joints.size(); ++ji) {
      Joint& j = joints[ji];
      if (j.offsetFromIndex == bodyIndex) {
        j.anchor = post_inv * j.anchor;
        j.axis = post_inv.rotFwd() * j.axis;
      }
    }

    for (size_t mi=0; mi<manipulators.size(); ++mi) {
      Manipulator& m = manipulators[mi];
      if (m.effectorIndex == bodyIndex) {
        m.xform = post_inv * m.xform;
      }
    }

  }

  void KinBody::alignJoint(size_t ji,
                           const Transform3Array& xforms,
                           const vec3& pos, // position in world coords
                           const vec3& which, // axes to move (in world frame)
                           bool restrictAxis) {

    assert(ji < joints.size());
    
    Joint& j = joints[ji];

    size_t bi = j.offsetFromIndex;
    
    const Transform3& xform = xforms[bi];

    // get the current pos of the joint in world frame
    vec3 world_anchor = xform * j.anchor;
    vec3 world_axis = xform.rotFwd() * j.axis;
    
    vec3 world_diff = pos - world_anchor;

    for (int i=0; i<3; ++i) {
      if (!which[i]) { world_diff[i] = 0; }
    }

    // get the projection perpendicular
    vec3 world_perp = world_diff - vec3::dot(world_diff, world_axis) * world_axis;

    if (restrictAxis && world_perp.norm() > 1e-9) {
      std::cerr << "error: trying to move " << j.name << " by direction " << world_diff << " which is not parallel to " << world_axis << "\n";
      exit(1);
    }
    
    j.anchor += xform.rotInv() * world_diff;
    
  }


}
