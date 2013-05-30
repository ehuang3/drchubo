/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _FAKERAVE_H_
#define _FAKERAVE_H_

#include <mzcommon/Transform3.h>
#include <mzcommon/TriMesh3.h>
#include <fstream>
#include <vector>
#include <assert.h>
#include <float.h>
#include <Eigen/Core>

namespace fakerave {

  typedef double real;
#define FR_REAL_MAX DBL_MAX
  typedef Transform3_t<real> Transform3;
  typedef vec3_t<real> vec3;
  typedef vec4_t<real> vec4;
  typedef quat_t<real> quat;
  typedef mat3_t<real> mat3;
  typedef mat4_t<real> mat4;
  typedef TriMesh3_t<real> TriMesh3;

  typedef std::vector<real> RealArray;
  typedef std::vector<bool> BoolArray;
  typedef std::vector<size_t> IndexArray;
  typedef std::vector<Transform3> Transform3Array;
  typedef std::vector<vec3> Vec3Array;
  typedef std::vector<vec4> Vec4Array;

  typedef Eigen::MatrixXd MatX;

  template <class Derived>
  inline bool isVecOfLength(const Eigen::MatrixBase<Derived>& m, int n) {
    return ( (m.rows() == n && m.cols() == 1) ||
             (m.rows() == 1 && m.cols() == n) );
  }

  template <class Derived>
  inline void mat2stdvec(const Eigen::MatrixBase<Derived>& c,
                         RealArray& a,
                         const IndexArray& idx) {

    assert( isVecOfLength(c, idx.size()) );

    for (size_t i=0; i<idx.size(); ++i) {
      a[idx[i]] = c(i);
    }

  }

  // Note: this uses the const cast trick from
  // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template <class Derived>
  inline void stdvec2mat(const RealArray& a,
                         const Eigen::MatrixBase<Derived>& cc,
                         const IndexArray& idx) {

    assert( isVecOfLength(cc, int(idx.size())) );
    
    Eigen::MatrixBase<Derived>& c = const_cast<Eigen::MatrixBase<Derived>&>(cc);

    for (size_t i=0; i<idx.size(); ++i) {
      c(i) = a[idx[i]];
    }

  }  

  inline void stdvec2mat(const RealArray& a,
                         MatX& c,
                         const IndexArray& idx) {

    if (!isVecOfLength(c, int(idx.size()))) { c = MatX(idx.size(), 1); }

    for (size_t i=0; i<idx.size(); ++i) {
      c(i) = a[idx[i]];
    }

  }  

  inline void deltaTransform(const Transform3& td, 
                             const Transform3& tc,
                             vec3& dp, vec3& dq) {
    dp = td.translation() - tc.translation();
    dq = quat::omega(tc.rotation(), td.rotation());
  }

  // Note: this uses the const cast trick from
  // http://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template <class Derived>
  inline void dpdq2mat(const vec3& dp, const vec3& dq, 
                       const Eigen::MatrixBase<Derived>& mm) {

    assert( isVecOfLength(mm, 6) );

    Eigen::MatrixBase<Derived>& m = const_cast<Eigen::MatrixBase<Derived>&>(mm);

    for (int d=0; d<3; ++d) {
      m(d+0) = dp[d];
      m(d+3) = dq[d];
    }

  }


  //////////////////////////////////////////////////////////////////////

  class KinBody;
  class Body;

  class Geom {
  public:

    Transform3 xform;
 
    std::string type;

    std::string renderFile;
    real renderScale;

    std::string dataFile;
    real dataScale;

    real radius;
    real height;
    vec3 boxExtents;

    bool haveColor;
    vec4 diffuseColor;

    TriMesh3 render;
    TriMesh3 data;

    unsigned int list;

    Geom();

  };

  typedef std::vector<Geom> GeomArray;

  //////////////////////////////////////////////////////////////////////

  class Body {
  public:

    std::string name;
    std::string offsetFromName;

    size_t offsetFromIndex;
    size_t parentJointIndex;

    mat3 inertia;

    Transform3 xform;

    real mass;
    vec3 comPos;

    bool haveMass;
    bool haveInertia;

    GeomArray geoms;
    GeomArray cgeoms;

    Body();
  
  };

  typedef std::vector<Body> BodyArray;

  //////////////////////////////////////////////////////////////////////

  class Joint {
  public:

    std::string name;
    std::string type;

    std::string body1Name;
    std::string body2Name;
    std::string offsetFromName;

    size_t body1Index;
    size_t body2Index;
    size_t offsetFromIndex;

    vec3 anchor;
    vec3 axis;
    real limits[2];

    bool haveAnchor;
    bool haveAxis;
    bool haveLimits;

    Joint();

  };

  typedef std::vector<Joint> JointArray;

  //////////////////////////////////////////////////////////////////////
  
  class Manipulator {
  public:

    Transform3 xform;

    std::string name;
    std::string effectorName;
    std::string baseName;

    IndexArray jointIndices;
    BoolArray activeBodies;
    
    real drawScale;

    size_t effectorIndex;
    size_t baseIndex;

    Manipulator();
    
  };

  typedef std::vector<Manipulator> ManipulatorArray;

  //////////////////////////////////////////////////////////////////////

  enum DofIndex {
    DOF_POS_X   = size_t(-7),
    DOF_POS_Y   = size_t(-6),
    DOF_POS_Z   = size_t(-5),
    DOF_ROT_X   = size_t(-4),
    DOF_ROT_Y   = size_t(-3),
    DOF_ROT_Z   = size_t(-2),
    DOF_INVALID = size_t(-1),
  };

  class KinBody {
  public:

    std::string filename;
    std::string directory;
    std::string modelsdir;
    std::string prefix;

    BodyArray bodies;
    JointArray joints;
    ManipulatorArray manipulators;

    IndexArray bodyOrder;

    real DEFAULT_PTOL;
    real DEFAULT_QTOL;
    size_t   DEFAULT_ITER;

    KinBody();

    //////////////////////////////////////////////////

    static bool clampAngle(real lo, real hi, real& a);
    
    bool clampToLimits(RealArray& jvalues) const;
    
    bool clampToLimits(MatX& jmat, const IndexArray& jidx) const;

    //////////////////////////////////////////////////

    void loadXML(const std::string& filename);

    void setFilename(const std::string& filename);

    size_t lookupBody(const std::string& name) const;

    size_t lookupJoint(const std::string& name) const;

    size_t lookupManipulator(const std::string& name) const;

    bool isAncestorOf(size_t rootIndex, size_t leafIndex) const;

    bool bodyDependsOnJoint(const size_t bodyIndex,
                            const size_t jointIndex) const;


    void resolve(size_t& i, const std::string& n, bool allowBlank);

    void resolve();

    vec3 jointAnchor(const Transform3Array& xforms,
                     size_t jointIndex) const;
    
    vec3 jointAxis(const Transform3Array& xforms,
                   size_t jointIndex) const;
               
    void transforms(const RealArray& jvalues,
                    Transform3Array& xforms,
                    const BoolArray* active=0) const;
    
    // jacobian with respect to a single joint
    void jointJacobian(const Transform3Array& xforms,
                       size_t bodyIndex,
                       const vec3& point,      // only used for jpos
                       bool pointInWorldFrame, // only used for jpos
                       size_t jointIndex,
                       vec3* jpos,
                       vec3* jrot) const;
    
    // get full jacobian
    void jacobian(const Transform3Array& xforms,
                  size_t bodyIndex,
                  const vec3& point,
                  bool pointInWorldFrame,
                  const IndexArray& jointIndices,
                  MatX& Jtranspose) const;

    void numericalJacobian(RealArray& jvalues,
                           Transform3Array& xforms,
                           size_t bodyIndex,
                           const vec3& point,
                           bool pointInWorldFrame,
                           const IndexArray& jointIndices,
                           MatX& Jtranspose,
                           real h=1e-3) const;

    Transform3 manipulatorFK(const Transform3Array& xforms,
                             size_t manipIndex) const;
    
    void manipulatorJacobian(const Transform3Array& xforms,
                             size_t mi,
                             MatX& Jtranspose) const;

    void manipulatorJacobian(const Transform3Array& xforms,
                             size_t mi,
                             const IndexArray& jointIndices,
                             MatX& Jtranspose) const;

    void manipulatorNumericalJacobian(size_t mi,
                                      RealArray& jvalues,
                                      Transform3Array& xforms,
                                      MatX& Jtranspose, 
                                      real h=1e-3) const;

    void centerJoints(const IndexArray& jidx,
                      RealArray& jvalues) const;

    void zeroJoints(const IndexArray& jidx,
                    RealArray& jvalues) const;

    bool manipulatorIK(size_t mi,
                       const Transform3& desired,
                       RealArray& jvalues,
                       Transform3Array& xforms,
                       bool respectLimits=true,
                       real ptol=-1,
                       real qtol=-1,
                       size_t maxiter=-1) const;

    void offsetBody(size_t bodyIndex, 
                    const Transform3& rel);

    void offsetBody(size_t bodyIndex, 
                    const vec3& diff);


    void alignJoint(size_t jointIndex,
                    const Transform3Array& xforms,
                    const vec3& pos,
                    const vec3& which,
                    bool restrictAxis);

    
    // render in opengl
    void render(const Transform3Array& xforms,
                const vec4& defaultColor=vec4(0.5,0.5,0.5,1),
                const Vec4Array* overrideColors=0) const;

    void renderSkeleton(const Transform3Array& xforms,
                        GLUquadric* quadric,
                        const BoolArray* activeJoints=0) const;

    vec3 com(const Transform3Array& xforms) const;

    real totalMass() const;
    
    void adjustTotalMass(real fraction);

    void comJacobian(const Transform3Array& xforms, 
                     const IndexArray& jointIndices,
                     MatX& Jt) const;

    void comNumericalJacobian(RealArray& jvalues,
                              Transform3Array& xforms,
                              const IndexArray& jointIndices,
                              MatX& Jtranspose, 
                              real h=1e-3) const;


    // create display lists for items
    void compileDisplayLists();

  };

  void drawAxes(real scale);

  class BodyLookup {
  public:
    const KinBody& kbody;
    BodyLookup(const KinBody& k): kbody(k) {}
    size_t operator()(const std::string& name) const {
      return kbody.lookupBody(name);
    }
  };

  class JointLookup {
  public:
    const KinBody& kbody;
    JointLookup(const KinBody& k): kbody(k) {}
    size_t operator()(const std::string& name) const {
      return kbody.lookupJoint(name);
    }
  };

  class ManipulatorLookup {
  public:
    const KinBody& kbody;
    ManipulatorLookup(const KinBody& k): kbody(k) {}
    size_t operator()(const std::string& name) const {
      return kbody.lookupManipulator(name);
    }
  };


};


#endif
