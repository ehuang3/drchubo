#include "fakerave.h"
#include <mzcommon/TinyDom.h>
#include <mzcommon/strutils.h>
#include <memory>
#include <stdexcept>

#define warning if (0) std::cerr << "warning: "

namespace {

  using namespace fakerave;

  std::string lname(const TinyDomElement* e) { 
    return lower(e->name());
  }


  Transform3 compose1(const Transform3& t2, 
                      const Transform3& t1) {
    
    return Transform3( t2.rotation() * t1.rotation(),
                       t2.translation() + t1.translation() );

  }

  void handleModel(const TinyDomElement* e, 
                   const std::string& dir,
                   std::string& file,
                   real& scale,
                   TriMesh3& mesh) {

    assert(e->children().size() == 1);
    const TinyDomCharacterData* c = 
      dynamic_cast<const TinyDomCharacterData*>(*e->children().begin());
    assert(c);

    std::istringstream istr(c->value);

    scale = 1;

    istr >> file;
    assert(file.length());

    if (!istr.peek() != EOF) {
      istr >> scale;
      assert(istr);
    }

    std::string fullFile = combineDir(dir, file);
    size_t pos = fullFile.rfind('.');
    if (pos < fullFile.length()) {
      std::string ext = lower(fullFile.substr(pos+1, fullFile.length()-pos-1));
      if (ext == "obj") {
        mesh.parseObj(fullFile);
      } else if (ext == "wrl") {
        mesh.parseWrl(fullFile);
      }
    }
      

    if (file.length() && mesh.empty()) {
      warning << "failed to parse " << file << "\n";
    }

  }


  template <class Tval>
  void handleNumbers(const TinyDomElement* e, Tval* data, int len) {

    std::string s;

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomCharacterData* c = 
        dynamic_cast<const TinyDomCharacterData*>(*i);

      if (c) { s += c->value + " "; }
    
    }

    std::istringstream istr(s);

    for (int i=0; i<len; ++i) {
      istr >> data[i];
      assert( istr );
    }

  }

  vec3 handleTranslation(const TinyDomElement* e) {
    vec3 rval;
    handleNumbers(e, rval.v, 3);
    return rval;
  }

  quat handleRotationAxis(const TinyDomElement* e) {
    real xyza[4];
    handleNumbers(e, xyza, 4);
    return quat::fromAxisAngle(vec3(xyza[0], xyza[1], xyza[2]), xyza[3]*M_PI/180);
  }

  quat handleRotationMat(const TinyDomElement* e) {
    mat3 m;
    handleNumbers(e, m.data, 9);
    return quat::fromMat3(m);
  }

  quat handleQuat(const TinyDomElement* e) {
    real wxyz[4];
    handleNumbers(e, wxyz, 4);
    return quat(wxyz[1], wxyz[2], wxyz[3], wxyz[0]);
  }

  bool handleTransform(const TinyDomElement* e, Transform3& t) {

    if (lname(e) == "translation") {

      t = compose1( Transform3(handleTranslation(e)), t );
      return true;

    } else if (lname(e) == "rotationaxis") {

      t = compose1( Transform3(handleRotationAxis(e)), t );
      return true;
    
    } else if (lname(e) == "rotationmat") {
    
      t = compose1( Transform3(handleRotationMat(e)), t );
      return true;
    
    } else if (lname(e) == "quat") {
    
      t = compose1( Transform3(handleQuat(e)), t );
      return true;

    } else {

      return false;

    }

  }

  std::string handleString(const TinyDomElement* e) {
    assert(e->children().size() == 1);
    const TinyDomCharacterData* c = 
      dynamic_cast<const TinyDomCharacterData*>(*e->children().begin());
    assert(c);
    return c->trimmedValue();
  }

  void handleJoints(const TinyDomElement* e,
                    IndexArray& jointIndices,
                    const KinBody& k) {

    std::string s = handleString(e);
    
    std::istringstream istr(s);
    std::string jname;

    jointIndices.clear();

    while ( (istr >> jname) ) {
      size_t ji = k.lookupJoint(jname);
      assert(ji < k.joints.size());
      assert(k.joints[ji].name == jname);
      jointIndices.push_back(ji);
    }

  }

  //////////////////////////////////////////////////////////////////////

  void handleMass(const TinyDomElement* e, Body& b) {
  
    assert(lname(e) == "mass");
  
    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {
    
      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);
      if (!c) { continue; }

      std::string cname = lname(c);

      if (cname == "total") {
        assert(!b.haveMass);
        handleNumbers(c, &b.mass, 1);
        b.haveMass = true;
      } else if (cname == "inertia") {
        assert(!b.haveInertia);
        handleNumbers(c, b.inertia.data, 9);
        b.haveInertia = true;
      } else {
        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";
      }

    }

  }





  //////////////////////////////////////////////////////////////////////

  void handleGeom(const TinyDomElement* e,
                  const KinBody& k, 
                  Geom& g) {

    g.type = lower(e->attribute("type"));

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);

      if (!c) { continue; }

      std::string cname = lname(c);

      if (g.type == "trimesh" && cname == "render") {
        handleModel(c, k.modelsdir, g.renderFile, g.renderScale, g.render);
      } else if (g.type == "trimesh" && cname == "data") {
        handleModel(c, k.modelsdir, g.dataFile, g.dataScale, g.data);
      } else if ((g.type == "cylinder" || g.type == "sphere") && 
                 cname == "radius") {
        handleNumbers(c, &g.radius, 1);
      } else if (g.type == "cylinder" && cname == "height") {
        handleNumbers(c, &g.height, 1);
      } else if (g.type == "box" && cname == "extents") {
        handleNumbers(c, g.boxExtents.v, 3);
      } else if (cname == "diffusecolor") {
        handleNumbers(c, g.diffuseColor.v, 3);
        g.diffuseColor[3] = 1;
        g.haveColor = true;
        //std::cerr << "geom of type " << g.type << " has color " << g.diffuseColor << "\n";
      } else if (handleTransform(c, g.xform)) {
        //std::cerr << "  transform for " << g.type << " is now " << g.xform << "\n";
      } else if (cname == "donotrender") {
        // ignore for now
      } else {
        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";
      }

    }

  
  
  }

  void handleCGeom(const TinyDomElement* e,
                   Geom& g) {

    g.type = lower(e->attribute("type"));

    if (g.type == "auto_aabb") {
    } else if (g.type == "auto_sphere") {
    } else if (g.type == "cylinder") {
    } else if (g.type == "sphere") {
    } else if (g.type == "box") {
    } else {
      std::cerr << "invalid cgeom type " << g.type << "\n";
      exit(1);
    }

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);

      if (!c) { continue; }

      std::string cname = lname(c);

      if ((g.type == "cylinder" || g.type == "sphere") && cname == "radius") {
        handleNumbers(c, &g.radius, 1);
      } else if (g.type == "cylinder" && cname == "height") {
        handleNumbers(c, &g.height, 1);
      } else if (g.type == "box" && cname == "extents") {
        handleNumbers(c, g.boxExtents.v, 3);
      } else if (cname == "diffusecolor") {
        handleNumbers(c, g.diffuseColor.v, 3);
        g.diffuseColor[3] = 1;
        g.haveColor = true;
        //std::cerr << "geom of type " << g.type << " has color " << g.diffuseColor << "\n";
      } else if (handleTransform(c, g.xform)) {
        //std::cerr << "  transform for " << g.type << " is now " << g.xform << "\n";
      } else {
        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";
      }

    }

    
  }

  //////////////////////////////////////////////////////////////////////

  void handleBody(const TinyDomElement* e, KinBody& k) {

    assert(lname(e) == "body");

    std::string name = k.prefix + e->attribute("name");

    assert(!name.empty());

    size_t i = k.lookupBody(name);
    if (i >= k.bodies.size()) { 
      i = k.bodies.size();
      k.bodies.push_back(Body());
      k.bodies.back().name = name;
    }

    assert( i < k.bodies.size() && k.bodies[i].name == name );
  
    Body& b = k.bodies[i];

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);

      if (!c) { continue; }

      std::string cname = lname(c);

      if (cname == "mass") {
        handleMass(c, b);
      } else if (cname == "offsetfrom") {
        assert(b.offsetFromName.empty());
        b.offsetFromName = k.prefix + handleString(c);
      } else if (cname == "geom") {
        b.geoms.push_back(Geom());
        handleGeom(c, k, b.geoms.back());
      } else if (cname == "cgeom") {
        b.cgeoms.push_back(Geom());
        handleCGeom(c, b.cgeoms.back());
      } else if (handleTransform(c, b.xform)) {
        //std::cerr << "transform for " << name << " is now " << b.xform << "\n";
      } else {
        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";
      }

    }


  }

  //////////////////////////////////////////////////////////////////////

  void handleJoint(const TinyDomElement* e, KinBody& k) {

    std::string name = k.prefix + e->attribute("name");

    assert(!name.empty());

    std::string enable = e->attribute("enable");
    if (!enable.empty() && enable == "false") {
      warning << "ignoring disabled joint " << name << "\n";
      return;
    }
 
    size_t i = k.lookupJoint(name);

    if (i >= k.joints.size()) { 
      i = k.joints.size();
      k.joints.push_back(Joint());
      k.joints.back().name = name;
    }

    assert(i < k.joints.size() && k.joints[i].name == name);

    Joint& j = k.joints[i];

    j.name = name;
    j.type = e->attribute("type");

    assert(!j.name.empty());
    assert(j.type == "hinge");

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);

      if (!c) { continue; }

      std::string cname = lname(c);

      if (cname == "body") {
        if (j.body1Name.empty()) {
          j.body1Name = k.prefix + handleString(c);
        } else {
          assert(j.body2Name.empty());
          j.body2Name = k.prefix + handleString(c);
        }
      } else if (cname == "offsetfrom") {
        assert(j.offsetFromName.empty());
        j.offsetFromName = k.prefix + handleString(c);
      } else if (cname == "anchor") {
        assert(!j.haveAnchor);
        handleNumbers(c, j.anchor.v, 3);
        j.haveAnchor = true;
      } else if (cname == "axis") {
        assert(!j.haveAxis);
        handleNumbers(c, j.axis.v, 3);
        j.haveAxis = true;
      } else if (cname == "limitsdeg") {
        assert(!j.haveLimits);
        handleNumbers(c, j.limits, 2);
        j.limits[0] *= M_PI/180;
        j.limits[1] *= M_PI/180;
        j.haveLimits = true;
      } else if (cname == "maxtorque" ||
                 cname == "maxvel" ||
                 cname == "maxaccel") {
        // NOP
      } else {
        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";
      }

    }

  }

  //////////////////////////////////////////////////////////////////////

  void handleManipulator(const TinyDomElement* e, KinBody& k) {

    std::string name = k.prefix + e->attribute("name");

    assert(!name.empty());
 
    size_t i = k.lookupManipulator(name);

    if (i >= k.manipulators.size()) { 
      i = k.manipulators.size();
      k.manipulators.push_back(Manipulator());
      k.manipulators.back().name = name;
    }

    assert(i < k.manipulators.size() && k.manipulators[i].name == name);

    Manipulator& m = k.manipulators[i];

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);

      if (!c) { continue; }

      std::string cname = lname(c);

      if (cname == "effector") {
        assert(m.effectorName.empty());
        m.effectorName = k.prefix + handleString(c);
      } else if (cname == "base") {
        assert(m.baseName.empty());
        m.baseName = k.prefix + handleString(c);
      } else if (cname == "drawscale") {
        handleNumbers(c, &m.drawScale, 1);
      } else if (cname == "joints") {
        handleJoints(c, m.jointIndices, k);
      } else if (handleTransform(c, m.xform)) {
        // nop
      } else {

        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";

      }

    }

  }

  //////////////////////////////////////////////////////////////////////
  
  void handleKinbody(const TinyDomElement* e, 
                     KinBody& k) {


    assert(lname(e) == "kinbody");

    Transform3 localXform;

    for (TinyDomElement::ChildList::const_iterator i=e->children().begin(); 
         i!=e->children().end(); ++i) {

      const TinyDomElement* c = dynamic_cast<const TinyDomElement*>(*i);

      if (!c) { continue; }

      std::string cname = lname(c);

      if (cname == "body") {

        handleBody(c, k);

      } else if (cname == "joint") {

        handleJoint(c, k);

      } else if (cname == "manipulator") {

        handleManipulator(c, k);

      } else if (cname == "modelsdir") {

        k.modelsdir = combineDir(k.directory, handleString(c));
      
      } else if (cname == "kinbody") {

        std::string p2 = c->attribute("prefix");

        std::string f2 = combineDir(k.directory, c->attribute("file"));

        std::ifstream istr(f2.c_str());

        if (!istr.is_open()) {
          std::cerr << "error: failed to open " << f2 << "\n";
          exit(1);
        }

        std::auto_ptr<TinyDomElement> f(TinyDom::parse(istr));

        KinBody k2;
        k2.setFilename(f2);
        k2.modelsdir = k.modelsdir;
        k2.prefix = k.prefix + p2;

        handleKinbody(f.get(), k2);

        handleKinbody(c, k2);
      
        k.bodies.insert(k.bodies.end(), k2.bodies.begin(), k2.bodies.end());
        k.joints.insert(k.joints.end(), k2.joints.begin(), k2.joints.end());
        k.manipulators.insert(k.manipulators.end(), k2.manipulators.begin(), k2.manipulators.end());
        

      } else if (handleTransform(c, localXform)) {

        //std::cerr << "transform for " << k.filename << " is now " << k.xform << "\n";

      } else if (cname == "adjacent") {

        // NOP

      } else {

        warning << "ignoring " << e->name() << " -> " << c->name() << "\n";

      }
    
    }

    for (size_t i=0; i<k.bodies.size(); ++i) {
      if (k.bodies[i].offsetFromName.empty()) {
        k.bodies[i].xform = localXform * k.bodies[i].xform;
      }
    }


  
  }

}

namespace fakerave {

  void KinBody::loadXML(const std::string& f) {

    setFilename(f);
    
    filename = f;
    directory = directoryOf(f);
    modelsdir = directory;

    std::ifstream istr(filename.c_str());
    std::auto_ptr<TinyDomElement> e(TinyDom::parse(istr));

    handleKinbody(e.get(), *this);

    resolve();

  }

}
