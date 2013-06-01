#ifndef  FOOTPRINT_INC
#define  FOOTPRINT_INC

#include <vector>
#include <iostream>
#include "mzcommon/Transform3.h"
#include "src/fakerave.h"

using namespace std;
using namespace fakerave;

/**
 * \struct Footprint
 * \brief Contains the 2D position and orientation of a foot placement, plus whether this location refers to the left or right foot.
 */
class Footprint {
public:
  Transform3 transform;
  bool is_left;

  Footprint(Transform3 t, bool is_left);
  Footprint(double x, double y, double theta, bool is_left);
  Footprint();

  double x() const;
  double y() const;
  double theta() const;

  Transform3 getTransform3() const;
  void setTransform3(Transform3 transform);

  friend ostream& operator<<(ostream& os, const Footprint& fp) {
    os << "x: " << fp.x() << "\ty: " << fp.y() << "\ttheta: " << fp.theta() << "\t " << (fp.is_left ? "Left" : "Right");
    return os;
  }
};

/**
 * \fn walkLine
 * \brief Generates a foot plan for walking in a straight line (parallel to the robot's current orientation)
 */
std::vector<Footprint> walkLine(double dist, /// The distance to walk in meters
                                double width, /// The ground distance between the center of the robot to the center of a foot
                                double max_step_length, /// The maximum allowed length the robot may step
                                Footprint* init_left, /// The position the left foot starts in
                                Footprint* init_right, /// The position the right foot starts in
                                bool stance_handedness /// Which foot the robot will be starting off standing on
  );

/**
 * \fn walkCircle
 * \brief Generates a foot plan for walking in a circular arc of a given radius.
 */
std::vector<Footprint> walkCircle(double radius, /// The radius of the circle to walk in
                                  double distance, /// The distance to walk along the circle
                                  double width, /// The distance between the center of the robot and the center of a foot
                                  double max_step_length, /// The maximum HALF allowed length the robot may step
                                  double max_step_angle, /// The maximum HALF angle between successive steps
                                  Footprint* init_left, /// The position the left foot starts in
                                  Footprint* init_right, /// The position the right foot starts in
                                  bool stance_handedness /// Which foot the robot will be starting off standing on
  );

#endif   /* ----- #ifndef FOOTPRINT_INC  ----- */

/* Local Variables: */
/* mode: c++ */
/* c-basic-offset: 2 */
/* End: */
