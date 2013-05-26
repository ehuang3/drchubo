#include <stdlib.h>
#include <math.h>
#include <vector>
#include "zmp/footprint.h"
#include <iostream>
#include "mzcommon/quat.h"

using namespace std;
using namespace fakerave;

Footprint::Footprint(Transform3 t, bool is_left) {
    this->transform = t;
    this->is_left = is_left;
}
Footprint::Footprint(double x, double y, double theta, bool is_left) {
    vec3 translation(x, y, 0);
    Transform3 transform(quat::fromAxisAngle(vec3(0.0,0.0,1.0), theta), vec3(x, y, 0));
    this->transform = transform;
    this->is_left = is_left;
}

double Footprint::x() const { return this->transform.translation().x(); } 
double Footprint::y() const{ return this->transform.translation().y(); } 
double Footprint::theta() const {
    vec3 forward = this->transform.rotFwd() * vec3(1.0, 0.0, 0.0);
    return atan2(forward.y(), forward.x());
} 


Footprint::Footprint(){
    Footprint( 0.0, 0.0, 0.0, 0.0 );
}

Transform3 Footprint::getTransform3() const {

    return transform;
}

void Footprint::setTransform3(Transform3 transform){
    this->transform = transform;
}

bool is_even(int i) {
    return (i%2) == 0;
}

bool is_odd(int i) {
    return !is_even(i);
}

vector<Footprint> walkLine(double dist,
                           double width,
                           double max_step_length,
                           Footprint* init_left,
                           Footprint* init_right,
                           bool left_is_stance_foot) {
    // FIXME: line should not start with two side-by-side steps

    assert(dist > 0);
    assert(width > 0);
    assert(max_step_length > 0);
    assert((left_is_stance_foot ? init_left : init_right)
           && "The stance foot must not be null");
    
    // select stance foot, fill out transforms
    Footprint* stance_foot;
    if (left_is_stance_foot) stance_foot = init_left;
    else stance_foot = init_right;
    Transform3 T_line_to_world =
        stance_foot->transform
        * Transform3(quat(), vec3(0, left_is_stance_foot?-width:width, 0));

    // find K, N, and L simple equation
    const int K = int(ceil(dist/max_step_length) + 1e-10);
    const double L = dist/K;
    const int N = K+3;

    // build result vector
    vector<Footprint> result(N);

    // Do all steps
    for (int i = 0; i < N; i++) {
        bool is_left = (is_even(i) == left_is_stance_foot);
        double x = (i-1)*L;
        double y = width * is_left ? width : -width;
        result[i] = Footprint(x, y, 0, is_left);
    }

    result[1] = Footprint(0, result[1].y(), 0, result[1].is_left);
    result[N-1] = Footprint(dist, result[N-1].y(), 0, result[N-1].is_left);
    result[N-2] = Footprint(dist, result[N-2].y(), 0, result[N-2].is_left);

    // run through results transforming them back into the original frame of reference
    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        it->transform = T_line_to_world * it->transform;
    }
    result[0] = Footprint(*stance_foot);

    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        std::cout << *it << std::endl;
    }

    return result;
}

vector<Footprint> walkCircle(double radius,
                             double distance,
                             double width,
                             double max_step_length,
                             double max_step_angle,
                             Footprint* init_left,
                             Footprint* init_right,
                             bool left_is_stance_foot) {
    assert(distance > 0);
    assert(width > 0);
    assert(max_step_length > 0);
    assert(max_step_angle >= -3.14159265359);
    assert(max_step_angle < 3.14159265359);
    assert((left_is_stance_foot ? init_left : init_right)
           && "The stance foot must not be null");

    // select stance foot, fill out transforms
    Footprint* stance_foot;
    if (left_is_stance_foot) stance_foot = init_left;
    else stance_foot = init_right;
    Transform3 T_circle_to_world =
        stance_foot->transform
        * Transform3(quat(), vec3(0, left_is_stance_foot?-width:width, 0));

    double alpha = distance / abs(radius);
    double outer_dist = (abs(radius) + width) * alpha;
    int K_step_length = ceil(outer_dist / max_step_length);
    int K_angle = ceil(alpha / max_step_angle);
    int K = max(K_step_length, K_angle);
    double dTheta = alpha/K * radius/(abs(radius));

#ifdef DEBUG
    cout << "outer_dist IS " << outer_dist << endl;
    cout << outer_dist / max_step_length << endl;
    cout << "Ksl IS " << K_step_length << endl;
    cout << "Kang IS " << K_angle << endl;
    cout << "K IS " << K << endl;
    cout << "dTheta IS " << dTheta << endl;
#endif

    // init results list
    vector<Footprint> result;

    // fill out results
    for(int i = 2; i < K + 1; i++) {
        double theta_i = dTheta * (i - 1);
        if (is_even(i) xor left_is_stance_foot) { // i odd means this step is for the stance foot
            result.push_back(Footprint((radius - width) * sin(theta_i),
                                       radius - ((radius - width) * cos(theta_i)),
                                       theta_i,
                                       true));
        }
        else {
            result.push_back(Footprint((radius + width) * sin(theta_i),
                                       radius - ((radius + width) * cos(theta_i)),
                                       theta_i,
                                       false));
        }
    }

    // fill out the last two footsteps
    double theta_last = dTheta * K;
    if (is_even(K) xor left_is_stance_foot) { // K even means we end on the stance foot
        result.push_back(Footprint((radius + width) * sin(theta_last),
                                   radius - ((radius + width) * cos(theta_last)),
                                   theta_last,
                                   false));
        result.push_back(Footprint((radius - width) * sin(theta_last),
                                   radius - ((radius - width) * cos(theta_last)),
                                   theta_last,
                                   true));
    }
    else {
        result.push_back(Footprint((radius - width) * sin(theta_last),
                                   radius - ((radius - width) * cos(theta_last)),
                                   theta_last,
                                   true));
        result.push_back(Footprint((radius + width) * sin(theta_last),
                                   radius - ((radius + width) * cos(theta_last)),
                                   theta_last,
                                   false));
    }

    // run through results transforming them back into the original frame of reference
    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        it->transform = T_circle_to_world * it->transform;
    }
    result.insert(result.begin(), Footprint(*stance_foot));

    // return the result
    return result;
}
