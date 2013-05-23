/**
 * @file swing.cpp
 * @author A. Huaman
 * @date 2013-03-12
 */
#include <math.h>
#include "swing.h"

#define PI 3.14159

/**
 * @function getSpline3
 * @brief Get coeff for cubic spline with start at t = 0 and end at t = tf
 */
void getSpline3( double _tf, double _x0, double _xf, double _dx0, double _dxf,
		 double &_a0, double &_a1, double &_a2, double &_a3 ) {
  
    _a0 = _x0; 
    _a1 = _dx0;
    _a2 = (3/(_tf*_tf))*(_xf - _x0) - (2/_tf)*_dx0 - (1/_tf)*_dxf;
    _a3 = -(2/(_tf*_tf*_tf))*(_xf -_x0) + (1/(_tf*_tf))*(_dxf + _dx0);
}


/**
 * @function swingSimpleCycloid
 * @brief Simple cycloid from start to goal (no vertical step). Start velocity is not 100% straight up
 * @brief Yaw changes at the same rate through the cycloid
 */
void swingSimpleCycloid( double _x0, double _y0, double _theta0,
			 double _x1, double _y1, double _theta1,
			 int _count, 
			 bool _isLeft,
                         double _maxheight,
			 double _pos[N][3],
			 double _yaw[N] ) {
  
    double d, r;
    double a, cos_a, sin_a;
    double xp_t, yp_t;
    double dtheta;
    double t1, t0, t, dT, dt;
  
    d = sqrt( pow( (_x1 - _x0), 2 ) + pow( (_y1 - _y0), 2 ) );
    a = atan2( (_y1 - _y0), (_x1 - _x0) );
    cos_a = cos(a);
    sin_a = sin(a);
  
    // consider t in [0, 2*PI]
    t0 = 0; t1 = 2*PI;
    dT = ( t1 - t0 );
  
    r = d / dT;
    dtheta = (_theta1 - _theta0) / (_count - 1);
    dt = (t1 - t0)/(_count - 1);

    // Generate the values
    for( int i = 0; i < _count; ++i ) {
    
        t = t0 + dt*i;

        xp_t = r*(t-sin(t));
        yp_t = _maxheight*(1-cos(t))/2.0;
    
        _pos[i][0] = _x0 + xp_t*cos_a;
        _pos[i][1] = _y0 + xp_t*sin_a;
        _pos[i][2] = yp_t;
        _yaw[i] = _theta0 + dtheta*i;
    
    }

}


/**
 * @function swingEllipse
 * @brief Generate Ellipse trajectory
 */
void swingEllipse( double _x0, double _y0, double _theta0,
                   double _x1, double _y1, double _theta1,
                   int _count, 
                   bool _isLeft,
                   double _maxheight,
                   double _pos[N][3],
                   double _yaw[N] ) {

    double a, b;  
    double t, t0, t1, dt;
    double xChange, yChange, yawChange, yaw, dyaw;
    double alpha, c, s;
    double l;
    int i;

    xChange = _x1 - _x0;
    yChange = _y1 - _y0;
    a = sqrt(xChange * xChange + yChange * yChange) / 2; 
    b = a / M_PI;

    yawChange = _theta1 - _theta0;
    dyaw = yawChange / (_count-1);

    t0 = M_PI;
    t1 = 0;
    dt = (t1 - t0) / (_count-1);
    t = t0;

    alpha = atan2(yChange, xChange);
    c = cos(alpha);
    s = sin(alpha);

    l = 0; 
    yaw = _theta0;

    for (i = 0; i < _count - 1; i++) {

        _pos[i][0] = _x0 + l*c;
        _pos[i][1] = _y0 + l*s;
        _pos[i][2] = b * sin(t);
        _yaw[i] = yaw;

        t += dt;
        yaw += dyaw;
        l = a * cos(t) + a;

    }

    _pos[i][0] = _x1;
    _pos[i][1] = _y1;
    _pos[i][2] = 0;
    _yaw[i] = _theta1;

}

/**
 * @function swing2Cycloids
 * @brief Swing with 2 cycloids: a) Starting and ending in vertical steps (x and y) AND b) Passing through the vertical steps (for z) 
 */
void swing2Cycloids( double _x0, double _y0, double _theta0,
		     double _x1, double _y1, double _theta1,
		     int _count, 
		     bool _isLeft,
                     double _maxheight,
		     double _pos[N][3],
		     double _yaw[N] ) {
  
    // Vertical movement parameters
    int m = 0.1*_count; // 0.1 is the % of steps used to do the vertical step.
  
    // Variables
    double d, r;
    double a, cos_a, sin_a;
    double xp_t, yp_t;
    double dtheta;
    double t, dt;
    double a0, a1, a2, a3; // for Spline
  
    // Math
    d = sqrt( pow( (_x1 - _x0), 2 ) + pow( (_y1 - _y0), 2 ) );
    a = atan2( (_y1 - _y0), (_x1 - _x0) );
    cos_a = cos(a);
    sin_a = sin(a);
  
    // z
    double tm =  2*PI*m / (_count - 1 );
    r = d / ( 2*(PI - tm + sin(tm)) );
    dt = (2*PI) / (_count - 1);

    for( int i = 0; i < _count; ++i ) {
        t = 0 + dt*i;
        _pos[i][2] = (_maxheight / 2.0 )*(1-cos(t));
        _yaw[i] = _theta0 + i*(_theta1-_theta0)/(_count-1);
    }


    // x, y and yaw
    r = d / (2*PI);
  
    dtheta = (_theta1 - _theta0) / (_count - 2*m - 1);
    dt = (2*PI) /( _count - 2*m - 1 );
    // Spline for yaw
    getSpline3( _count - 2*m - 1, _theta0, _theta1, 0, 0,
                a0, a1, a2, a3 );

    for( int i = 0; i < _count - 2*m; ++i ) {
        t = 0 + dt*i;
        xp_t = r*(t-sin(t));
        yp_t = _maxheight*(1-cos(t))/2.0;
    
        _pos[m+i][0] = _x0 + xp_t*cos_a;
        _pos[m+i][1] = _y0 + xp_t*sin_a;
        _yaw[m+i] = a3*pow(i,3) + a2*pow(i,2) + a1*i + a0;
    }



    // Cut x and y from the vertical
    for( int i = 0; i < m; ++i ) {
        _pos[i][0] = _x0;
        _pos[i][1] = _y0;
        _yaw[i] = _theta0;
    
        _pos[_count - 1 - i][0] = _x1;
        _pos[_count - 1 - i][1] = _y1;
        _yaw[_count - 1 - i] = _theta1;
    }

}

/**
 * @function swingSimpleBezier
 * @brief Bezier curve for xy, cycloid for z
 */
void swingSimpleBezier( double _x0, double _y0, double _theta0,
			double _x1, double _y1, double _theta1,
			int _count, 
			bool _isLeft,
                        double _maxheight,
			double _pos[N][3],
			double _yaw[N] ) {

    // Control location of 2 Bezier control points (p1 and p2)
    double control_percentage = 0.1;

    double d, r;
    double a, cos_a, sin_a;
    double xp_t, yp_t;
    double dtheta;
    double t1, t0, t, dT, dt;
    double b, db; // step for Bezier curve, defined in [0 1]

    d = sqrt( pow( (_x1 - _x0), 2 ) + pow( (_y1 - _y0), 2 ) );
    a = atan2( (_y1 - _y0), (_x1 - _x0) );
    cos_a = cos(a);
    sin_a = sin(a);
  
    // consider t in [0, 2*PI]
    t0 = 0; t1 = 2*PI;
    dT = ( t1 - t0 );
  
    r = d / dT;
    dtheta = (_theta1 - _theta0) / (_count - 1);
    dt = (t1 - t0)/(_count - 1);
    db = 1.0/(_count - 1);

    // Generate the values
    double px0, py0, px1, py1, px2, py2, px3, py3;
    double dist = d*control_percentage;

    px0 = _x0; 
    py0 = _y0;
 
    px1 = _x0 + dist*cos(_theta0);
    py1 = _y0 + dist*sin(_theta0);

    px2 = _x1 - dist*cos(_theta1);
    py2 = _y1 - dist*sin(_theta1);

    px3 = _x1;
    py3 = _y1;

    for( int i = 0; i < _count; ++i ) {
    
        t = t0 + dt*i;
        b = 0 + db*i;

        yp_t = _maxheight*(1-cos(t))/2.0;
    
        _pos[i][0] = pow( (1-b), 3)*px0 + 3*pow( (1-b), 2)*b*px1 + 3*(1-b)*pow(b,2)*px2 + pow(b,3)*px3;
        _pos[i][1] = pow( (1-b), 3)*py0 + 3*pow( (1-b), 2)*b*py1 + 3*(1-b)*pow(b,2)*py2 + pow(b,3)*py3;
        _pos[i][2] = yp_t;
        _yaw[i] = _theta0 + dtheta*i;
    
    }
}

/**
 * @function swingEllipse2
 * @brief Generate Ellipse trajectory, vel of x, y, z all start from zero
 */
void swingEllipse2( double _x0, double _y0, double _theta0,
                    double _x1, double _y1, double _theta1,
                    int _count, 
                    bool _isLeft,
                    double _pos[N][3],
                    double _yaw[N] ) {

    double a, b;  
    double t, t0, t1, dt;
    double xChange, yChange, yawChange, yaw, dyaw;
    double alpha, c, s;
    double l;
    int i;

    xChange = _x1 - _x0;
    yChange = _y1 - _y0;
    a = sqrt(xChange * xChange + yChange * yChange) / 2; 
    b = a / M_PI;

    yawChange = _theta1 - _theta0;
    dyaw = yawChange / (_count-1);

    t0 = 0;
    t1 = M_PI;
    dt = (t1 - t0) / (_count-1);
    t = t0;

    alpha = atan2(yChange, xChange);
    c = cos(alpha);
    s = sin(alpha);

    l = 0; 
    yaw = _theta0;

    for (i = 0; i < _count - 1; i++) {

        _pos[i][0] = _x0 + l*c;
        _pos[i][1] = _y0 + l*s;
        _pos[i][2] = b * sin( (cos(t)+1)*M_PI/2);
        _yaw[i] = yaw;

        t += dt;
        yaw += dyaw;
        l = a * cos( (cos(t)+1)*M_PI/2) + a;

    }

    _pos[i][0] = _x1;
    _pos[i][1] = _y1;
    _pos[i][2] = 0;
    _yaw[i] = _theta1;

}
