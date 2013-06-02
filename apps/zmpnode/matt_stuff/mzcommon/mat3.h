/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/
#ifndef _MAT3_H_
#define _MAT3_H_

#include "vec3.h"

/** Encapsulate a 3-by-3 matrix used for transforming 3D
 *  coordinates. Data is stored in a row-major format. */
template <class real> class mat3_t {
public:

  /** Enumerations for row-major matrix element access. */
  enum {
    /** Indexes the (0,0) element. */
    XX=0, 
    /** Indexes the (0,1) element. */
    XY,   
    /** Indexes the (0,2) element. */
    XZ,
    /** Indexes the (1,0) element. */
    YX,   
    /** Indexes the (1,1) element. */
    YY,   
    /** Indexes the (1,2) element. */
    YZ,
    /** Indexes the (2,0) element. */
    ZX,   
    /** Indexes the (2,1) element. */
    ZY,   
    /** Indexes the (2,2) element. */
    ZZ
  };

  /** Default constructor initializes to identity matrix. */
  mat3_t() { *this = identity(); }

  /** Construct with optional initialization.
   *
   *  @param init If this is true, then initialize to the identity
   *              matrix, otherwise, leave data uninitialized.
   */
  mat3_t(bool init) { if (init) { *this = identity(); } }

  /** Copy from row-major 3-by-3 matrix. */
  mat3_t(const real src[9]) { memcpy(data, src, 9*sizeof(real)); }

  /** Copy constructor. */
  mat3_t(const mat3_t& src) { memcpy(data, src.data, 9*sizeof(real)); }

  /** Assignment operator. */
  mat3_t& operator=(const mat3_t& src) { 
    memcpy(data, src.data, 9*sizeof(real));
    return *this;
  }

  /** Return the transpose of this matrix. */
  mat3_t& transpose(mat3_t& m) const {
    const int tidx[9] = {
      0, 3, 6, 
      1, 4, 7, 
      2, 5, 8, 
    };
    for (int i=0; i<9; ++i) {
      m.data[i] = data[tidx[i]];
    }
    return m;
  };

  /** Return the transpose of this matrix. */
  mat3_t transpose() const {
    mat3_t rval(false);
    return transpose(rval);
  }

  /** Return the identity matrix. */
  static mat3_t identity() {
    const real id[9] = {
      1.0, 0.0, 0.0, 
      0.0, 1.0, 0.0, 
      0.0, 0.0, 1.0, 
    };
    return mat3_t(id);
  };

  /** Row/column access */

  vec3_t<real> row(size_t i) const {
    return vec3_t<real>(data[3*i+0], data[3*i+1], data[3*i+2]);
  }
  vec3_t<real> col(size_t i) const {
    return vec3_t<real>(data[i+0], data[i+3], data[i+6]);
  }

  void setRow(size_t i, const vec3_t<real>& v) {
    data[3*i+0] = v[0];
    data[3*i+1] = v[1];
    data[3*i+2] = v[2];
  }

  void setCol(size_t i, const vec3_t<real>& v) {
    data[i+0] = v[0];
    data[i+3] = v[1];
    data[i+6] = v[2];
  }

  /** Elementwise access. */
  const real& operator[](size_t i) const {
    return data[i];
  }

  /** Subscript access. */
  const real& operator()(size_t row, size_t col) const {
    return data[row*3 + col];
  }

  /** Elementwise access. */
  real& operator[](size_t i)  {
    return data[i];
  }
  
  /** Subscript access. */
  real& operator()(size_t row, size_t col)  {
    return data[row*3 + col];
  }

  /** Determinant */
  real determinant() const {
    return  (data[XX] * (data[YY] * data[ZZ] - data[YZ] * data[ZY]) +
             data[XY] * (data[YZ] * data[ZX] - data[YX] * data[ZZ]) +
             data[XZ] * (data[YX] * data[ZY] - data[YY] * data[ZX]));
  }
  
  /** Inverse. Determinant should be returned by method above. */
  mat3_t& inverse(mat3_t& m, real det) const {
    real invDet = 1.0/det;
    m[XX] = (data[YY] * data[ZZ] - data[YZ] * data[ZY]) * invDet;
    m[XY] = (data[XZ] * data[ZY] - data[XY] * data[ZZ]) * invDet;
    m[XZ] = (data[XY] * data[YZ] - data[XZ] * data[YY]) * invDet;
    m[YX] = (data[YZ] * data[ZX] - data[YX] * data[ZZ]) * invDet;
    m[YY] = (data[XX] * data[ZZ] - data[XZ] * data[ZX]) * invDet;
    m[YZ] = (data[XZ] * data[YX] - data[XX] * data[YZ]) * invDet;
    m[ZX] = (data[YX] * data[ZY] - data[YY] * data[ZX]) * invDet;
    m[ZY] = (data[XY] * data[ZX] - data[XX] * data[ZY]) * invDet;
    m[ZZ] = (data[XX] * data[YY] - data[XY] * data[YX]) * invDet;
    return m;
  }

  /** Inverse -- don't use this if you have already precomputed
   *  the determinant. */
  mat3_t inverse() const {
    real det = determinant();
    mat3_t rval(false);
    return inverse(rval, det);
  }

  /** Skew-symmetric cross product matrix associated with the given
   *  vector. */
  static mat3_t cross(const vec3_t<real>& v) {
    mat3_t rval(false);
    rval(0,0) =  0;     rval(0,1) = -v[2];  rval(0,2) =  v[1];
    rval(1,0) =  v[2];  rval(1,1) =  0;     rval(1,2) = -v[0];
    rval(2,0) = -v[1];  rval(2,1) =  v[0];  rval(2,2) =  0; 
    return rval;
  }

  /** Outer product matrix. */
  static mat3_t outer(const vec3_t<real>& a, 
                      const vec3_t<real>& b) {
    mat3_t rval(false);
    for (int row=0; row<3; ++row) {
      for (int col=0; col<3; ++col) {
        rval(row,col) = a[row]*b[col];
      }
    }
    return rval;
  }

  /** Underlying data. */
  real data[9];

};

/** Return the product a matrix and a scalar. */
template <class real, class real2> inline mat3_t<real> operator*(const mat3_t<real>& A, real2 s) {
  mat3_t<real> rval(false);
  for (int i=0; i<9; ++i) { rval.data[i] = A.data[i]*s; }
  return rval;
}

/** Return the product a matrix and a scalar. */
template <class real, class real2> inline mat3_t<real> operator*(real2 s, const mat3_t<real>& A) {
  return A*s;
}

/** Compare matrices for equality. */
template <class real> inline bool operator==(const mat3_t<real>& A, const mat3_t<real>& B) {
  return memcmp(A.data, B.data, sizeof(A.data))==0;
}

/** Return the product of two matrices. */
template <class real> inline mat3_t<real> operator*(const mat3_t<real>& A, const mat3_t<real>& B) {
  mat3_t<real> rval(false);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      real accum = 0.0;
      // out(i,j) = ith row of A dotted with jth col of B
      for (int k=0; k<3; ++k) {
	accum += A(i, k) * B(k, j);
      }
      rval(i, j) = accum;
    }
  }
  return rval;
}

/** @relates mat3_t Return the product of a matrix and a 3D vector. */
template <class real> inline vec3_t<real> operator*(const mat3_t<real>& m, const vec3_t<real>& v) {
  
  vec3_t<real> rval(v[0] * m(0,0) + v[1] * m(0,1) + v[2] * m(0,2),
                    v[0] * m(1,0) + v[1] * m(1,1) + v[2] * m(1,2),
                    v[0] * m(2,0) + v[1] * m(2,1) + v[2] * m(2,2));

  return rval;

}

/** @relates mat3_t Matrix stream output. */
template <class real> inline std::ostream& operator<<(std::ostream& ostr, const mat3_t<real>& m) {
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      ostr << std::setw(10) << m(i,j) << " ";
    }
    ostr << "\n";
  }
  return ostr;
}

/** @relates mat3_t Matrix negation. */
template <class real> inline mat3_t<real> operator-(const mat3_t<real>& m) {
  mat3_t<real> rval(false);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      rval(i,j) = -m(i,j);
    }
  }
  return rval;
}

/** @relates mat3_t Matrix addition */
template <class real> inline mat3_t<real> operator+(const mat3_t<real>& m1,
                                                    const mat3_t<real>& m2) {
  mat3_t<real> rval(false);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      rval(i,j) = m1(i,j) + m2(i,j);
    }
  }
  return rval;
}


/** @relates mat3_t Matrix subtraction */
template <class real> inline mat3_t<real> operator-(const mat3_t<real>& m1,
                                                    const mat3_t<real>& m2) {
  mat3_t<real> rval(false);
  for (int i=0; i<3; ++i) {
    for (int j=0; j<3; ++j) {
      rval(i,j) = m1(i,j) - m2(i,j);
    }
  }
  return rval;
}


/** @relates mat3_t Matrix of doubles. */
typedef mat3_t<double> mat3d;

/** @relates mat3_t Matrix of floats. */
typedef mat3_t<float>  mat3f;

#endif
