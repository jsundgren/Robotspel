/**
   Copyright 2004-2016 Karljohan Lundin
   
   This file is a part of KJs Algebra 3D Package.
   
   KJs Algebra 3D Package is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
   
   KJs Algebra 3D Package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with KJs Algebra 3D Package; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/


#ifndef _A3D_LINEARALGEBRA_HH_
#define _A3D_LINEARALGEBRA_HH_

/**
   \mainpage
   
   \section Introduction
   
   I started developing this package because I needed a linear algebra
   package for the IRIX platform and found that the available packages
   does not focus on what I want --- usability through intuitive use
   of operators.
   
   This is an extendable template-based package for 3D linear algebra.
   As many operators as possible are overloaded for easy use and the
   code is kept simple and portable for easy deployment on any
   platform. For easy debugging and tests the print operators (<<) are
   also overloaded.  This package is NOT optimized for speed.
   
   \section Practical Notation
   
   The extensive implementation of operators and casting makes this
   package very effective for use with OpenGL.  There are a number of
   especially nice notations that I will show here.
   
   For example the matrix classes have data stored in column-major
   order for effective interfacing with OpenGL.  Thus, fetching the
   projection matrix is a one row command:
   
   \verbatim
   a3d::Matrix4f Mm;
   glGetFloatv( GL_MODELVIEW_MATRIX, Mm );
   \endverbatim
   
   This can also be done with vectors, for example colour.
   
   \section Features
   
   Primarily this package is developed for simple notation.  Vector
   and matrix operators are all implemented as operators.  Also the
   cross product is implemented, here using the modulus operator (%)
   for nice looking code.
   
   A side effect of the template structure of this package is that any
   type can be used as base for the algebra types.  Algebra types
   using floating point data in single and double precision are
   typedefed, but fixed point data or complex types can also be used.
   
   Another functionality that is nice in 3D environments is the \p
   fromScreenPosition, that calculates the 3D position of a screen
   position and also the vector that points inwards at that point.
   
   \section Download
   
   The A3D linear algebra package is only one single header file with
   all functions inline. This is required for correct template
   handling, however it also means that all you have to do is download
   a header file and included it to your C++ files.
   
   The header file can be downloaded from http://webstaff.itn.liu.se/~karlu20/div/A3D/bin
   
   \section Licensing
   
   The latest version of this package is provided under GNU GPL. That
   means that any derived work, work that use the package or modified
   versions of this package must also be released as GNU GPL if it is
   released at all. You may keep both the source and binaries for
   yourself if you like. Contact me if you want to buy a license that
   does not force you to release your source code.
*/

#include <cmath>
#include <typeinfo>
#include <iostream>
#include <limits>
#include <algorithm>
#include <string.h>

/*! \def A3D_WARNINGS
  \brief If defined functions are allowed to print warnings when called
*/
#ifdef A3D_WARNINGS
# define A3D_WARNING_EXPENSIVE(S) fprintf( stderr, "(W) Expensive call: %s\n", S );
# define A3D_WARNING_OPERATOR(S) fprintf( stderr, "(W) %s\n", S );
#else
# define A3D_WARNING_EXPENSIVE(S) /*S*/
# define A3D_WARNING_OPERATOR(S) /*S*/
#endif


/**
   KJs Algebra 3D Package for C++.  The goal is to have all possible
   operators overloaded inline so that intuitive and effective
   3D algebra can be performed.
*/
namespace a3d {
  
  template< class T > struct Math;
  template< class T, class M = Math<T> > struct Vector3;
  template< class T, class M = Math<T> > struct Vector4;
  template< class T, class M = Math<T> > struct Rotation;
  template< class T, class M = Math<T> > struct Quaternion;
  template< class T, class M = Math<T> > struct Matrix3;
  template< class T, class M = Math<T> > struct Matrix4;
  
  
  /**
     Template-based math functions providing the essential operators
     for a specific type.  Implemented for float and double.  Create
     your own set for your own type, for example fixed point type or
     complex type.
  */
  template< class T >
  struct Math {
    static inline T sqrt(const T &v); /**< Calculate square root. */
    static inline T sin(const T &v);  /**< Calculate sinus. */
    static inline T cos(const T &v);  /**< Calculate cosinus. */
    static inline T acos(const T &v); /**< Calculate arcus cosinus. */
    static inline T epsilon();        /**< Return the smalles significant value of the type. */
  };
  
  
  /**
     A vector of three values for 3d calculations.
  */
  template< class T, class M >
  struct Vector3 {
    
    /** Creates an uninitialized vector. */
    inline Vector3(){}
    
    /** Creates a vector with the specified values. */
    inline Vector3(T x, T y, T z){
      v[0] = x; v[1] = y; v[2] = z;
    }
    
    /** Copies the specified vector. */
    template< class T2, class M2 >
    inline Vector3(const Vector3<T2,M2> &x){
      v[0] = T(x.v[0]); v[1] = T(x.v[1]); v[2] = T(x.v[2]);
    }
    
    /** Copies a float array vector. */
    inline Vector3(const float *va){
      v[0] = va[0]; v[1] = va[1]; v[2] = va[2];
    }
    
    /** Creates a vector initialized to zero. */
    inline explicit Vector3(int i) {
      v[0] = T(0); v[1] = T(0); v[2] = T(0);
    }
    
    /** Cast from homogenious coordinates. */
    template< class T2, class M2 >
    inline explicit Vector3(const Vector4<T2,M2> &x){
      v[0] = T(x.v[0]); v[1] = T(x.v[1]); v[2] = T(x.v[2]);
    }
    
    /** Copy operator. */
    inline Vector3<T,M>& operator= (const Vector3<T,M> &o){
      v[0] = o.v[0]; v[1] = o.v[1]; v[2] = o.v[2];
      return *this;
    }
    
    /** Copy operator. */
    template< class T2, class M2 >
    inline Vector3<T,M>& operator= (const Vector3<T2,M2> &o){
      v[0] = T(o.v[0]); v[1] = T(o.v[1]); v[2] = T(o.v[2]);
      return *this;
    }
    
    /** Copies a float array vector. */
    inline Vector3<T,M>& operator= (const float *va){
      v[0] = va[0]; v[1] = va[1]; v[2] = va[2];
      return *this;
    }
    
    /** Negates the vector. */
    inline Vector3<T,M> operator- () const {
      return Vector3<T,M>( -v[0], -v[1], -v[2] );
    }
    
    /** Returns a normalized version of this vector. */
    inline Vector3<T,M> normalized() const {
      return ( T(1) / length() ) * *this;
    }
    
    /**
       If this is called in an OpenGL context it will find the
       3D equivalent of the specified screen position and the
       vector that points straight in at that position.
       
       Two values are given representing the 2D screen coordinate
       and two vectors in which the result will be inserted.
       The function will return true on success.
       
       \param sx screen x coordinate in the range zero to screen width
       minus one
       
       \param sy screen y coordinate in the range zero to screen
       height minus one.

       This coordinate is usually defined upside down compared to the
       3D system, but if it is not just negate the parameter and this
       function treats it as not upside down.
       
       \param x the specified screen coordinate in 3D space
       
       \param v a unit vector pointing from eye to x position
       
       \return true on successful estimation
    */
    template< class S >
    inline static bool fromScreenPosition( S sx, S sy,
                                           Vector3<T,M> &x,
                                           Vector3<T,M> &v );
    
    /**
       This function treats this vector as a HSV space color vector
       and returns it transformed to RGB color space.
    */
    inline Vector3<T,M> hsv2rgb() const;
    
    /**
       This function treats this vector as a RGB space color vector
       and returns it transformed to HSV color space.
    */
    inline Vector3<T,M> rgb2hsv() const;
    
    /** Returns the length of this vector */
    inline T length() const { return M::sqrt( *this * *this ); }
    
    /** Returns the square length of this vector */
    inline T length2() const { return *this * *this; }
    
    /**
       Casts this vector to an array representation.
       
       This makes it possible to provide for example a float vector
       as an array argument in for example OpenGL.
       
       Use like this:
       \verbatim
       Vector3f c = Vector3f::hsv2rgb( 0.5, 0.5, 0.5 );
       glColor3fv( (float*)c );
       \endverbatim
    */
    inline operator const T* () const { return v; }

    /**
       Casts this vector to an array representation.
       
       This makes it possible to provide for example a float vector
       as an array argument in for example OpenGL.  Anything written
       in the returned array will end up in the vector.
       
       Use like this:
       \verbatim
       Vector3f c = Vector3f::hsv2rgb( 0.5, 0.5, 0.5 );
       glColor3fv( (float*)c );
       \endverbatim
    */
    inline operator T* (){ return v; }
    
    /** Index operator.  Access element. */
    inline const T& operator()(int i) const { return v[i]; }
    
    /** Index operator.  Access element. */
    inline const T& operator[](int i) const { return v[i]; }

    /** Index operator.  Access element. */
    inline T& operator()(int i){ return v[i]; }
    
    /** Index operator.  Access element. */
    inline T& operator[](int i){ return v[i]; }

    T v[3];
  };
  
  /** Floating point vector, single precision. */
  typedef Vector3<float> Vector3f;
  
  /** Floating point vector, double precision. */
  typedef Vector3<double> Vector3d;
  
  
  /**
     A vector of four values for 3d calculations in
     homogeneous coordinates.
  */
  template< class T, class M >
  struct Vector4 {

    /** Creates an uninitialized vector. */
    inline Vector4(){}

    /** Creates a vector initialized to [ 0 0 0 1 ]. */
    inline explicit Vector4(int i) {
      v[0] = T(0); v[1] = T(0); v[2] = T(0); v[3] = T(1);
    }

    /** Creates a vector with the specified values. */
    inline Vector4(T x, T y, T z, T s){
      v[0] = x; v[1] = y; v[2] = z; v[3] = s;
    }

    /** Copies a float array vector. */
    inline Vector4(const float *va){
      v[0] = va[0]; v[1] = va[1]; v[2] = va[2]; v[3] = va[3];
    }
    
    /** Copies the specified vector. */
    template< class T2, class M2 >
    inline Vector4(const Vector4<T2,M2> &x){
      v[0] = T(x.v[0]); v[1] = T(x.v[1]); v[2] = T(x.v[2]); v[3] = T(x.v[3]);
    }
    
    /** Casts from non-homogeneous coordinates. */
    template< class T2, class M2 >
    inline explicit Vector4(const Vector3<T2,M2> &x){
      v[0] = T(x.v[0]); v[1] = T(x.v[1]); v[2] = T(x.v[2]); v[3] = T(1);
    }
    
    /** Copy operator. */
    inline Vector4<T,M>& operator= (const Vector4<T,M> &o){
      v[0] = o.v[0]; v[1] = o.v[1]; v[2] = o.v[2]; v[3] = o.v[3];
      return *this;
    }

    /** Copy operator. */
    template< class T2, class M2 >
    inline Vector4<T,M>& operator= (const Vector4<T2,M2> &o){
      v[0] = T(o.v[0]); v[1] = T(o.v[1]); v[2] = T(o.v[2]); v[3] = T(o.v[3]);
      return *this;
    }

    /** Copies a float array vector. */
    inline Vector4<T,M>& operator= (const float *va){
      v[0] = va[0]; v[1] = va[1]; v[2] = va[2]; v[3] = va[3];
      return *this;
    }
    
    /** Negates the vector. */
    inline Vector4<T,M> operator- () const {
      return Vector4<T>( -v[0], -v[1], -v[2], -v[3] );
    }
    
    /**
       This function treats this vector as a HSVA space color vector
       and returns it transformed to RGBA color space.
    */
    inline Vector4<T,M> hsv2rgb() const;
    
    /**
       This function treats this vector as a RGBA space color vector
       and returns it transformed to HSVA color space.
    */
    inline Vector4<T,M> rgb2hsv() const;
    
    /** Returns the length of this vector */
    inline T length() const { return M::sqrt( *this * *this ); }
    
    /** Returns the square length of this vector */
    inline T length2() const { return *this * *this; }
    
    /**
       Casts this vector to an array representation.
       
       This makes it possible to provide for example a float vector
       as an array argument in for example OpenGL.
       
       Use like this:
       \verbatim
       Vector4f c = Vector4f::hsv2rgb( 0.5, 0.5, 0.5, 0.7 );
       glColor4fv( (float*)c );
       \endverbatim
    */
    inline operator const T* () const { return v; }
    
    /**
       Casts this vector to an array representation.
       
       This makes it possible to provide for example a float vector
       as an array argument in for example OpenGL.  Anything written
       in the returned array will end up in the vector.
       
       Use like this:
       \verbatim
       Vector4f c = Vector4f::hsv2rgb( 0.5, 0.5, 0.5, 0.7 );
       glColor4fv( (float*)c );
       \endverbatim
    */
    inline operator T* (){ return v; }
    
    /** Index operator.  Access element. */
    inline const T& operator()(int i) const { return v[i]; }
    
    /** Index operator.  Access element. */
    inline const T& operator[](int i) const { return v[i]; }
    
    /** Index operator.  Access element. */
    inline T& operator()(int i){ return v[i]; }
    
    /** Index operator.  Access element. */
    inline T& operator[](int i){ return v[i]; }
    
    T v[4];
  };
  
  /** Floating point vector in homogenious coordinates, single
      precision. */
  typedef Vector4<float> Vector4f;
  
  /** Floating point vector in homogenious coordinates, double
      precision. */
  typedef Vector4<double> Vector4d;
  
  
  /**
     A rotation struct.  Use this to set glRotate.  The angle is still
     in radians, though, so remember to convert it to degrees when
     applying to glRotate.
  */
  template< class T, class M >
  struct Rotation {
    
    /** Creates an uninitialized rotation. */
    inline Rotation(){}
    
    /** Creates a rotation initialized to zero rotation. */
    inline explicit Rotation(int i) : a(0), v(1,0,0) {}
    
    /** Creates a rotation from an axis and angle. */
    template< class S, class T2, class M2 >
    inline Rotation(S a, const Vector3<T2,M2> &v) : a( T(a) ), v(v) {}
    
    /** Creates a rotation from an axis and angle. */
    template< class S, class T2, class M2 >
    inline Rotation(const Vector3<T2,M2> &v, S a) : a( T(a) ), v(v) {}
    
    /** Copies a rotation. */
    template< class T2, class M2 >
    inline Rotation(const Rotation<T2,M2> &r) : a( T(r.a) ), v(r.v) {}
    
    /** Creates a rotation from a quaternion (cast). */
    template< class T2, class M2 >
    inline explicit Rotation(const Quaternion<T2,M2> &q)
      : a( T(2) * M::acos( T(q.s) ) ) {
      T2 l = q.v.length();
      if( l > M::epsilon() ){
        v = ( T2(1) / l ) * q.v;
      }
      else
        v = Vector3<T,M>(1,0,0);
    }
    
    /** Copy operator. */
    inline Rotation<T,M>& operator= (const Rotation<T,M> &r){
      a = r.a; v = r.v;
      return *this;
    }
    
    /** Copy operator. */
    template< class T2, class M2 >
    inline Rotation<T,M>& operator= (const Rotation<T2,M2> &r){
      a = T(r.a); v = r.v;
      return *this;
    }
    
    /**
       Returns the angle of this rotation, in degrees.
    */
    T getAngleInDegrees() const {
      return (T)( (180/M_PI)*a );
    }
    
    /**
       Create a vector representation of the rotation. The vector
       length represents the angle, in radians, and the vector
       orientation represents the axis.
     */
    Vector3<T,M> toRotationVector() const {
      return v*a;
    }
    
    /**
       Create the rotation from a vector representation of the
       rotation. The vector length represents the angle, in radians,
       and the vector orientation represents the axis.
     */
    template< class T2, class M2 >
    static Rotation<T,M> fromRotationVector(const Vector3<T2,M2> &v){
      T2 length = v.length();
      if( length < M2::epsilon() ){
        return Rotation<T,M>(0); }
      return Rotation<T,M>( Vector3<T,M>( (T2(1)/length)*v ), T(length) );
    }
    
    T a; /**< Rotation angle in radians. */
    Vector3<T,M> v; /**< Axis of rotation. */
  };
  
  /** Floating point rotation. */
  typedef Rotation<float> Rotationf;
  
  /** Floating point rotation, double precision. */
  typedef Rotation<double> Rotationd;
  
  
  /**
     A quaternion struct primarily for rotation representations and
     calculations.  This implementation, however, can also be used for
     real four dimensional complex calculations.  All operators are
     correctly implemented, so take care.  The inverse and negation
     does not always give the result people think they should.
  */
  template< class T, class M >
  struct Quaternion {
    
    /** Creates an uninitialized quaternion. */
    inline Quaternion() {}
    
    /** Creates a quaternion initialized to zero rotation.
        Observe that this is not the same as a zero quaternion. */
    inline explicit Quaternion(int i)
      : s( T(1) ), v(0,0,0) {}
    
    /** Creates a quaternion from the specified real and imaginary parts. */
    template< class S >
    inline Quaternion( S s, S vi, S vj, S vk )
      : s( T(s) ), v(Vector3<T,M>(vi,vj,vk)) {}
    
    /** Creates a quaternion from the specified real and imaginary parts. */
    template< class S, class T2, class M2 >
    inline Quaternion( S s, const Vector3<T2,M2> &v )
      : s( T(s) ), v(v) {}
    
    /** Creates a quaternion from the specified real and imaginary parts. */
    template< class S, class T2, class M2 >
    inline Quaternion( const Vector3<T2,M2> &v, S s )
      : s( T(s) ), v(v) {}
    
    /** Copies a quaternion. */
    template< class T2, class M2 >
    inline Quaternion(const Quaternion<T2,M2> &q){
      s = T(q.s);
      v = q.v;
    }
    
    /** Cast from rotation. */
    template< class T2, class M2 >
    inline explicit Quaternion(const Rotation<T2,M2> &r){
      T t = M::sin( T(0.5) * T(r.a) );
      s = M::cos( T(0.5) * T(r.a) );
      v = Vector3<T,M>( t * T(r.v[0]),  t * T(r.v[1]),  t * T(r.v[2]) );
    }

    /**
       Casting from vector to quaternion.  The vector becomes the
       complex part of the quaternion
    */
    template< class T2, class M2 >
    inline explicit Quaternion(const Vector3<T2,M2> &v) : s(0), v(v) {}
    
    /** Casting from matrix. */
    template< class T2, class M2 >
    inline explicit Quaternion(const Matrix3<T2,M2> &m) {
      
      T2 t = m(0,0) + m(1,1) + m(2,2) + 1;
      if( t > M2::epsilon() ){
        T2 S = 1 / ( 2 * M2::sqrt(t) );
        s = 1 / ( T2(4) * S );
        v[0] = ( m(2,1) - m(1,2) ) * S;
        v[1] = ( m(0,2) - m(2,0) ) * S;
        v[2] = ( m(1,0) - m(0,1) ) * S;
      }
      
      else if( m(0,0) + m(1,1) + m(2,2) > 0 ){ 
        T2 S = M2::sqrt( m(0,0) + m(1,1) + m(2,2) +T2(1) ) * 2; // S=4*qw 
        s = (1/T2(4)) * S;
        v[0] = (m(2,1) - m(1,2)) / S;
        v[1] = (m(0,2) - m(2,0)) / S; 
        v[2] = (m(1,0) - m(0,1)) / S; 
      }
      else if( (m(0,0) > m(1,1)) && (m(0,0) > m(2,2)) ){
        T2 S = M2::sqrt( T2(1) + m(0,0) - m(1,1) - m(2,2) ) * 2; // S=4*qx 
        s = (m(2,1) - m(1,2)) / S;
        v[0] = (1/T2(4)) * S;
        v[1] = (m(0,1) + m(1,0)) / S; 
        v[2] = (m(0,2) + m(2,0)) / S; 
      }
      else if( m(1,1) > m(2,2) ){ 
        T2 S = M2::sqrt( T2(1) + m(1,1) - m(0,0) - m(2,2) ) * 2; // S=4*qy
        s = (m(0,2) - m(2,0)) / S;
        v[0] = (m(0,1) + m(1,0)) / S; 
        v[1] = (1/T2(4)) * S;
        v[2] = (m(1,2) + m(2,1)) / S; 
      }
      else { 
        T2 S = M2::sqrt( T2(1) + m(2,2) - m(0,0) - m(1,1) ) * 2; // S=4*qz
        s = (m(1,0) - m(0,1)) / S;
        v[0] = (m(0,2) + m(2,0)) / S;
        v[1] = (m(1,2) + m(2,1)) / S;
        v[2] = (1/T2(4)) * S;
      }
    }
    
    /** Copy operator. */
    inline Quaternion<T,M>& operator= (const Quaternion<T,M> &o){
      s = o.s;
      v = o.v;
      return *this;
    }
    
    /** Copy operator. */
    template< class T2, class M2 >
    inline Quaternion<T,M>& operator= (const Quaternion<T2,M2> &o){
      s = T(o.s);
      v = o.v;
      return *this;
    }
    
    /** Negating quaternion. Both real and imaginary parts are
        negated. */
    inline Quaternion<T,M> operator- () const {
      return Quaternion<T,M>( -s, -v );
    }
    
    /**
       Spherical linear interpolation between this and the provided
       quaternion.
       
       The interpolation is specified through the t parameter in
       the interval [0,1].
    */
    inline Quaternion<T,M> slerp( const Quaternion<T,M> &q, double t ){
      double Th = M::acos( s * q.s + v * q.v );
      if( Th > M::epsilon() || Th < -M::epsilon() ){
        return ( T(1)/M::sin(Th) )*( M::sin( (1-t)*Th ) * *this +
                                     M::sin( ( t )*Th ) *   q );
      }
      else {
        if( t < 0.5 ){ return *this; }
        else { return q; }
      }
    }
    
    /**
       Spherical linear interpolation from zero rotation to this.
       
       Use this function to scale the rotation defined by this
       quaternion.  Using multiplication with a scalar value will not
       scale the rotation but the complex number.
    */
    inline Quaternion<T,M> slerp(double t){
      return Quaternion<T,M>(0).slerp( *this, t );
    }
    
    /**
       Creates a quaternion from an axis and an angle of rotation
       around this axis.
       
       The axis of rotation must be of unit length.
    */
    template< class S, class T2, class M2 >
    inline static Quaternion<T,M> fromAxisAngle(const Vector3<T2,M2> &v, S a ){
      return Quaternion<T>( M::cos( T(0.5) * T(a) ), M2::sin( T2(0.5) * T2(a) ) * v );
    }
    
    /**
       Create the quaternion that would rotate a specified unit vector
       to the other specified unit vector.
       
       The result is unspecified if either vector is not of unit length.
       
       \param x the initial unit vector
       \param y the rotated unit vector
    */
    template< class T2, class M2, class T3, class M3 >
    inline static Quaternion<T,M> fromVectorPair(const Vector3<T2,M2> &x,
                                                 const Vector3<T3,M3> &y){
      Vector3<T,M> xy = x % y;
      T xyl = xy.length();
      if( xyl < M::epsilon() ){
        if( x*y > 0 ){
          return Quaternion<T,M>( 1, Vector3<T,M>(0,0,0) ); }
        else {
          return Quaternion<T,M>( 0, Vector3<T,M>(0,1,0) ); }
      }
      else{
        T xys = T( x * y );
        return Quaternion<T,M>( ( M::sqrt( T(0.5) * (1+xys) )     ),
                                ( M::sqrt( T(0.5) * (1-xys) )/xyl ) * xy );
      }
    }
    
    /**
       Create the quaternion that would rotate a specified angle of
       unit vectors into the other specified angle of unit vectors.
       
       The result is unspecified if either vector is not of unit
       length or if the angle between the vector pairs differs.
       
       \param x0 the first unit vector in the initial angle
       \param x1 the second unit vector in the initial angle
       \param y0 the first unit vector in the rotated angle
       \param y1 the second unit vector in the rotated angle
    */
    template< class T2, class M2, class T3, class M3, class T4, class M4, class T5, class M5 >
    inline static Quaternion<T,M> fromAnglePair(const Vector3<T2,M2> &x0,
                                                const Vector3<T3,M3> &y0,
                                                const Vector3<T4,M4> &x1,
                                                const Vector3<T5,M5> &y1){
      Quaternion<T,M> R0 = Quaternion<T,M>::fromVectorPair(x0,x1);
      Vector3<T,M> v0 = ( y0 - x0*(x0*y0) ).normalized();
      Vector3<T,M> v1 = ( y1 - x1*(x1*y1) ).normalized();
      Vector3<T,M> v10 = R0.conjugate().rotate(v1);
      Quaternion<T,M> R1 = Quaternion<T,M>::fromVectorPair(v0,v10);
      return R0*R1;
    }
                                                
    /**
       Create a quaternion from Euler angles.
    */
    template< class T2, class M2 >
    inline static Quaternion<T,M> fromEulerAngles(Vector3<T2,M2> &v){
      
      T cosX = M::cos( T(0.5) * T(v[0]) );
      T cosY = M::cos( T(0.5) * T(v[1]) );
      T cosZ = M::cos( T(0.5) * T(v[2]) );
      
      T sinX = M::sin( T(0.5) * T(v[0]) );
      T sinY = M::sin( T(0.5) * T(v[1]) );
      T sinZ = M::sin( T(0.5) * T(v[2]) );
      
      return Quaternion<T,M>( cosX*cosY*cosZ + sinX*sinY*sinZ,
                              Vector3<T,M>( sinX*cosY*cosZ - cosX*sinY*sinZ,
                                            cosX*sinY*cosZ + sinX*cosY*sinZ,
                                            cosX*cosY*sinZ - sinX*sinY*cosZ ) );
    }
    
    /**
       Rotate a vector using this quaternion.  For this to be correct
       the quaternion must be unit, which is usually true if only
       rotational operations have previously been performed.
    */
    template< class T2, class M2 >
    inline Vector3<T,M> rotate(const Vector3<T2,M2> &v) const {
      return ( ( *this * Quaternion<T,M>(v) ) * conjugate() ).v;
    }
    
    /**
       Rotate a vector using this quaternion.  For this to be correct
       the quaternion must be unit, which is usually true if only
       rotational operations have previously been performed.
    */
    template< class T2, class M2 >
    inline Vector4<T,M> rotate(const Vector4<T2,M2> &v) const {
      return Vector4<T>( ( ( *this * Quaternion<T,M>(v) ) * conjugate() ).v );
    }
    
    /** Returns the quaternion conjugate. */
    inline Quaternion<T,M> conjugate() const {
      return Quaternion<T,M>( s, -v );
    }
    
    /** Returns the quaternion inverse.  If the quaternion is unit the
        result is equal to the conjugate, but this function is
        slightly slower than using the conjugate directly. */
    inline Quaternion<T,M> inverse() const {
      A3D_WARNING_EXPENSIVE("Quaternion.inverse() --- conjugate can often be used");
      return conjugate() / ( *this * conjugate() ).s;
    }
    
    /** Returns the norm of this quaternion. */
    inline T norm() const {
      return M::sqrt( (*this * conjugate()).s );
    }
    
    /** Return a unit version of this quaternion. */
    inline Quaternion<T,M> unit() const {
      T scale = T(1)/norm();
      return Quaternion<T,M>( scale * v, scale * s );
    }
    
    T s; /**< The real part of the quaternion. */
    Vector3<T,M> v; /**< The imaginary part of the quaternion. */
  };
  
  /** Floating point quaternion. */
  typedef Quaternion<float> Quaternionf;
  
  /** Floating point quaternion, double precision. */
  typedef Quaternion<double> Quaterniond;
  
  
  /**
     A matrix of nine values for 3d calculations.
     
     Observe that the data is stored in column-major mode for
     easy interfacing with OpenGL.  The interfaces are specified
     in row-major mode (for example the constructor and print
     operator).
  */
  template< class T, class M >
  struct Matrix3 {
    
    /** Creates an uninitialized matrix. */
    inline Matrix3(){}
    
    /** Creates a unit matrix. */
    inline explicit Matrix3(int i){
      loadIdentity();
    }
    
    /** Copy constructor. */
    template< class T2, class M2 >
    inline Matrix3(const Matrix3<T2,M2> &x){
      m[0] = T(x.m[0]); m[1] = T(x.m[1]); m[2] = T(x.m[2]);
      m[3] = T(x.m[3]); m[4] = T(x.m[4]); m[5] = T(x.m[5]);
      m[6] = T(x.m[6]); m[7] = T(x.m[7]); m[8] = T(x.m[8]);
    }
    
    /** Creates matrix from elements specified in row-major order. */
    template< class S >
    inline Matrix3( S m00, S m01, S m02,
                    S m10, S m11, S m12,
                    S m20, S m21, S m22 ){
      m[0] = T(m00); m[1] = T(m10); m[2] = T(m20);
      m[3] = T(m01); m[4] = T(m11); m[5] = T(m21);
      m[6] = T(m02); m[7] = T(m12); m[8] = T(m22);
    }

    /** Creates matrix from three column vectors. */
    template< class T2, class M2 >
    inline Matrix3(const Vector3<T2,M2> &v0,
                   const Vector3<T2,M2> &v1,
                   const Vector3<T2,M2> &v2) {
      m[0] = T(v0(0)); m[1] = T(v0(1)); m[2] = T(v0(2));
      m[3] = T(v1(0)); m[4] = T(v1(1)); m[5] = T(v1(2));
      m[6] = T(v2(0)); m[7] = T(v2(1)); m[8] = T(v2(2));
    }
    
    /** Casting from Matrix4. */
    inline explicit Matrix3(const Matrix4<T,M> &x){
      m[0] = x.m[0]; m[1] = x.m[1]; m[2] = x.m[2];
      m[3] = x.m[4]; m[4] = x.m[5]; m[5] = x.m[6];
      m[6] = x.m[8]; m[7] = x.m[9]; m[8] = x.m[10];
    }
    
    /** Casting from Matrix4. */
    template< class T2, class M2 >
    inline explicit Matrix3(const Matrix4<T2,M2> &x){
      m[0] = T(x.m[0]); m[1] = T(x.m[1]); m[2] = T(x.m[2]);
      m[3] = T(x.m[4]); m[4] = T(x.m[5]); m[5] = T(x.m[6]);
      m[6] = T(x.m[8]); m[7] = T(x.m[9]); m[8] = T(x.m[10]);
    }
    
    /** Casting from Quaternion. */
    template< class T2, class M2 >
    inline explicit Matrix3(const Quaternion<T2,M2> &q){
      m[0] = T( 1 - 2*q.v[1]*q.v[1] - 2*q.v[2]*q.v[2] );
      m[1] = T(     2*q.v[0]*q.v[1] + 2*q.v[2]*q.s    );
      m[2] = T(     2*q.v[0]*q.v[2] - 2*q.v[1]*q.s    );
      
      m[3] = T(     2*q.v[0]*q.v[1] - 2*q.v[2]*q.s    );
      m[4] = T( 1 - 2*q.v[0]*q.v[0] - 2*q.v[2]*q.v[2] );
      m[5] = T(     2*q.v[1]*q.v[2] + 2*q.v[0]*q.s    );
      
      m[6] = T(     2*q.v[0]*q.v[2] + 2*q.v[1]*q.s    );
      m[7] = T(     2*q.v[1]*q.v[2] - 2*q.v[0]*q.s    );
      m[8] = T( 1 - 2*q.v[0]*q.v[0] - 2*q.v[1]*q.v[1] );
    }
    
    /** Make this matrix identity, i.e. ones in diagonal. */
    inline void loadIdentity(){
      m[0] = T(1); m[1] = T(0); m[2] = T(0);
      m[3] = T(0); m[4] = T(1); m[5] = T(0);
      m[6] = T(0); m[7] = T(0); m[8] = T(1);
    }

    /**
       Creates a matrix representing a rotation around the
       specified axis.
       
       The vector represented by (x,y,z) MUST be of unit length
       or the result will be wrong!
    */
    template< class S, class T2, class M2 >
    inline static Matrix3<T,M> fromAxisAngle(const Vector3<T2,M2> &v, S a){
      T2 c = M2::cos( T2(a) );
      T2 s = M2::sin( T2(a) );
      T2 t = 1.0 - c;
      Matrix3<T,M> m;
      
      m[0] = T( c + v.v[0]*v.v[0]*t );
      m[4] = T( c + v.v[1]*v.v[1]*t );
      m[8] = T( c + v.v[2]*v.v[2]*t );
      
      T2 tmp1 = v.v[0]*v.v[1]*t;
      T2 tmp2 = v.v[2]*s;
      m[1] = T( tmp1 + tmp2 );
      m[3] = T( tmp1 - tmp2 );
      
      tmp1 = v.v[0]*v.v[2]*t;
      tmp2 = v.v[1]*s;
      m[2] = T( tmp1 - tmp2 );
      m[6] = T( tmp1 + tmp2 );
      
      tmp1 = v.v[1]*v.v[2]*t;
      tmp2 = v.v[0]*s;
      m[5] = T( tmp1 + tmp2 );
      m[7] = T( tmp1 - tmp2 );
      return m;
    }
    
    /** Creates a scaling matrix. */
    template< class T2, class M2 >
    inline static Matrix3<T,M> fromScale(const Vector3<T2,M2> &s){
      return Matrix3<T,M>( T(s[0]), T(  0 ), T(  0 ),
                           T(  0 ), T(s[1]), T(  0 ),
                           T(  0 ), T(  0 ), T(s[2]) );
    }
    
    /** Creates a transposed version of the matrix. */
    inline Matrix3<T,M> transpose() const {
      return Matrix3<T,M>( m[0], m[1], m[2],
                           m[3], m[4], m[5],
                           m[6], m[7], m[8] );
    }
    
    /** Calculates the determinant. */
    inline T determinant() const {
      A3D_WARNING_EXPENSIVE("Matrix3.determinant()");
      return
        +m[0]*(m[4]*m[8]-m[5]*m[7])
        -m[1]*(m[3]*m[8]-m[5]*m[6])
        +m[2]*(m[3]*m[7]-m[4]*m[6]);
    }
    
    /** Calculates the comatrix. */
    inline Matrix3<T,M> comatrix() const {
      A3D_WARNING_EXPENSIVE("Matrix3.comatrix()");
      return Matrix3<T,M>( +(m[4]*m[8]-m[5]*m[7]), -(m[3]*m[8]-m[5]*m[6]), +(m[3]*m[7]-m[4]*m[6]),
                           -(m[1]*m[8]-m[2]*m[7]), +(m[0]*m[8]-m[2]*m[6]), -(m[0]*m[7]-m[1]*m[6]),
                           +(m[1]*m[5]-m[2]*m[4]), -(m[0]*m[5]-m[2]*m[3]), +(m[0]*m[4]-m[1]*m[3]) );
    }
    
    /**
       Returns the inverse of the matrix.
       
       This function is not optimized and is therefore extremely
       inefficient.
    */
    inline Matrix3<T,M> inverse() const {
      A3D_WARNING_EXPENSIVE("Matrix3.inverse()");
      return comatrix()/determinant();
    }
    
    /** Copy operator. */
    inline Matrix3<T,M>& operator= (const Matrix3<T,M> &o){
      m[0] = o.m[0]; m[1] = o.m[1]; m[2] = o.m[2];
      m[3] = o.m[3]; m[4] = o.m[4]; m[5] = o.m[5];
      m[6] = o.m[6]; m[7] = o.m[7]; m[8] = o.m[8];
      return *this;
    }
    
    /** Copy operator. */
    template< class T2, class M2 >
    inline Matrix3<T,M>& operator= (const Matrix3<T2,M2> &o){
      m[0] = T(o.m[0]); m[1] = T(o.m[1]); m[2] = T(o.m[2]);
      m[3] = T(o.m[3]); m[4] = T(o.m[4]); m[5] = T(o.m[5]);
      m[6] = T(o.m[6]); m[7] = T(o.m[7]); m[8] = T(o.m[8]);
      return *this;
    }

    /**
       Casts this matrix to an array representation.
       
       This makes it possible to provide for example a float matrix
       as an array argument in for example OpenGL.
       
       Use like this:
       \verbatim
       Matrix3f m;
       glGetParamv( GL_MODELVIEW_MATRIX, (float*)m );
       \endverbatim
    */
    inline operator const T* () const { return m; }

    /**
       Casts this matrix to an array representation.
       
       This makes it possible to provide for example a float matrix
       as an array argument in for example OpenGL.  Anything written
       in the returned array will end up in the matrix.
    */
    inline operator T* (){ return m; }
    
    /** Index access (row,col) */
    inline const T& operator()(int i, int j) const { return m[i+3*j]; }
    
    /** Index operator.  Access element. */
    inline const T& operator[](int i) const { return m[i]; }
    
    /** Index access (row,col) */
    inline T& operator()(int i, int j){ return m[i+3*j]; }
    
    /** Index operator.  Access element. */
    inline T& operator[](int i){ return m[i]; }
    
    T m[9]; /**< Matrix cell data. */
  };
  
  /** Floating point matrix. */
  typedef Matrix3<float> Matrix3f;
  
  /** Floating point matrix, double precision. */
  typedef Matrix3<double> Matrix3d;
  
  
  /**
     A matrix of 16 values for 3d calculations in
     homogeneous coordinates.
     
     Observe that the data is stored in column-major mode for
     easy interfacing with OpenGL.  The interfaces are specified
     in row-major mode (for example the constructor and print
     operator).
  */
  template< class T, class M >
  struct Matrix4 {

    /** Creates an uninitialized matrix. */
    inline Matrix4(){}
    
    /** Creates a unit matrix. */
    inline explicit Matrix4(int i){
      loadIdentity();
    }
    
    /** Copy constructor. */
    inline Matrix4(const Matrix4<T> &x){
      memcpy(m,x.m,16*sizeof(T));
    }
    
    /** Copy constructor. */
    template< class T2 >
    inline Matrix4(const Matrix4<T2> &x){
      m[ 0] = T(x.m[ 0]); m[ 1] = T(x.m[ 1]); m[ 2] = T(x.m[ 2]); m[ 3] = T(x.m[ 3]);
      m[ 4] = T(x.m[ 4]); m[ 5] = T(x.m[ 5]); m[ 6] = T(x.m[ 6]); m[ 7] = T(x.m[ 7]);
      m[ 8] = T(x.m[ 8]); m[ 9] = T(x.m[ 9]); m[10] = T(x.m[10]); m[11] = T(x.m[11]);
      m[12] = T(x.m[12]); m[13] = T(x.m[13]); m[14] = T(x.m[14]); m[15] = T(x.m[15]);
    }
    
    /** Creates matrix from three or four column vectors. */
    template< class T2, class M2 >
    inline Matrix4(const Vector4<T2,M2> &v0,
                   const Vector4<T2,M2> &v1,
                   const Vector4<T2,M2> &v2,
                   const Vector4<T2,M2> &v3 = Vector4<T2,M2>(0,0,0,1)) {
      m[ 0] = T(v0(0)); m[ 1] = T(v0(1)); m[ 2] = T(v0(2)); m[ 3] = T(v0(3));
      m[ 4] = T(v1(0)); m[ 5] = T(v1(1)); m[ 6] = T(v1(2)); m[ 7] = T(v1(3));
      m[ 8] = T(v2(0)); m[ 9] = T(v2(1)); m[10] = T(v2(2)); m[11] = T(v2(3));
      m[12] = T(v3(0)); m[13] = T(v3(1)); m[14] = T(v3(2)); m[15] = T(v3(3));
    }
    
    /** Creates matrix from three or four column vectors. */
    template< class T2, class M2 >
    inline Matrix4(const Vector3<T2,M2> &v0,
                   const Vector3<T2,M2> &v1,
                   const Vector3<T2,M2> &v2,
                   const Vector3<T2,M2> &v3 = Vector3<T2,M2>(0,0,0)) {
      m[ 0] = T(v0(0)); m[ 1] = T(v0(1)); m[ 2] = T(v0(2)); m[ 3] = T(0);
      m[ 4] = T(v1(0)); m[ 5] = T(v1(1)); m[ 6] = T(v1(2)); m[ 7] = T(0);
      m[ 8] = T(v2(0)); m[ 9] = T(v2(1)); m[10] = T(v2(2)); m[11] = T(0);
      m[12] = T(v3(0)); m[13] = T(v3(1)); m[14] = T(v3(2)); m[15] = T(1);
    }
    
    /** Creates matrix from elements specified in row-major order. */
    template< class S >
    inline Matrix4( S m00, S m01, S m02, S m03,
                    S m10, S m11, S m12, S m13,
                    S m20, S m21, S m22, S m23,
                    S m30, S m31, S m32, S m33 ){
      m[ 0] = T(m00); m[ 1] = T(m10); m[ 2] = T(m20); m[ 3] = T(m30);
      m[ 4] = T(m01); m[ 5] = T(m11); m[ 6] = T(m21); m[ 7] = T(m31);
      m[ 8] = T(m02); m[ 9] = T(m12); m[10] = T(m22); m[11] = T(m32);
      m[12] = T(m03); m[13] = T(m13); m[14] = T(m23); m[15] = T(m33);
    }
    
    /** Casting from Matrix3. */
    template< class T2, class M2 >
    inline explicit Matrix4(const Matrix3<T2,M2> &x){
      m[ 0] = T(x.m[0]); m[ 1] = T(x.m[1]); m[ 2] = T(x.m[2]); m[ 3] = T(0);
      m[ 4] = T(x.m[3]); m[ 5] = T(x.m[4]); m[ 6] = T(x.m[5]); m[ 7] = T(0);
      m[ 8] = T(x.m[6]); m[ 9] = T(x.m[7]); m[10] = T(x.m[8]); m[11] = T(0);
      m[12] = T(    0 ); m[13] = T(    0 ); m[14] = T(    0 ); m[15] = T(1);
    }
    
    /** Casting from Quaternion */
    template< class T2, class M2 >
    inline explicit Matrix4(const Quaternion<T2,M2> &q){
      m[ 0] = T( 1 - 2*q.v[1]*q.v[1] - 2*q.v[2]*q.v[2] );
      m[ 1] = T(     2*q.v[0]*q.v[1] + 2*q.v[2]*q.s    );
      m[ 2] = T(     2*q.v[0]*q.v[2] - 2*q.v[1]*q.s    );
      m[ 3] = T(0);
      
      m[ 4] = T(     2*q.v[0]*q.v[1] - 2*q.v[2]*q.s    );
      m[ 5] = T( 1 - 2*q.v[0]*q.v[0] - 2*q.v[2]*q.v[2] );
      m[ 6] = T(     2*q.v[1]*q.v[2] + 2*q.v[0]*q.s    );
      m[ 7] = T(0);
      
      m[ 8] = T(     2*q.v[0]*q.v[2] + 2*q.v[1]*q.s    );
      m[ 9] = T(     2*q.v[1]*q.v[2] - 2*q.v[0]*q.s    );
      m[10] = T( 1 - 2*q.v[0]*q.v[0] - 2*q.v[1]*q.v[1] );
      m[11] = T(0);
      
      m[12] = T(0);
      m[13] = T(0);
      m[14] = T(0);
      m[15] = T(1);
    }
    
    /** Make this matrix identity, i.e. ones in diagonal. */
    inline void loadIdentity(){
      m[ 0] = T(1); m[ 1] = T(0); m[ 2] = T(0); m[ 3] = T(0);
      m[ 4] = T(0); m[ 5] = T(1); m[ 6] = T(0); m[ 7] = T(0);
      m[ 8] = T(0); m[ 9] = T(0); m[10] = T(1); m[11] = T(0);
      m[12] = T(0); m[13] = T(0); m[14] = T(0); m[15] = T(1);
    }
    
    /** Creates a translating matrix. */
    template< class T2, class M2 >
    inline static Matrix4<T,M> fromTranslation(const Vector3<T2,M2> &v){
      return Matrix4<T>( T(1), T(0), T(0), T(v[0]),
                         T(0), T(1), T(0), T(v[1]),
                         T(0), T(0), T(1), T(v[2]),
                         T(0), T(0), T(0), T(  1 ) );
    }
    
    /**
       Create a matrix from a array representation of an array
       in row-major mode.  [ (00), (01), (02), (03), (10), ... ]
    */
    template< class S >
    inline static Matrix4<T,M> fromArrayRM(const S * ma){
      return Matrix4<T,M>( (T)ma[ 0], (T)ma[ 1], (T)ma[ 2], (T)ma[ 3],
                           (T)ma[ 4], (T)ma[ 5], (T)ma[ 6], (T)ma[ 7],
                           (T)ma[ 8], (T)ma[ 9], (T)ma[10], (T)ma[11],
                           (T)ma[12], (T)ma[13], (T)ma[14], (T)ma[15] );
    }

    /**
       Create a matrix from a array representation of an array
       in column-major mode.  [ (00), (10), (20), (30), (01), ... ]
    */
    template< class S >
    inline static Matrix4<T,M> fromArrayCM(const S * ma){
      return Matrix4<T,M>( (T)ma[ 0], (T)ma[ 4], (T)ma[ 8], (T)ma[12],
                           (T)ma[ 1], (T)ma[ 5], (T)ma[ 9], (T)ma[13],
                           (T)ma[ 2], (T)ma[ 6], (T)ma[10], (T)ma[14],
                           (T)ma[ 3], (T)ma[ 7], (T)ma[11], (T)ma[15] );
    }
    
    inline Matrix3<T,M> getScaleRotation() const {
      return Matrix3<T,M>( m[ 0], m[ 4], m[ 8],
                           m[ 1], m[ 5], m[ 9],
                           m[ 2], m[ 6], m[10] );
    }
    
    inline Vector3<T,M> getTranslation() const {
      return Vector3<T,M>( m[12], m[13], m[14] );
    }
    
    /**
       Separates the transform represented by this matrix into
       scale, translation and rotation.
    */
    inline bool separate( Vector3<T,M> &scale,
                          Matrix3<T,M> &rotation,
                          Vector3<T,M> &translation ) const;
    
    /**
       Struct performing interpolation between two matrices
    */
    struct Interpolator {
      /** Create an interpolation between the specified matrices. */
      inline Interpolator( const Matrix4<T,M>& m1, const Matrix4<T,M>& m2 ){
        Matrix3<T> m1_r, m2_r;
        m1.separate( m1_s, m1_r, m1_t );
        m2.separate( m2_s, m2_r, m2_t );
        m1_q = Quaternion<T,M>(m1_r);
        m2_q = Quaternion<T,M>(m2_r);
      }
      /** Get the interpolated matrix */
      template< class S >
      inline Matrix4<T,M> operator()(S t){
        T t2 = T(t);
        Matrix4<T,M> m =
          Matrix4<T,M>(m1_q.slerp(m2_q,t)) *
          Matrix4<T,M>( (1-t2)*m1_s(0)+t2*m2_s(0), T(0), T(0), T(0),
                        T(0), (1-t2)*m1_s(1)+t2*m2_s(1), T(0), T(0),
                        T(0), T(0), (1-t2)*m1_s(2)+t2*m2_s(2), T(0),
                        T(0), T(0), T(0), T(1) );
        m(0,3) = (1-t)*m1_t(0) + t*m2_t(0);
        m(1,3) = (1-t)*m1_t(1) + t*m2_t(1);
        m(2,3) = (1-t)*m1_t(2) + t*m2_t(2);
        return m;
      }
    private:
      Vector3<T,M> m1_s, m2_s, m1_t, m2_t;
      Quaternion<T,M> m1_q, m2_q;
    };
    
    /** Creates a transposed version of the matrix. */
    inline Matrix4<T,M> transpose() const {
      return Matrix4<T,M>( m[ 0], m[ 1], m[ 2], m[ 3],
                           m[ 4], m[ 5], m[ 6], m[ 7],
                           m[ 8], m[ 9], m[10], m[11],
                           m[12], m[13], m[14], m[15] );
    }
    
    /** Calculates the determinant. */
    inline T determinant() const {
      A3D_WARNING_EXPENSIVE("Matrix4.determinant()");
      return
        +m[ 0] * ( +m[ 5]*m[10]*m[15] +m[ 6]*m[11]*m[13] +m[ 7]*m[ 9]*m[14]
                   -m[ 7]*m[10]*m[13] -m[ 6]*m[ 9]*m[15] -m[ 5]*m[11]*m[14] )
        -m[ 1] * ( +m[ 4]*m[10]*m[15] +m[ 6]*m[11]*m[12] +m[ 7]*m[ 8]*m[14]
                   -m[ 7]*m[10]*m[12] -m[ 6]*m[ 8]*m[15] -m[ 4]*m[11]*m[14] )
        +m[ 2] * ( +m[ 4]*m[ 9]*m[15] +m[ 5]*m[11]*m[12] +m[ 7]*m[ 8]*m[13]
                   -m[ 7]*m[ 9]*m[12] -m[ 5]*m[ 8]*m[15] -m[ 4]*m[11]*m[13] )
        -m[ 3] * ( +m[ 4]*m[ 9]*m[14] +m[ 5]*m[10]*m[12] +m[ 6]*m[ 8]*m[13]
                   -m[ 6]*m[ 9]*m[12] -m[ 5]*m[ 8]*m[14] -m[ 4]*m[10]*m[13] );
    }
    
    /** Calculates the comatrix. */
    inline Matrix4<T,M> comatrix() const {
      A3D_WARNING_EXPENSIVE("Matrix4.comatrix()");
      return Matrix4<T,M>( +( +m[ 5]*m[10]*m[15] +m[ 6]*m[11]*m[13] +m[ 7]*m[ 9]*m[14]
                              -m[ 7]*m[10]*m[13] -m[ 6]*m[ 9]*m[15] -m[ 5]*m[11]*m[14] ),
                           -( +m[ 4]*m[10]*m[15] +m[ 6]*m[11]*m[12] +m[ 7]*m[ 8]*m[14]
                              -m[ 7]*m[10]*m[12] -m[ 6]*m[ 8]*m[15] -m[ 4]*m[11]*m[14] ),
                           +( +m[ 4]*m[ 9]*m[15] +m[ 5]*m[11]*m[12] +m[ 7]*m[ 8]*m[13]
                              -m[ 7]*m[ 9]*m[12] -m[ 5]*m[ 8]*m[15] -m[ 4]*m[11]*m[13] ),
                           -( +m[ 4]*m[ 9]*m[14] +m[ 5]*m[10]*m[12] +m[ 6]*m[ 8]*m[13]
                              -m[ 6]*m[ 9]*m[12] -m[ 5]*m[ 8]*m[14] -m[ 4]*m[10]*m[13] ),
                           
                           -( +m[ 1]*m[10]*m[15] +m[ 2]*m[11]*m[13] +m[ 3]*m[ 9]*m[14]
                              -m[ 3]*m[10]*m[13] -m[ 2]*m[ 9]*m[15] -m[ 1]*m[11]*m[14] ),
                           +( +m[ 0]*m[10]*m[15] +m[ 2]*m[11]*m[12] +m[ 3]*m[ 8]*m[14]
                              -m[ 3]*m[10]*m[12] -m[ 2]*m[ 8]*m[15] -m[ 0]*m[11]*m[14] ),
                           -( +m[ 0]*m[ 9]*m[15] +m[ 1]*m[11]*m[12] +m[ 3]*m[ 8]*m[13]
                              -m[ 3]*m[ 9]*m[12] -m[ 1]*m[ 8]*m[15] -m[ 0]*m[11]*m[13] ),
                           +( +m[ 0]*m[ 9]*m[14] +m[ 1]*m[10]*m[12] +m[ 2]*m[ 8]*m[13]
                              -m[ 2]*m[ 9]*m[12] -m[ 1]*m[ 8]*m[14] -m[ 0]*m[10]*m[13] ),
                           
                           +( +m[ 1]*m[ 6]*m[15] +m[ 2]*m[ 7]*m[13] +m[ 3]*m[ 5]*m[14]
                              -m[ 3]*m[ 6]*m[13] -m[ 2]*m[ 5]*m[15] -m[ 1]*m[ 7]*m[14] ),
                           -( +m[ 0]*m[ 6]*m[15] +m[ 2]*m[ 7]*m[12] +m[ 3]*m[ 4]*m[14]
                              -m[ 3]*m[ 6]*m[12] -m[ 2]*m[ 4]*m[15] -m[ 0]*m[ 7]*m[14] ),
                           +( +m[ 0]*m[ 5]*m[15] +m[ 1]*m[ 7]*m[12] +m[ 3]*m[ 4]*m[13]
                              -m[ 3]*m[ 5]*m[12] -m[ 1]*m[ 4]*m[15] -m[ 0]*m[ 7]*m[13] ),
                           -( +m[ 0]*m[ 5]*m[14] +m[ 1]*m[ 6]*m[12] +m[ 2]*m[ 4]*m[13]
                              -m[ 2]*m[ 5]*m[12] -m[ 1]*m[ 4]*m[14] -m[ 0]*m[ 6]*m[13] ),
                           
                           -( +m[ 1]*m[ 6]*m[11] +m[ 2]*m[ 7]*m[ 9] +m[ 3]*m[ 5]*m[10]
                              -m[ 3]*m[ 6]*m[ 9] -m[ 2]*m[ 5]*m[11] -m[ 1]*m[ 7]*m[10] ),
                           +( +m[ 0]*m[ 6]*m[11] +m[ 2]*m[ 7]*m[ 8] +m[ 3]*m[ 4]*m[10]
                              -m[ 3]*m[ 6]*m[ 8] -m[ 2]*m[ 4]*m[11] -m[ 0]*m[ 7]*m[10] ),
                           -( +m[ 0]*m[ 5]*m[11] +m[ 1]*m[ 7]*m[ 8] +m[ 3]*m[ 4]*m[ 9]
                              -m[ 3]*m[ 5]*m[ 8] -m[ 1]*m[ 4]*m[11] -m[ 0]*m[ 7]*m[ 9] ),
                           +( +m[ 0]*m[ 5]*m[10] +m[ 1]*m[ 6]*m[ 8] +m[ 2]*m[ 4]*m[ 9]
                              -m[ 2]*m[ 5]*m[ 8] -m[ 1]*m[ 4]*m[10] -m[ 0]*m[ 6]*m[ 9] ) );
    }
    
    /**
       Returns the inverse of the matrix.
       
       This function is not optimized and is therefore extremely
       inefficient.
    */
    inline Matrix4<T,M> inverse() const {
      A3D_WARNING_EXPENSIVE("Matrix4.inverse()");
      return comatrix()/determinant();
    }
    
    /** Returns the inverse of the affine matrix. This function will
        produce an unknown result if applied to a matrix that does not
        represent an affine transform. */
    inline Matrix4<T,M> affineInverse() const {
      Matrix3<T,M> Ri = this->getScaleRotation().inverse();
      Vector3<T,M> Ti = -Ri * this->getTranslation();
      return Matrix4<T,M>::fromTranslation(Ti) * Matrix4<T,M>(Ri);
    }
    
    /** Copy operator. */
    inline Matrix4<T,M>& operator= (const Matrix4<T,M> &o){
      m[ 0] = o.m[ 0]; m[ 1] = o.m[ 1]; m[ 2] = o.m[ 2]; m[ 3] = o.m[ 3];
      m[ 4] = o.m[ 4]; m[ 5] = o.m[ 5]; m[ 6] = o.m[ 6]; m[ 7] = o.m[ 7];
      m[ 8] = o.m[ 8]; m[ 9] = o.m[ 9]; m[10] = o.m[10]; m[11] = o.m[11];
      m[12] = o.m[12]; m[13] = o.m[13]; m[14] = o.m[14]; m[15] = o.m[15];
      return *this;
    }
    
    /** Copy operator. */
    template< class T2, class M2 >
    inline Matrix4<T,M>& operator= (const Matrix4<T2,M2> &o){
      m[ 0] = T(o.m[ 0]); m[ 1] = T(o.m[ 1]); m[ 2] = T(o.m[ 2]); m[ 3] = T(o.m[ 3]);
      m[ 4] = T(o.m[ 4]); m[ 5] = T(o.m[ 5]); m[ 6] = T(o.m[ 6]); m[ 7] = T(o.m[ 7]);
      m[ 8] = T(o.m[ 8]); m[ 9] = T(o.m[ 9]); m[10] = T(o.m[10]); m[11] = T(o.m[11]);
      m[12] = T(o.m[12]); m[13] = T(o.m[13]); m[14] = T(o.m[14]); m[15] = T(o.m[15]);
      return *this;
    }

    /**
       Casts this matrix to an array representation.
       
       This makes it possible to provide for example a float matrix
       as an array argument in for example OpenGL.
    */
    inline operator const T* () const { return m; }

    /**
       Casts this matrix to an array representation.
       
       This makes it possible to provide for example a float matrix
       as an array argument in for example OpenGL.  Anything written
       in the returned array will end up in the matrix.
    */
    inline operator T* (){ return m; }

    /** Index access (row,col) */
    inline const T& operator()(int i, int j) const { return m[i+4*j]; }
    
    /** Index operator.  Access element. */
    inline const T& operator[](int i) const { return m[i]; }

    /** Index access (row,col) */
    inline T& operator()(int i, int j){ return m[i+4*j]; }
    
    /** Index operator.  Access element. */
    inline T& operator[](int i){ return m[i]; }
    
    T m[16]; /**< Matrix cell data. */
  };
  
  /** Floating point matrix. */
  typedef Matrix4<float> Matrix4f;
  
  /** Floating point matrix, double precision. */
  typedef Matrix4<double> Matrix4d;
  
  
  // General operators
  
  /** Operator */
  template< class S1, class S2 >
  inline S1& operator+= ( S1 &o1, const S2 &o2 ){
    return ( o1 = o1 + o2 );
  }
  
  /** Operator */
  template< class S1, class S2 >
  inline S1& operator-= ( S1 &o1, const S2 &o2 ){
    return ( o1 = o1 - o2 );
  }
  
  /**
     This operator is defined as o1 = o2 * o1.

     This is because in those cases where the multiplication is
     non-commutative this order is the most common.  Examples are
     vector-matrix or quaternion-quaternion multiplication.
  */
  template< class S1, class S2 >
  inline S1& operator*= ( S1 &o1, const S2 &o2 ){
    return ( o1 = o2 * o1 );
  }
  
  /** Operator */
  template< class S1, class S2 >
  inline S1& operator/= ( S1 &o1, const S2 &o2 ){
    return ( o1 = o1 / o2 );
  }
  
  // Scalar product
  
  /** Operator */
  template< class T, class M >
  inline T operator* (const Vector3<T,M> &o1,const Vector3<T,M> &o2){
    return o1[0]*o2[0] + o1[1]*o2[1] + o1[2]*o2[2];
  }

  /** Operator */
  template< class T, class M >
  inline T operator* (const Vector4<T,M> &o1,const Vector4<T,M> &o2){
    return o1[0]*o2[0] + o1[1]*o2[1] + o1[2]*o2[2] + o1[3]*o2[3];
  }

  // Cross product

  /**
     The cross product operator.
     
     Since the modulus operator looks most like the cross and is not
     defined for two vector operands this operator is used as the
     cross product operator.
  */
  template< class T, class M >
  inline Vector3<T,M> operator% (const Vector3<T,M> &o1,const Vector3<T,M> &o2){
    return Vector3<T,M>( o1[1]*o2[2]-o1[2]*o2[1],
                         o1[2]*o2[0]-o1[0]*o2[2],
                         o1[0]*o2[1]-o1[1]*o2[0] );
  }

  // Vector addition

  /** Operator */
  template< class T, class M >
  inline Vector3<T,M> operator+ (const Vector3<T,M> &o1, const Vector3<T,M> &o2){
    return Vector3<T,M>( o1[0]+o2[0], o1[1]+o2[1], o1[2]+o2[2] );
  }

  /** Operator */
  template< class T, class M >
  inline Vector3<T,M> operator- (const Vector3<T,M> &o1, const Vector3<T,M> &o2){
    return Vector3<T,M>( o1[0]-o2[0], o1[1]-o2[1], o1[2]-o2[2] );
  }

  /** Operator */
  template< class T, class M >
  inline Vector4<T,M> operator+ (const Vector4<T,M> &o1, const Vector4<T,M> &o2){
    return Vector4<T,M>( o1[0]+o2[0], o1[1]+o2[1], o1[2]+o2[2], o1[3]+o2[3] );
  }

  /** Operator */
  template< class T, class M >
  inline Vector4<T,M> operator- (const Vector4<T,M> &o1, const Vector4<T,M> &o2){
    return Vector4<T,M>( o1[0]-o2[0], o1[1]-o2[1], o1[2]-o2[2], o1[3]-o2[3] );
  }

  // Vector scaling

  /** Vector scaling operator */
  template< class S, class T, class M >
  inline Vector3<T,M> operator* (S o1, const Vector3<T,M> &o2){
    return Vector3<T,M>( ((T)o1)*o2[0], ((T)o1)*o2[1], ((T)o1)*o2[2] );
  }

  /** Vector scaling operator */
  template< class S, class T, class M >
  inline Vector3<T,M> operator* (const Vector3<T,M> &o1, S o2){
    return o2*o1;
  }

  /** Vector scaling operator */
  template< class S, class T, class M >
  inline Vector3<T,M> operator/ (const Vector3<T,M> &o1, S o2){
    return ( ((T)1)/(T)o2 )*o1;
  }

  /** Vector scaling operator */
  template< class S, class T, class M >
  inline Vector4<T,M> operator* (S o1, const Vector4<T,M> &o2){
    return Vector4<T,M>( ((T)o1)*o2[0], ((T)o1)*o2[1], ((T)o1)*o2[2], ((T)o1)*o2[3] );
  }

  /** Vector scaling operator */
  template< class S, class T, class M >
  inline Vector4<T,M> operator* (const Vector4<T,M> &o1, S o2){
    return o2*o1;
  }

  /** Vector scaling operator */
  template< class S, class T, class M >
  inline Vector4<T,M> operator/ (const Vector4<T,M> &o1, S o2){
    return ( ((T)1)/(T)o2 )*o1;
  }

  // Quaternion operators
  
  /** Warning for this operator:  a multiplication with a scalar
      destroys the unity of the quaternion. */
  template< class S, class T, class M >
  inline Quaternion<T,M> operator* (S o1, const Quaternion<T,M> &o2){
    A3D_WARNING_OPERATOR("scalar * Quaternion destroys unity");
    return Quaternion<T,M>( ((T)o1)*o2.s, ((T)o1)*o2.v );
  }
  
  /** Warning for this operator:  a multiplication with a scalar
      destroys the unity of the quaternion. */
  template< class S, class T, class M >
  inline Quaternion<T,M> operator* (const Quaternion<T,M> &o1, S o2){
    return o2*o1;
  }
  
  /** Quaternion multiplication operator. */
  template< class T, class M >
  inline Quaternion<T,M> operator* (const Quaternion<T,M> &o1, const Quaternion<T,M> &o2){
    return Quaternion<T,M>( o1.s*o2.s - o1.v*o2.v,
                            o1.s*o2.v + o2.s*o1.v + o1.v%o2.v );
  }
  
  /** Warning for this operator:  a division by a scalar destroys
      the unity of the quaternion. */
  template< class S, class T, class M >
  inline Quaternion<T,M> operator/ (const Quaternion<T,M> &o1, S o2){
    return ( ((T)1) / (T)o2 ) * o1;
  }
  
  /**
     Warning for this operator:  division between two quaternions is not uniquely
     defined so make sure that you know what you are doing.
     
     This operator defines division, q1 / q2, as q1 x 1/q2.  Futhermore, if at least
     the second operand is unit it is cheaper to do multiplication with the conjugate.
  */
  template< class T, class M >
  inline Quaternion<T,M> operator/ (const Quaternion<T,M> &o1, const Quaternion<T,M> &o2){
    A3D_WARNING_OPERATOR("Quaternion division is not uniquely defined");
    return o1 * o2.inverse();
  }
  
  /** Quaternion addition operator. */
  template< class T, class M >
  inline Quaternion<T,M> operator+ (const Quaternion<T,M> &o1, const Quaternion<T,M> &o2){
    return Quaternion<T,M>( o1.s+o2.s, o1.v+o2.v );
  }
  
  /** Quaternion substraction operator. */
  template< class T, class M >
  inline Quaternion<T,M> operator- (const Quaternion<T,M> &o1, const Quaternion<T,M> &o2){
    return Quaternion<T,M>( o1.s-o2.s, o1.v-o2.v );
  }
  
  // Matrix operators
  
  /** Matrix scaling operator. */
  template< class T, class S, class M >
  inline Matrix3<T,M> operator* (const S &o1, const Matrix3<T,M> &o2){
    return Matrix3<T,M>( o1*o2[0], o1*o2[3], o1*o2[6],
                         o1*o2[1], o1*o2[4], o1*o2[7],
                         o1*o2[2], o1*o2[5], o1*o2[8] );
  }
  
  /** Matrix scaling operator. */
  template< class T, class S, class M >
  inline Matrix4<T,M> operator* (const S &o1, const Matrix4<T,M> &o2){
    return Matrix4<T,M>( o1*o2[ 0], o1*o2[ 4], o1*o2[ 8], o1*o2[12],
                         o1*o2[ 1], o1*o2[ 5], o1*o2[ 9], o1*o2[13],
                         o1*o2[ 2], o1*o2[ 6], o1*o2[10], o1*o2[14],
                         o1*o2[ 3], o1*o2[ 7], o1*o2[11], o1*o2[15] );
  }
  
  /** Matrix scaling operator. */
  template< class T, class S, class M >
  inline Matrix3<T,M> operator/ (const Matrix3<T,M> &o1, const S &o2){
    return ( ((T)1)/o2 ) * o1;
  }
  
  /** Matrix scaling operator. */
  template< class T, class S, class M >
  inline Matrix4<T,M> operator/ (const Matrix4<T,M> &o1, const S &o2){
    return ( ((T)1)/o2 ) * o1;
  }
  
  // Matrix Matrix operators
  
  /** Matrix matrix multiplication operator. */
  template< class T, class M >
  inline Matrix3<T,M> operator* (const Matrix3<T,M> &o1, const Matrix3<T,M> &o2){
    return Matrix3<T,M>( o1[0]*o2[0]+o1[3]*o2[1]+o1[6]*o2[2],
                         o1[0]*o2[3]+o1[3]*o2[4]+o1[6]*o2[5],
                         o1[0]*o2[6]+o1[3]*o2[7]+o1[6]*o2[8],
                         o1[1]*o2[0]+o1[4]*o2[1]+o1[7]*o2[2],
                         o1[1]*o2[3]+o1[4]*o2[4]+o1[7]*o2[5],
                         o1[1]*o2[6]+o1[4]*o2[7]+o1[7]*o2[8],
                         o1[2]*o2[0]+o1[5]*o2[1]+o1[8]*o2[2],
                         o1[2]*o2[3]+o1[5]*o2[4]+o1[8]*o2[5],
                         o1[2]*o2[6]+o1[5]*o2[7]+o1[8]*o2[8] );
  }
  
  /** Matrix matrix multiplication operator. */
  template< class T, class M >
  inline Matrix4<T,M> operator* (const Matrix4<T,M> &o1, const Matrix4<T,M> &o2){
    return Matrix4<T,M>( o1[ 0]*o2[ 0]+o1[ 4]*o2[ 1]+o1[ 8]*o2[ 2]+o1[12]*o2[ 3],
                         o1[ 0]*o2[ 4]+o1[ 4]*o2[ 5]+o1[ 8]*o2[ 6]+o1[12]*o2[ 7],
                         o1[ 0]*o2[ 8]+o1[ 4]*o2[ 9]+o1[ 8]*o2[10]+o1[12]*o2[11],
                         o1[ 0]*o2[12]+o1[ 4]*o2[13]+o1[ 8]*o2[14]+o1[12]*o2[15],
                         o1[ 1]*o2[ 0]+o1[ 5]*o2[ 1]+o1[ 9]*o2[ 2]+o1[13]*o2[ 3],
                         o1[ 1]*o2[ 4]+o1[ 5]*o2[ 5]+o1[ 9]*o2[ 6]+o1[13]*o2[ 7],
                         o1[ 1]*o2[ 8]+o1[ 5]*o2[ 9]+o1[ 9]*o2[10]+o1[13]*o2[11],
                         o1[ 1]*o2[12]+o1[ 5]*o2[13]+o1[ 9]*o2[14]+o1[13]*o2[15],
                         o1[ 2]*o2[ 0]+o1[ 6]*o2[ 1]+o1[10]*o2[ 2]+o1[14]*o2[ 3],
                         o1[ 2]*o2[ 4]+o1[ 6]*o2[ 5]+o1[10]*o2[ 6]+o1[14]*o2[ 7],
                         o1[ 2]*o2[ 8]+o1[ 6]*o2[ 9]+o1[10]*o2[10]+o1[14]*o2[11],
                         o1[ 2]*o2[12]+o1[ 6]*o2[13]+o1[10]*o2[14]+o1[14]*o2[15],
                         o1[ 3]*o2[ 0]+o1[ 7]*o2[ 1]+o1[11]*o2[ 2]+o1[15]*o2[ 3],
                         o1[ 3]*o2[ 4]+o1[ 7]*o2[ 5]+o1[11]*o2[ 6]+o1[15]*o2[ 7],
                         o1[ 3]*o2[ 8]+o1[ 7]*o2[ 9]+o1[11]*o2[10]+o1[15]*o2[11],
                         o1[ 3]*o2[12]+o1[ 7]*o2[13]+o1[11]*o2[14]+o1[15]*o2[15] );
  }
  
  // Matrix vector operators

  /** Matrix vector multiplication operator. */
  template< class T, class M >
  inline Vector3<T,M> operator* (const Matrix3<T,M> &o1, const Vector3<T,M> &o2){
    return Vector3<T,M>( o1[0]*o2[0]+o1[3]*o2[1]+o1[6]*o2[2],
                         o1[1]*o2[0]+o1[4]*o2[1]+o1[7]*o2[2],
                         o1[2]*o2[0]+o1[5]*o2[1]+o1[8]*o2[2] );
  }

  /** Vector matrix multiplication operator. */
  template< class T, class M >
  inline Matrix3<T,M> operator* (const Vector3<T,M> &o1, const Matrix3<T,M> &o2){
    return Matrix3<T,M>( o1[0]*o2[0]+o1[0]*o2[1]+o1[0]*o2[2],
                         o1[0]*o2[3]+o1[0]*o2[4]+o1[0]*o2[5],
                         o1[0]*o2[6]+o1[0]*o2[7]+o1[0]*o2[8],
                         o1[1]*o2[0]+o1[1]*o2[1]+o1[1]*o2[2],
                         o1[1]*o2[3]+o1[1]*o2[4]+o1[1]*o2[5],
                         o1[1]*o2[6]+o1[1]*o2[7]+o1[1]*o2[8],
                         o1[2]*o2[0]+o1[2]*o2[1]+o1[2]*o2[2],
                         o1[2]*o2[3]+o1[2]*o2[4]+o1[2]*o2[5],
                         o1[2]*o2[6]+o1[2]*o2[7]+o1[2]*o2[8] );
  }

  /** Matrix vector multiplication operator. */
  template< class T, class M >
  inline Vector4<T,M> operator* (const Matrix4<T,M> &o1, const Vector4<T,M> &o2){
    return Vector4<T,M>( o1[ 0]*o2[0]+o1[ 4]*o2[1]+o1[ 8]*o2[2]+o1[12]*o2[3],
                         o1[ 1]*o2[0]+o1[ 5]*o2[1]+o1[ 9]*o2[2]+o1[13]*o2[3],
                         o1[ 2]*o2[0]+o1[ 6]*o2[1]+o1[10]*o2[2]+o1[14]*o2[3],
                         o1[ 3]*o2[0]+o1[ 7]*o2[1]+o1[11]*o2[2]+o1[15]*o2[3] );
  }

  /** Vector matrix multiplication operator. */
  template< class T, class M >
  inline Matrix4<T,M> operator* (const Vector4<T,M> &o1, const Matrix4<T,M> &o2){
    return Matrix4<T,M>( o1[0]*o2[ 0]+o1[0]*o2[ 1]+o1[0]*o2[ 2]+o1[0]*o2[ 3],
                         o1[0]*o2[ 4]+o1[0]*o2[ 5]+o1[0]*o2[ 6]+o1[0]*o2[ 7],
                         o1[0]*o2[ 8]+o1[0]*o2[ 9]+o1[0]*o2[10]+o1[0]*o2[11],
                         o1[0]*o2[12]+o1[0]*o2[13]+o1[0]*o2[14]+o1[0]*o2[15],
                         o1[1]*o2[ 0]+o1[1]*o2[ 1]+o1[1]*o2[ 2]+o1[1]*o2[ 3],
                         o1[1]*o2[ 4]+o1[1]*o2[ 5]+o1[1]*o2[ 6]+o1[1]*o2[ 7],
                         o1[1]*o2[ 8]+o1[1]*o2[ 9]+o1[1]*o2[10]+o1[1]*o2[11],
                         o1[1]*o2[12]+o1[1]*o2[13]+o1[1]*o2[14]+o1[1]*o2[15],
                         o1[2]*o2[ 0]+o1[2]*o2[ 1]+o1[2]*o2[ 2]+o1[2]*o2[ 3],
                         o1[2]*o2[ 4]+o1[2]*o2[ 5]+o1[2]*o2[ 6]+o1[2]*o2[ 7],
                         o1[2]*o2[ 8]+o1[2]*o2[ 9]+o1[2]*o2[10]+o1[2]*o2[11],
                         o1[2]*o2[12]+o1[2]*o2[13]+o1[2]*o2[14]+o1[2]*o2[15],
                         o1[3]*o2[ 0]+o1[3]*o2[ 1]+o1[3]*o2[ 2]+o1[3]*o2[ 3],
                         o1[3]*o2[ 4]+o1[3]*o2[ 5]+o1[3]*o2[ 6]+o1[3]*o2[ 7],
                         o1[3]*o2[ 8]+o1[3]*o2[ 9]+o1[3]*o2[10]+o1[3]*o2[11],
                         o1[3]*o2[12]+o1[3]*o2[13]+o1[3]*o2[14]+o1[3]*o2[15] );
  }
  
  // Printing operators
  
  /** Printing operator. */
  template< class T, class M >
  inline std::ostream& operator<< (std::ostream &out, const Vector3<T,M> &o){
    out << "Vector3<" << typeid(T).name() << ">"
      "( " << o[0] << ", " << o[1] << ", " << o[2] << " )";
    return out;
  }
  
  /** Printing operator. */
  template< class T, class M >
  inline std::ostream& operator<< (std::ostream &out, const Vector4<T,M> &o){
    out << "Vector4<" << typeid(T).name() << ">"
      "( " << o[0] << ", " << o[1] << ", " << o[2] << ", " << o[3] << " )";
    return out;
  }
  
  /** Printing operator. */
  template< class T, class M >
  inline std::ostream& operator<< (std::ostream &out, const Quaternion<T,M> &o){
    out << "Quaternion<" << typeid(T).name() << ">"
      "( " << o.s << ", " << o.v << " )";
    return out;
  }
  
  /** Printing operator. */
  template< class T, class M >
  inline std::ostream& operator<< (std::ostream &out, const Rotation<T,M> &o){
    out << "Rotation<" << typeid(T).name() << ">"
      "( " << o.v << ", " << o.a << " )";
    return out;
  }
  
  /** Printing operator. */
  template< class T, class M >
  inline std::ostream& operator<< (std::ostream &out, const Matrix3<T,M> &o){
    out
      << "Matrix3<" << typeid(T).name() << ">" << std::endl
      << "( " << o[0] << "  " << o[3] << "  " << o[6] << std::endl
      << "  " << o[1] << "  " << o[4] << "  " << o[7] << std::endl
      << "  " << o[2] << "  " << o[5] << "  " << o[8] << " )";
    return out;
  }
  
  /** Printing operator. */
  template< class T, class M >
  inline std::ostream& operator<< (std::ostream &out, const Matrix4<T,M> &o){
    out
      << "Matrix4<" << typeid(T).name() << ">" << std::endl 
      << "( " << o[ 0] << "  " << o[ 4] << "  " << o[ 8] << "  " << o[12] << std::endl
      << "  " << o[ 1] << "  " << o[ 5] << "  " << o[ 9] << "  " << o[13] << std::endl
      << "  " << o[ 2] << "  " << o[ 6] << "  " << o[10] << "  " << o[14] << std::endl
      << "  " << o[ 3] << "  " << o[ 7] << "  " << o[11] << "  " << o[15] << " )";
    return out;
  }
  
  
  // Math functions
  
  template<>
  inline float Math<float>::sqrt(const float &v){ return ::sqrtf(v); }

  template<>
  inline float Math<float>::sin(const float &v){ return ::sinf(v); }

  template<>
  inline float Math<float>::cos(const float &v){ return ::cosf(v); }

  template<>
  inline float Math<float>::acos(const float &v){ return ::acosf(v); }

  template<>
  inline float Math<float>::epsilon(){
    return std::numeric_limits<float>::epsilon(); }
  

  template<>
  inline double Math<double>::sqrt(const double &v){ return ::sqrt(v); }

  template<>
  inline double Math<double>::sin(const double &v){ return ::sin(v); }

  template<>
  inline double Math<double>::cos(const double &v){ return ::cos(v); }

  template<>
  inline double Math<double>::acos(const double &v){ return ::acos(v); }

  template<>
  inline double Math<double>::epsilon(){
    return std::numeric_limits<double>::epsilon(); }
  
  
  // Other functions
  
  template< class T, class M >
  a3d::Vector3<T,M> a3d::Vector3<T,M>::hsv2rgb() const {
    float h = float(v[0]) * 360.0f;
    float s = this->v[1];
    float v = this->v[2];
  
    // normalize hue to lie in 0<=h<360
    h = (int)h % 360;
    h += (h<0.0)*360.0;
  
    // divide hue into 60 degree sectors
    long i = long(h/60.0);
    float f = h/60.0 - i;
  
    float p = 1.0 - s;
    float q = 1.0 - s*f;
    float t = 1.0 - s*(1.-f);

    // each hue sector will be represented by rgb values taken
    // from one of v, p, q, or t
    float r = (((i==0)|(i==5)) + ((i==2)|(i==3))*p + (i==1)*q + (i==4)*t) * v;
    float g = (((i==1)|(i==2)) + ((i==4)|(i==5))*p + (i==3)*q + (i==0)*t) * v;
    float b = (((i==3)|(i==4)) + ((i==0)|(i==1))*p + (i==5)*q + (i==2)*t) * v;
    
    return a3d::Vector3<T,M>( r, g, b );
  }

  template< class T, class M >
  a3d::Vector4<T,M> a3d::Vector4<T,M>::hsv2rgb() const{
    a3d::Vector3<T,M> rgb = a3d::Vector3<T,M>(*this).hsv2rgb();
    return a3d::Vector4<T,M>( rgb[0], rgb[1], rgb[2], (*this)[3] );
  }

  template< class T, class Mth >
  a3d::Vector3<T,Mth> a3d::Vector3<T,Mth>::rgb2hsv() const {
    
    T R = v[0];
    T G = v[1];
    T B = v[2];
    
    T H, S;
    
    T M = std::max(R,std::max(G,B));
    T m = std::min(R,std::min(G,B));
    T C = M - m;
    
    if( M <= Mth::epsilon() ){ S = 0; }
    else{ S = C/M; }
    
    if( C <= Mth::epsilon() ){ H = 0; }
    else if( M == R ){ H = ( (G-B)/C )+6; }
    else if( M == G ){ H = ( (B-R)/C )+2; }
    else /* M == B */{ H = ( (R-G)/C )+4; }
    if( H > 6 ){ H -= 6; }
    H /= (T)(6);
    
    return a3d::Vector3<T,Mth>( H, S, M );
  }

  template< class T, class M >
  a3d::Vector4<T,M> a3d::Vector4<T,M>::rgb2hsv() const{
    a3d::Vector3<T,M> hsv = a3d::Vector3<T,M>(*this).rgb2hsv();
    return a3d::Vector4<T,M>( hsv[0], hsv[1], hsv[2], (*this)[3] );
  }
  
  template< class T, class M >
  template< class S >
  bool a3d::Vector3<T,M>::fromScreenPosition( S sx, S sy,
                                              a3d::Vector3<T,M> &x,
                                              a3d::Vector3<T,M> &v ){
#ifdef GL_MODELVIEW_MATRIX
    
    a3d::Matrix4f Mm;
    glGetFloatv( GL_MODELVIEW_MATRIX, (float*)Mm );
    a3d::Matrix4f Mmi = Mm.inverse();
    
    GLint size[4];
    glGetIntegerv( GL_VIEWPORT, size );
    int width = size[2]-size[0];
    int height = size[3]-size[1];
    if( width <= 0 || height <= 0 ){
      return false; }
    
    a3d::Matrix4f Mp;
    glGetFloatv( GL_PROJECTION_MATRIX, (float*)Mp );
    if( fabsf( Mp[10] -1.0f ) < std::numeric_limits<float>::epsilon() ||
        fabsf( Mp[10] +1.0f ) < std::numeric_limits<float>::epsilon() ){
      return false; }
    
    float n = Mp[14]/( Mp[10] -1.0f );
    float f = Mp[14]/( Mp[10] +1.0f );
    
    sy = sy < 0 ? sy = -sy : height-sy;
    
    a3d::Matrix4f Mpi = Mp.inverse();
    
    a3d::Vector4f v0_4( 2.0f*sx/width -1.0f,
                        2.0f*sy/height -1.0f,
                        -1.0f, 1.0f );
    v0_4 = n * v0_4;
    v0_4 = Mpi * v0_4;
    v0_4 = Mmi * v0_4;
    
    a3d::Vector4f v1_4( 2.0f * sx/width -1.0f,
                        2.0f * sy/height -1.0f,
                        +1.0f, 1.0f );
    v1_4 = f * v1_4;
    v1_4 = Mpi * v1_4;
    v1_4 = Mmi * v1_4;
    
    x = a3d::Vector3<T,M>( v0_4[0], v0_4[1], v0_4[2] );
    v = a3d::Vector3<T,M>( v1_4[0] - v0_4[0],
                           v1_4[1] - v0_4[1],
                           v1_4[2] - v0_4[2] ).normalized();
    
    return true;
#else
    return false;
#endif // end #ifdef GL_MODELVIEW_MATRIX
  }
  
  template< class T, class M >
  bool a3d::Matrix4<T,M>::separate( a3d::Vector3<T,M> &scale,
                                    a3d::Matrix3<T,M> &rotation,
                                    a3d::Vector3<T,M> &translation ) const{
    translation = a3d::Vector3<T,M>( (*this)(0,3), (*this)(1,3), (*this)(2,3) );
    rotation = Matrix3<T,M>(*this);
    scale = a3d::Vector3<T,M>( ( rotation * Vector3<T,M>(1,0,0) ).length(),
                               ( rotation * Vector3<T,M>(0,1,0) ).length(),
                               ( rotation * Vector3<T,M>(0,0,1) ).length() );
  
    if( scale(0) < M::epsilon() ||
        scale(1) < M::epsilon() ||
        scale(2) < M::epsilon() ){
      return false; }
  
    rotation(0,0) /= scale(0); rotation(0,1) /= scale(0); rotation(0,2) /= scale(0); 
    rotation(1,0) /= scale(1); rotation(1,1) /= scale(1); rotation(1,2) /= scale(1); 
    rotation(2,0) /= scale(2); rotation(2,1) /= scale(2); rotation(2,2) /= scale(2); 
    
    return true;
  }

} // namespace a3d

#endif // end #ifndef _A3D_LINEARALGEBRA_HH_

