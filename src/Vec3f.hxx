#ifndef VEC3F_HXX
#define VEC3F_HXX

#include <math.h>
#include <assert.h>
//#include <iostream>
//#include <fstream>

class Vec3f 
{
public:
  float x,y,z;

  Vec3f() 
  {};
  Vec3f(float x,float y, float z) 
    : x(x),y(y),z(z) 
  {};

  //! Array operator: just for convenience: @f$ a[0]==a.x, a[1]==a.y, a[2]==a.z @f$ 
  /*! be careful : a[0] is often slower than accessing a.x !
   * Also assert that @f$  0 <= i < 3 @f$ 
   */
  inline const float &operator[](const int i) const
  { return *(&x+i); }; 

  //! Array operator: just for convenience: @f$  a[0]==a.x, a[1]==a.y, a[2]==a.z @f$ ...
  /*! be careful : a[0] is often slower than accessing a.x !
   * Also assert that @f$  0 <= i < 3 @f$  
   */
  inline float &operator[](const int i)
  { return *(&x+i); }; //((float *)(this))[i]; };

  inline Vec3f &operator=(const Vec3f &b)
  { x = b.x; y = b.y; z = b.z; return *this;};
};

#define Epsilon 1E-5
#define Infinity HUGE_VAL

  /*! dot product */
inline float Dot(const Vec3f &a, const Vec3f &b)
{ return a.x*b.x+a.y*b.y+a.z*b.z; };

  /*! component-wise product */
inline Vec3f Product(const Vec3f &a, const Vec3f &b)
{ return Vec3f(a.x*b.x,a.y*b.y,a.z*b.z); };

  /*! vector product */
inline Vec3f Cross(const Vec3f &a, const Vec3f &b)
{ return Vec3f(a.y*b.z-a.z*b.y,
	    a.z*b.x-a.x*b.z,
	    a.x*b.y-a.y*b.x); };


inline Vec3f operator-(const Vec3f &v)
{ return Vec3f(-v.x,-v.y,-v.z); };

inline float Length(const Vec3f &v)
{ return sqrt(Dot(v,v)); };

  /*! still undocumented */
inline Vec3f operator*(const float f, const Vec3f &v)
{ return Vec3f(f*v.x, f*v.y, f*v.z); };

  /*! still undocumented */
inline Vec3f operator*(const Vec3f &v, const float f)
{ return Vec3f(f*v.x, f*v.y, f*v.z); };

  /*! still undocumented */
inline void operator*=(Vec3f &v, const float f)
{ v.x *= f; v.y*=f; v.z*=f; };

  /*! still undocumented */
inline void operator*=(Vec3f &v, const Vec3f &f)
{ v.x *= f.x; v.y*=f.y; v.z*=f.z; };


  /*! still undocumented */
inline Vec3f operator/(const Vec3f &v, const float f)
{ return (1/f)*v; };

  /*! still undocumented */
inline void operator/=(Vec3f &v, const float f)
{ v *= (1/f); };

  /*! still undocumented */
inline Vec3f operator+(const Vec3f &a, const Vec3f &b)
{ return Vec3f(a.x+b.x, a.y+b.y, a.z+b.z); };

  /*! still undocumented */
inline Vec3f operator-(const Vec3f &a, const Vec3f &b)
{ return Vec3f(a.x-b.x, a.y-b.y, a.z-b.z); };

inline void Normalize(Vec3f &v)
{ v *= (1.f/Length(v)); };

//inline ostream &operator<<(ostream &o,const Vec3f &v)
//{ o << "(" << v.x << "," << v.y << "," << v.z << ")"; return o; };


#endif


