/*
 * Copyright (c) 2016, M.Naruoka (fenrir)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the naruoka.org nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __COORDINATE_H__
#define __COORDINATE_H__

/** @file
 * @brief coordinate system definition
 */

#include <cmath>

#ifdef pow2
#define POW2_ALREADY_DEFINED
#else
#define pow2(x) ((x)*(x))
#endif

#ifdef pow3
#define POW3_ALREADY_DEFINED
#else
#define pow3(x) ((x)*(x)*(x))
#endif

#include "param/vector3.h"
#include "param/matrix.h"

#include "WGS84.h"

template <class FloatT = double>
class System_3D {
  protected:
    typedef System_3D self_t;
    FloatT v[3];
    template <class T>
    static const T &const_ref(T *ptr){return static_cast<const T &>(*ptr);}
    template <class T>
    static T &non_const_ref(const T &ref){return const_cast<T &>(ref);}
  public:
    static const unsigned int value_boundary = 3;
    System_3D(){
      for(unsigned i(0); i < value_boundary; i++){
        v[i] = FloatT(0);
      }
    }
    System_3D(const FloatT &v0, const FloatT &v1, const FloatT &v2){
      v[0] = v0;
      v[1] = v1;
      v[2] = v2;
    }
    System_3D(const self_t &orig){
      for(unsigned i(0); i < value_boundary; i++){
        v[i] = orig.v[i];
      }
    }
    System_3D(const Vector3<FloatT> &vec){
      for(unsigned i(0); i < value_boundary; i++){
        v[i] = vec.get(i);
      }
    }
    template <template <class> class Array2D_Type, class ViewType>
    System_3D(const Matrix<FloatT, Array2D_Type, ViewType> &matrix){
      if(matrix.rows() < matrix.columns()){
        for(unsigned i(0); i < value_boundary; i++){
          v[i] = matrix(0, i);
        }
      }else{
        for(unsigned i(0); i < value_boundary; i++){
          v[i] = matrix(i, 0);
        }
      }
    }
    ~System_3D(){}
    System_3D &operator=(const System_3D &another){
      if(this != &another){
        for(unsigned i(0); i < value_boundary; i++){
          v[i] = another.v[i];
        }
      }
      return *this;
    }
    
    const FloatT &operator[](const int &i) const {return v[i];}
    FloatT &operator[](const int &i){
      return non_const_ref(const_ref(this)[i]);
    }

    operator Vector3<FloatT>() const {
      return Vector3<FloatT>(v);
    }

    friend std::ostream &operator<<(std::ostream &out, const self_t &self){
      out << operator[](0) << " "
          << operator[](1) << " "
          << operator[](2);
      return out;
    }
    friend std::istream &operator>>(std::istream &in, self_t &self){
      in >> self[0];
      in >> self[1];
      in >> self[2];
      return in;
    }
};

template <class FloatT, class Earth> class System_XYZ;
template <class FloatT, class Earth> class System_LLH;

template <class FloatT = double, class Earth = WGS84>
class System_XYZ : public System_3D<FloatT> {
  protected:
    typedef System_XYZ<FloatT, Earth> self_t;
    typedef System_3D<FloatT> super_t;
    using super_t::const_ref;
    using super_t::non_const_ref;
  public:
    System_XYZ() : super_t() {}
    template <class T>
    System_XYZ(const T &x, const T &y, const T &z)
        : super_t(x, y, z) {}
    System_XYZ(const self_t &xyz)
        : super_t(xyz) {}
    System_XYZ(const Vector3<FloatT> &vec)
        : super_t(vec) {}
    template <template <class> class Array2D_Type, class ViewType>
    System_XYZ(const Matrix<FloatT, Array2D_Type, ViewType> &matrix)
        : super_t(matrix) {}
    ~System_XYZ(){}
    
    const FloatT &x() const {return super_t::operator[](0);}
    const FloatT &y() const {return super_t::operator[](1);}
    const FloatT &z() const {return super_t::operator[](2);}

    FloatT &x(){return non_const_ref(const_ref(this).x());}
    FloatT &y(){return non_const_ref(const_ref(this).y());}
    FloatT &z(){return non_const_ref(const_ref(this).z());}

    self_t &operator+=(const self_t &another){
      x() += another.x();
      y() += another.y();
      z() += another.z();
      return *this;
    }
    self_t operator+(const self_t &another) const {
      self_t copy(*this);
      return copy += another;
    }
    self_t &operator-=(const self_t &another){
      x() -= another.x();
      y() -= another.y();
      z() -= another.z();
      return *this;
    }
    self_t operator-(const self_t &another) const {
      self_t copy(*this);
      return copy -= another;
    }
    
    FloatT dist() const {
      
      return FloatT(std::sqrt(
          pow2(x()) + pow2(y()) + pow2(z())));
    }
    
    FloatT dist(const self_t &another) const {
      return (*this - another).dist();
    }
    
    static const FloatT f0, a0, b0, e0;

    /**
     * Convert to LatLongAlt
     * 
     */
    System_LLH<FloatT, Earth> llh() const {
      if((x() == 0) && (y() == 0) && (z() == 0)){
        return System_LLH<FloatT, Earth>(0, 0, -a0);
      }

      FloatT h(pow2(a0) - pow2(b0));
      FloatT p(std::sqrt(pow2(x()) + pow2(y())));
      //cout << "p => " << p << endl;
      FloatT t(std::atan2(z()*a0, p*b0));
      FloatT sint(std::sin(t)), cost(std::cos(t));
      
      FloatT _lat(std::atan2(z() + (h / b0 * pow3(sint)), p - (h / a0 * pow3(cost))));
      FloatT n(a0 / std::sqrt(1.0 - pow2(e0) * pow2(std::sin(_lat))));
      
      return System_LLH<FloatT, Earth>(
          _lat,
          std::atan2(y(), x()),
          (p / std::cos(_lat) - n));
    }
};

template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::f0(Earth::F_e);
template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::a0(Earth::R_e);
template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::b0(Earth::R_e * (1.0 - Earth::F_e));
template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::e0(std::sqrt(Earth::F_e * (2.0 - Earth::F_e)));

template <class FloatT = double, class Earth = WGS84>
class System_LLH : public System_3D<FloatT> {
  protected:
    typedef System_LLH<FloatT, Earth> self_t;
    typedef System_XYZ<FloatT, Earth> xyz_t;
    typedef System_3D<FloatT> super_t;
    using super_t::const_ref;
    using super_t::non_const_ref;
  public:
    System_LLH() : super_t() {}
    System_LLH(const FloatT &latitude, const FloatT &longitude, const FloatT &height) 
        : super_t(latitude, longitude, height) {}
    System_LLH(const self_t &llh) 
        : super_t(llh) {}
    ~System_LLH(){}
    
    const FloatT &latitude() const {return super_t::operator[](0);}
    const FloatT &longitude() const {return super_t::operator[](1);}
    const FloatT &height() const {return super_t::operator[](2);}

    FloatT &latitude(){return non_const_ref(const_ref(this).latitude());}
    FloatT &longitude(){return non_const_ref(const_ref(this).longitude());}
    FloatT &height(){return non_const_ref(const_ref(this).height());}

    /**
     * Convert to ECEF (Earth-centered Earth-Fixed)
     *
     */
    xyz_t xyz() const {
      FloatT
          clat(std::cos(latitude())), clng(std::cos(longitude())),
          slat(std::sin(latitude())), slng(std::sin(longitude()));
      FloatT n(xyz_t::a0 / std::sqrt(1.0 - pow2(xyz_t::e0) * pow2(slat)));
      
      return System_XYZ<FloatT, Earth>(
          (n + height()) * clat * slng,
          (n + height()) * clat * slng,
          (n * (1.0 -pow2(xyz_t::e0)) + height()) * slat);
    }

    friend std::ostream &operator<<(std::ostream &out, const self_t &self){
      out << (latitude() / M_PI * 180) << " "
          << (longitude() / M_PI * 180) << " "
          << height();
      return out;
    }
    friend std::istream &operator>>(std::istream &in, self_t &self){
      FloatT _lat, _lng;
      in >> _lat;
      in >> _lng;
      in >> self[2];
      self.latitude() = _lat / 180 * M_PI;
      self.longitude() = _lng / 180 * M_PI;
      return in;
    }
};

template <class FloatT = double, class Earth = WGS84>
class System_ENU : public System_3D<FloatT> {
  protected:
    typedef System_ENU<FloatT, Earth> self_t;
    typedef System_3D<FloatT> super_t;
    typedef System_XYZ<FloatT, Earth> xyz_t;
    typedef System_LLH<FloatT, Earth> llh_t;
    using super_t::const_ref;
    using super_t::non_const_ref;
  public:
    System_ENU() : super_t() {}
    System_ENU(const FloatT &east, const FloatT &north, const FloatT &up) 
        : super_t(east, north, up) {}
    System_ENU(const self_t &enu) 
        : super_t(enu) {}
    System_ENU(const Vector3<FloatT> &vec)
        : super_t(vec) {}
    template <template <class> class Array2D_Type, class ViewType>
    System_ENU(const Matrix<FloatT, Array2D_Type, ViewType> &mat)
        : super_t(mat) {}
    ~System_ENU() {}
    
    const FloatT &east() const {return super_t::operator[](0);}  ///< “Œ¬•ª[m]
    const FloatT &north() const {return super_t::operator[](1);}  ///< –k¬•ª[m]
    const FloatT &up() const {return super_t::operator[](2);}  ///< ã¬•ª[m]
    
    FloatT &east(){return non_const_ref(const_ref(this).east());}
    FloatT &north(){return non_const_ref(const_ref(this).north());}
    FloatT &up(){return non_const_ref(const_ref(this).up());}

    static self_t relative_rel(const xyz_t &rel_pos, const llh_t &base_llh){
      FloatT s1(std::sin(base_llh.longitude())),
          c1(std::cos(base_llh.longitude())),
          s2(std::sin(base_llh.latitude())),
          c2(std::cos(base_llh.latitude()));
      
      return self_t(
          -rel_pos.x() * s1      + rel_pos.y() * c1,
          -rel_pos.x() * c1 * s2 - rel_pos.y() * s1 * s2 + rel_pos.z() * c2,
           rel_pos.x() * c1 * c2 + rel_pos.y() * s1 * c2 + rel_pos.z() * s2);
    }
    
    static self_t relative_rel(const xyz_t &rel_pos, const xyz_t &base){
      return relative_rel(rel_pos, base.llh());
    }
    
    static self_t relative(const xyz_t &pos, const xyz_t &base){
      return relative_rel(pos - base, base);
    }
    
    xyz_t absolute(const xyz_t &base) const {
      llh_t base_llh(base.llh());
      FloatT s1(std::sin(base_llh.longitude())), c1(std::cos(base_llh.longitude()));
      FloatT s2(std::sin(base_llh.latitude())), c2(std::cos(base_llh.latitude()));
      
      return xyz_t(
          -east() * s1 - north() * c1 * s2 + up() * c1 * c2 + base.x(),
           east() * c1 - north() * s1 * s2 + up() * s1 * c2 + base.y(),
                         north() * c2      + up() * s2      + base.z());
    }
    
    FloatT elevation() const {
      return FloatT(std::atan2(up(), std::sqrt(pow2(east()) + pow2(north()))));
    }
    FloatT azimuth() const {
      return FloatT(std::atan2(east(), north()));
    }
};

#ifdef POW2_ALREADY_DEFINED
#undef POW2_ALREADY_DEFINED
#else
#undef pow2
#endif

#ifdef POW3_ALREADY_DEFINED
#undef POW3_ALREADY_DEFINED
#else
#undef pow3
#endif

#endif // __COORDINATE_H__
