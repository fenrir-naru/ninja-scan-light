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
    System_3D(const Matrix<FloatT> &matrix){
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
    
    FloatT &operator[](int i){return v[i];}

    operator Vector3<FloatT>() const {
      return Vector3<FloatT>(v[0], v[1], v[2]);
    }

    friend std::ostream &operator<<(std::ostream &out, const self_t &self){
      out << const_cast<self_t *>(&self)->operator[](0) << " "
          << const_cast<self_t *>(&self)->operator[](1) << " "
          << const_cast<self_t *>(&self)->operator[](2);
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
  public:
    System_XYZ() : super_t() {}
    template <class T>
    System_XYZ(const T &x, const T &y, const T &z)
        : super_t(x, y, z) {}
    System_XYZ(const self_t &xyz)
        : super_t(xyz) {}
    System_XYZ(const Vector3<FloatT> &vec)
        : super_t(vec) {}
    System_XYZ(const Matrix<FloatT> &matrix)
        : super_t(matrix) {}
    ~System_XYZ(){}
    
    FloatT &x(){return super_t::operator[](0);}
    FloatT &y(){return super_t::operator[](1);}
    FloatT &z(){return super_t::operator[](2);}
    
    self_t &operator+=(const self_t &another){
      x() += const_cast<self_t *>(&another)->x();
      y() += const_cast<self_t *>(&another)->y();
      z() += const_cast<self_t *>(&another)->z();
      return *this;
    }
    self_t operator+(const self_t &another) const {
      self_t copy(*this);
      return copy += another;
    }
    self_t &operator-=(const self_t &another){
      x() -= const_cast<self_t *>(&another)->x();
      y() -= const_cast<self_t *>(&another)->y();
      z() -= const_cast<self_t *>(&another)->z();
      return *this;
    }
    self_t operator-(const self_t &another) const {
      self_t copy(*this);
      return copy -= another;
    }
    
    FloatT dist() const {
      
      const FloatT &_x(const_cast<self_t *>(this)->x());
      const FloatT &_y(const_cast<self_t *>(this)->y());
      const FloatT &_z(const_cast<self_t *>(this)->z());
      
      return FloatT(std::sqrt(
          pow2(_x) + pow2(_y) + pow2(_z)
        ));
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
      const FloatT &_x(const_cast<self_t *>(this)->x());
      const FloatT &_y(const_cast<self_t *>(this)->y());
      const FloatT &_z(const_cast<self_t *>(this)->z());
      
      if((_x == 0) && (_y == 0) && (_z == 0)){
        return System_LLH<FloatT, Earth>(0, 0, -a0);
      }

      FloatT h(pow2(a0) - pow2(b0));
      FloatT p(std::sqrt(pow2(_x) + pow2(_y)));
      //cout << "p => " << p << endl;
      FloatT t(std::atan2(_z*a0, p*b0));
      FloatT sint(std::sin(t)), cost(std::cos(t));
      
      FloatT _lat(std::atan2(_z + (h / b0 * pow3(sint)), p - (h / a0 * pow3(cost))));
      FloatT n(a0 / std::sqrt(1.0 - pow2(e0) * pow2(std::sin(_lat))));
      
      return System_LLH<FloatT, Earth>(
          _lat,
          std::atan2(_y, _x),
          (p / std::cos(_lat) - n)
        );
    }
};

template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::f0(Earth::F_e);
template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::a0(Earth::R_e);
template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::b0(a0 * (1.0 - f0));
template <class FloatT, class Earth>
const FloatT System_XYZ<FloatT, Earth>::e0(std::sqrt(f0 * (2.0 - f0)));

template <class FloatT = double, class Earth = WGS84>
class System_LLH : public System_3D<FloatT> {
  protected:
    typedef System_LLH<FloatT, Earth> self_t;
    typedef System_3D<FloatT> super_t;
  public:
    System_LLH() : super_t() {}
    System_LLH(const FloatT &latitude, const FloatT &longitude, const FloatT &height) 
        : super_t(latitude, longitude, height) {}
    System_LLH(const self_t &llh) 
        : super_t(llh) {}
    ~System_LLH(){}
    
    FloatT &latitude()  {return super_t::operator[](0);}
    FloatT &longitude() {return super_t::operator[](1);}
    FloatT &height()    {return super_t::operator[](2);}
    
    static const FloatT f0, a0, b0, e0;

    /**
     * Convert to ECEF (Earth-centered Earth-Fixed)
     *
     */
    System_XYZ<FloatT, Earth> xyz() const {
      const FloatT &_lat (const_cast<self_t *>(this)->latitude());   ///< 緯度[rad]
      const FloatT &_lng(const_cast<self_t *>(this)->longitude());  ///< 経度[rad]
      const FloatT &_h   (const_cast<self_t *>(this)->height());     ///< 高度[m]
      
      FloatT n(a0/std::sqrt(1.0 - pow2(e0) * pow2(std::sin(_lat))));
      
      return System_XYZ<FloatT, Earth>(
          (n + _h) * std::cos(_lat) * std::cos(_lng),
          (n + _h) * std::cos(_lat) * std::sin(_lng),
          (n * (1.0 -pow2(e0)) + _h) * std::sin(_lat)
        );
    }

    friend std::ostream &operator<<(std::ostream &out, const self_t &self){
      out << (const_cast<self_t *>(&self)->latitude() / M_PI * 180) << " "
          << (const_cast<self_t *>(&self)->longitude() / M_PI * 180) << " "
          << const_cast<self_t *>(&self)->height();
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

template <class FloatT, class Earth>
const FloatT System_LLH<FloatT, Earth>::f0(Earth::F_e);
template <class FloatT, class Earth>
const FloatT System_LLH<FloatT, Earth>::a0(Earth::R_e);
template <class FloatT, class Earth>
const FloatT System_LLH<FloatT, Earth>::b0(a0 * (1.0 - f0));
template <class FloatT, class Earth>
const FloatT System_LLH<FloatT, Earth>::e0(std::sqrt(f0 * (2.0 - f0)));

template <class FloatT = double, class Earth = WGS84>
class System_ENU : public System_3D<FloatT> {
  protected:
    typedef System_ENU<FloatT, Earth> self_t;
    typedef System_3D<FloatT> super_t;
    typedef System_XYZ<FloatT, Earth> xyz_t;
    typedef System_LLH<FloatT, Earth> llh_t;
  public:
    System_ENU() : super_t() {}
    System_ENU(const FloatT &east, const FloatT &north, const FloatT &up) 
        : super_t(east, north, up) {}
    System_ENU(const self_t &enu) 
        : super_t(enu) {}
    System_ENU(const Vector3<FloatT> &vec)
        : super_t(vec) {}
    System_ENU(const Matrix<FloatT> &mat)
        : super_t(mat) {}
    ~System_ENU() {}
    
    FloatT &east()  {return super_t::operator[](0);}  ///< 東成分[m]
    FloatT &north() {return super_t::operator[](1);}  ///< 北成分[m]
    FloatT &up()    {return super_t::operator[](2);}  ///< 上成分[m]
    
    static self_t relative_rel(const xyz_t &rel_pos, const llh_t &base_llh){
      const FloatT &rel_x(const_cast<xyz_t &>(rel_pos).x());
      const FloatT &rel_y(const_cast<xyz_t &>(rel_pos).y());
      const FloatT &rel_z(const_cast<xyz_t &>(rel_pos).z());
      
      FloatT s1(std::sin(const_cast<llh_t &>(base_llh).longitude())),
          c1(std::cos(const_cast<llh_t &>(base_llh).longitude())),
          s2(std::sin(const_cast<llh_t &>(base_llh).latitude())),
          c2(std::cos(const_cast<llh_t &>(base_llh).latitude()));
      
      return self_t(
          -rel_x * s1 + rel_y * c1,
          -rel_x * c1 * s2 - rel_y * s1 * s2 + rel_z * c2,
          rel_x * c1 * c2 + rel_y * s1 * c2 + rel_z * s2
        );
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
      
      const FloatT &_east(const_cast<self_t *>(this)->east());
      const FloatT &_north(const_cast<self_t *>(this)->north());
      const FloatT &_up(const_cast<self_t *>(this)->up());
      
      return xyz_t(
          -_east * s1 - _north * c1 * s2 + _up * c1 * c2 
            + const_cast<xyz_t *>(&base)->x(),
          _east * c1 - _north * s1 * s2 + _up * s1 * c2
            + const_cast<xyz_t *>(&base)->y(),
          _north * c2 + _up * s2
            + const_cast<xyz_t *>(&base)->z()
        );
    }
    
    FloatT elevation() const {
      const FloatT &_east(const_cast<self_t *>(this)->east());
      const FloatT &_north(const_cast<self_t *>(this)->north());
      const FloatT &_up(const_cast<self_t *>(this)->up());
      return FloatT(std::atan2(_up, std::sqrt(pow2(_east) + pow2(_north))));
    }
    FloatT azimuth() const {
      const FloatT &_east(const_cast<self_t *>(this)->east());
      const FloatT &_north(const_cast<self_t *>(this)->north());
      return FloatT(std::atan2(_east, _north));
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
