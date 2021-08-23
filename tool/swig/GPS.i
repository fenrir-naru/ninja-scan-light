/**
 * @file SWIG interface file for GPS related classes
 *
 */

%module GPS

%{
#if defined(SWIGRUBY)
#undef isfinite
#endif

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "navigation/GPS.h"
#include "navigation/RINEX.h"
%}

%include typemaps.i
%include std_string.i
%include exception.i

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

%feature("autodoc", "1");

%import "SylphideMath.i"
%import "Coordinate.i"

%extend GPS_Time {
  %typemap(out) std::tm {
    %append_output(SWIG_From(int)($1.tm_year + 1900));
    %append_output(SWIG_From(int)($1.tm_mon + 1));
    %append_output(SWIG_From(int)($1.tm_mday));
    %append_output(SWIG_From(int)($1.tm_hour));
    %append_output(SWIG_From(int)($1.tm_min));
    %append_output(SWIG_From(int)($1.tm_sec));
  }
#if defined(SWIGRUBY)
  %typemap(in) const std::tm & (std::tm temp = {0}) {
    $1 = &temp;
    int *dst[] = {
      &(temp.tm_year),
      &(temp.tm_mon),
      &(temp.tm_mday),
      &(temp.tm_hour),
      &(temp.tm_min),
      &(temp.tm_sec),
    };
    int i_max(sizeof(dst) / sizeof(dst[0]));
    if(i_max > RARRAY_LEN($input)){i_max = RARRAY_LEN($input);}
    for(int i(0); i < i_max; ++i){
      SWIG_Object obj = rb_ary_entry($input, i);
      int v;
      if(SWIG_IsOK(SWIG_AsVal(int)(obj, &v))){
        if(dst[i] == &(temp.tm_year)){
          *dst[i] = v - 1900;
        }else if(dst[i] == &(temp.tm_mon)){
          *dst[i] = v - 1;
        }else{
          *dst[i] = v;
        }
      }else{
        SWIG_exception(SWIG_TypeError, "int is expected");
      }
    }
  }
  %typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) const std::tm & {
    $1 = (TYPE($input) == T_ARRAY) ? 1 : 0;
  }
#endif
  %apply int *OUTPUT { int *week };
  %apply FloatT *OUTPUT { FloatT *seconds };
  void to_a(int *week, FloatT *seconds) const {
    *week = self->week;
    *seconds = self->seconds;
  }
}

%define MAKE_ACCESSOR(name, type)
%rename(%str(name ## =)) set_ ## name;
type set_ ## name (const type &v) {
  return self->name = v;
}
%rename(%str(name)) get_ ## name;
const type &get_ ## name () const {
  return self->name;
}
%enddef

%inline %{
template <class FloatT>
struct GPS_Ionospheric_UTC_Parameters : public GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters {};
%}
%extend GPS_Ionospheric_UTC_Parameters {
  %typemap(in,numinputs=0) FloatT values[4] (FloatT temp[4]) %{
    $1 = temp;
  %}
  %typemap(argout) FloatT values[4] {
    for(int i(0); i < 4; ++i){
      %append_output(SWIG_From(double)((double)($1[i])));
    }
  }
  %typemap(in) const FloatT values[4] (FloatT temp[4]) {
#ifdef SWIGRUBY
    if(!(RB_TYPE_P($input, T_ARRAY) && (RARRAY_LEN($input) == 4))){
      SWIG_exception(SWIG_TypeError, "array[4] is expected");
    }
    for(int i(0); i < 4; ++i){
      SWIG_Object obj(RARRAY_AREF($input, i));
      double v;
      if(SWIG_IsOK(SWIG_AsVal(double)(obj, &v))){
        temp[i] = (FloatT)v;
      }else{
        SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
      }
    }
#endif
    $1 = temp;
  }
  %typemap(default) (const unsigned int *buf) {
    $1 = NULL;
  }
  %typemap(in) const unsigned int *buf {
#ifdef SWIGRUBY
    if(!RB_TYPE_P($input, T_ARRAY)){
      SWIG_exception(SWIG_TypeError, "array is expected");
    }
    $1 = new unsigned int [RARRAY_LEN($input)];
    for(int i(0); i < RARRAY_LEN($input); ++i){
      SWIG_Object obj(RARRAY_AREF($input, i));
      if(!SWIG_IsOK(SWIG_AsVal(unsigned int)(obj, &($1[i])))){
        SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
      }
    }
#endif
  }
  %typemap(freearg) const unsigned int *buf {
    delete [] $1;
  }
  %rename("alpha=") set_alpha;
  void set_alpha(const FloatT values[4]){
    for(int i(0); i < 4; ++i){
      self->alpha[i] = values[i];
    } 
  }
  %rename("alpha") get_alpha;
  void get_alpha(FloatT values[4]) const {
    for(int i(0); i < 4; ++i){
      values[i] = self->alpha[i];
    }
  }
  %rename("beta=") set_beta;
  void set_beta(const FloatT values[4]){
    for(int i(0); i < 4; ++i){
      self->beta[i] = values[i];
    }
  }
  %rename("beta") get_beta;
  void get_beta(FloatT values[4]) const {
    for(int i(0); i < 4; ++i){
      values[i] = self->beta[i];
    }
  }
  MAKE_ACCESSOR(A1, FloatT);
  MAKE_ACCESSOR(A0, FloatT);
  MAKE_ACCESSOR(t_ot, unsigned int);
  MAKE_ACCESSOR(WN_t, unsigned int);
  MAKE_ACCESSOR(delta_t_LS, int);
  MAKE_ACCESSOR(WN_LSF, unsigned int);
  MAKE_ACCESSOR(DN, unsigned int);
  MAKE_ACCESSOR(delta_t_LSF, int);
  static GPS_Ionospheric_UTC_Parameters<FloatT> parse(const unsigned int *buf){
    typename GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters::raw_t raw;
    raw.template update<2, 0>(buf);
    GPS_Ionospheric_UTC_Parameters<FloatT> res;
    (typename GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters &)res = raw;
    return res;
  }
}

%inline %{
template <class FloatT>
struct GPS_Ephemeris : public GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris {
  GPS_Ephemeris() : GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris() {
    this->iodc = -1;
  }
};
%}
%extend GPS_Ephemeris {
  MAKE_ACCESSOR(svid, unsigned int);
          
  MAKE_ACCESSOR(WN, unsigned int);
  MAKE_ACCESSOR(URA, int);
  MAKE_ACCESSOR(SV_health, unsigned int);
  MAKE_ACCESSOR(iodc, int);
  MAKE_ACCESSOR(t_GD, FloatT);
  MAKE_ACCESSOR(t_oc, FloatT);
  MAKE_ACCESSOR(a_f2, FloatT);
  MAKE_ACCESSOR(a_f1, FloatT);
  MAKE_ACCESSOR(a_f0, FloatT);

  MAKE_ACCESSOR(iode, int);
  MAKE_ACCESSOR(c_rs, FloatT);
  MAKE_ACCESSOR(delta_n, FloatT);
  MAKE_ACCESSOR(M0, FloatT);
  MAKE_ACCESSOR(c_uc, FloatT);
  MAKE_ACCESSOR(e, FloatT);
  MAKE_ACCESSOR(c_us, FloatT);
  MAKE_ACCESSOR(sqrt_A, FloatT);
  MAKE_ACCESSOR(t_oe, FloatT);
  MAKE_ACCESSOR(fit_interval, FloatT);
          
  MAKE_ACCESSOR(c_ic, FloatT);
  MAKE_ACCESSOR(Omega0, FloatT);
  MAKE_ACCESSOR(c_is, FloatT);
  MAKE_ACCESSOR(i0, FloatT);
  MAKE_ACCESSOR(c_rc, FloatT);
  MAKE_ACCESSOR(omega, FloatT);
  MAKE_ACCESSOR(dot_Omega0, FloatT);
  MAKE_ACCESSOR(dot_i0, FloatT);
  %typemap(default) (const unsigned int *buf) {
    $1 = NULL;
  }
  %typemap(in) const unsigned int *buf {
#ifdef SWIGRUBY
    if(!RB_TYPE_P($input, T_ARRAY)){
      SWIG_exception(SWIG_TypeError, "array is expected");
    }
    $1 = new unsigned int [RARRAY_LEN($input)];
    for(int i(0); i < RARRAY_LEN($input); ++i){
      SWIG_Object obj(RARRAY_AREF($input, i));
      if(!SWIG_IsOK(SWIG_AsVal(unsigned int)(obj, &($1[i])))){
        SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
      }
    }
#endif
  }
  %typemap(freearg) const unsigned int *buf {
    delete [] $1;
  }
  int parse(const unsigned int &subframe_no, const unsigned int *buf){
    int res;
    typedef typename GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris eph_t;
    typename eph_t::raw_t raw;
    eph_t eph;
    switch(subframe_no){
      case 1: 
        res = raw.template update_subframe1<2, 0>(buf);
        eph = raw;
        self->WN = eph.WN;
        self->URA = eph.URA;
        self->SV_health = eph.SV_health;
        self->iodc = eph.iodc;
        self->t_GD = eph.t_GD;
        self->t_oc = eph.t_oc;
        self->a_f2 = eph.a_f2;
        self->a_f1 = eph.a_f1;
        self->a_f0 = eph.a_f0;
        break;
      case 2: 
        res = raw.template update_subframe2<2, 0>(buf);
        eph = raw;
        if((self->iodc < 0) || ((self->iodc & 0xFF) != res)){return -1;}
        self->iode = eph.iode;
        self->c_rs = eph.c_rs;
        self->delta_n = eph.delta_n;
        self->M0 = eph.M0;
        self->c_uc = eph.c_uc;
        self->e = eph.e;
        self->c_us = eph.c_us;
        self->sqrt_A = eph.sqrt_A;
        self->t_oe = eph.t_oe;
        self->fit_interval = eph_t::raw_t::fit_interval(raw.fit_interval_flag, self->iodc);
        break;
      case 3: 
        res = raw.template update_subframe3<2, 0>(buf);
        eph = raw;
        if((self->iodc < 0) || ((self->iodc & 0xFF) != res)){return -1;}
        self->c_ic = eph.c_ic;
        self->Omega0 = eph.Omega0;
        self->c_is = eph.c_is;
        self->i0 = eph.i0;
        self->c_rc = eph.c_rc;
        self->omega = eph.omega;
        self->dot_Omega0 = eph.dot_Omega0;
        self->dot_i0 = eph.dot_i0;
        break;
      default:
        return -1;
    }
    return res;
  }
  %typemap(in,numinputs=0) System_XYZ<FloatT, WGS84> & (System_XYZ<FloatT, WGS84> temp) %{
    $1 = &temp;
  %}
  %typemap(argout) System_XYZ<FloatT, WGS84> & {
    %append_output(SWIG_NewPointerObj((new $*1_ltype(*$1)), $1_descriptor, SWIG_POINTER_OWN));
  }
  void constellation(
      System_XYZ<FloatT, WGS84> &position, System_XYZ<FloatT, WGS84> &velocity,
      const GPS_Time<FloatT> &t, const FloatT &pseudo_range = 0,
      const bool &with_velocity = true) const {
    typename GPS_SpaceNode<FloatT>::SatelliteProperties::constellation_t res(
        self->constellation(t, pseudo_range, with_velocity));
    position = res.position;
    velocity = res.velocity;
  }
}

#undef MAKE_ACCESSOR

%extend GPS_SpaceNode {
  %typemap(out) const Ionospheric_UTC_Parameters & {
    %set_output(SWIG_NewPointerObj(
        %reinterpret_cast($1, GPS_Ionospheric_UTC_Parameters<FloatT> *),
        $descriptor(GPS_Ionospheric_UTC_Parameters<FloatT> *), 0));
  }
  %ignore satellites() const;
  %ignore satellite(const int &);
  void register_ephemeris(
      const int &prn, const GPS_Ephemeris<FloatT> &eph,
      const int &priority_delta = 1){
    self->satellite(prn).register_ephemeris(eph, priority_delta);
  }
  const GPS_Ephemeris<FloatT> &ephemeris(const int &prn) const {
    return %reinterpret_cast(
        %const_cast(self, GPS_SpaceNode<FloatT> *)->satellite(prn).ephemeris(), 
        const GPS_Ephemeris<FloatT> &);
  }
  %typemap(out) pierce_point_res_t {
    %append_output(SWIG_From(double)((double)($1.latitude)));
    %append_output(SWIG_From(double)((double)($1.longitude)));
  }
  %ignore iono_correction() const;
  %ignore tropo_correction() const;
  int read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    return RINEX_NAV_Reader<FloatT>::read_all(fin, *self);
  }
}

%include navigation/GPS.h

%define CONCRETIZE(type)
%template(Time) GPS_Time<type>;
%template(SpaceNode) GPS_SpaceNode<type>;
%template(Ionospheric_UTC_Parameters) GPS_Ionospheric_UTC_Parameters<type>;
%template(Ephemeris) GPS_Ephemeris<type>;
%enddef

CONCRETIZE(double);

#undef CONCRETIZE