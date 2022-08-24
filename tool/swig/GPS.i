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
#include <exception>

#include "navigation/GPS.h"
#include "navigation/GLONASS.h"
#include "navigation/RINEX.h"
#include "navigation/RINEX_Clock.h"
#include "navigation/SP3.h"
#include "navigation/ANTEX.h"

#include "navigation/GPS_Solver_Base.h"
#include "navigation/GPS_Solver.h"
#include "navigation/GPS_Solver_RAIM.h"
#include "navigation/GLONASS_Solver.h"

#if defined(__cplusplus) && (__cplusplus < 201103L)
#include <sstream>
namespace std {
template <class T>
inline std::string to_string(const T &value){
  // @see https://stackoverflow.com/a/5590404/15992898
  return static_cast<std::ostringstream &>(std::ostringstream() << value).str();
}
}
#endif
%}

%include typemaps.i
%include std_common.i
%include std_string.i
%include exception.i

#if !defined(SWIGIMPORTED)
%header {
struct native_exception : public std::exception {
#if defined(SWIGRUBY)
  int state;
  native_exception(const int &state_) : std::exception(), state(state_) {}
  void regenerate() const {rb_jump_tag(state);}
#else
  void regenerate() const {}
#endif
};
}
%exception {
  try {
    $action
  } catch (const native_exception &e) {
    e.regenerate();
    SWIG_fail;
  } catch (const std::exception& e) {
    SWIG_exception_fail(SWIG_RuntimeError, e.what());
  }
}
#endif

#ifdef SWIGRUBY
%header {
static VALUE yield_throw_if_error(const int &argc, const VALUE *argv) {
  struct yield_t {
    const int &argc;
    const VALUE *argv;
    static VALUE run(VALUE v){
      yield_t *arg(reinterpret_cast<yield_t *>(v));
      return rb_yield_values2(arg->argc, arg->argv);
    }
  } arg = {argc, argv};
  int state;
  VALUE res(rb_protect(yield_t::run, reinterpret_cast<VALUE>(&arg), &state));
  if(state != 0){throw native_exception(state);}
  return res;
}
static VALUE proc_call_throw_if_error(
    const VALUE &arg0, const int &argc, const VALUE *argv) {
  struct proc_call_t {
    const VALUE &arg0;
    const int &argc;
    const VALUE *argv;
    static VALUE run(VALUE v){
      proc_call_t *arg(reinterpret_cast<proc_call_t *>(v));
      return rb_proc_call_with_block(arg->arg0, arg->argc, arg->argv, Qnil);
    }
  } arg = {arg0, argc, argv};
  int state;
  VALUE res(rb_protect(proc_call_t::run, reinterpret_cast<VALUE>(&arg), &state));
  if(state != 0){throw native_exception(state);}
  return res;
}
static std::string inspect_str(const VALUE &v){
  VALUE v_inspect(rb_inspect(v));
  return std::string(RSTRING_PTR(v_inspect), RSTRING_LEN(v_inspect));
}
}
#endif

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
  %ignore canonicalize();
  %ignore GPS_Time(const int &_week, const float_t &_seconds);
  %typemap(in, numinputs=0) void *dummy "";
  GPS_Time(const int &week_, const float_t &seconds_, void *dummy){
    return &((new GPS_Time<FloatT>(week_, seconds_))->canonicalize());
  }
  %apply int *OUTPUT { int *week };
  %apply FloatT *OUTPUT { FloatT *seconds };
  void to_a(int *week, FloatT *seconds) const {
    *week = self->week;
    *seconds = self->seconds;
  }
#if defined(SWIG)  
  int __cmp__(const GPS_Time<FloatT> &t) const {
    return ((self->week < t.week) ? -1 
        : ((self->week > t.week) ? 1 
          : (self->seconds < t.seconds ? -1 
            : (self->seconds > t.seconds ? 1 : 0))));
  }
#endif
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

%define MAKE_VECTOR2ARRAY(type)
%typemap(out) std::vector<type> {
#if defined(SWIGRUBY)
  $result = rb_ary_new();
#endif
  for($1_type::const_iterator it($1.begin()), it_end($1.end());
      it != it_end; ++it){
    %append_output(SWIG_From(type)(*it));
  }
}
%enddef
#if defined(SWIGRUBY)
%define MAKE_ARRAY_INPUT(type, arg_name, f_conv)
%typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) type arg_name[ANY] {
  $1 = RB_TYPE_P($input, T_ARRAY) ? 1 : 0;
}
%typemap(in) type arg_name[ANY] ($*1_ltype temp[$1_dim0]) {
  if(!(RB_TYPE_P($input, T_ARRAY) && (RARRAY_LEN($input) == $1_dim0))){
    SWIG_exception(SWIG_TypeError, "array[$1_dim0] is expected");
  }
  for(int i(0); i < $1_dim0; ++i){
    if(!SWIG_IsOK(f_conv(RARRAY_AREF($input, i), &temp[i]))){
      SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
    }
  }
  $1 = temp;
}
%enddef
#else
#define MAKE_ARRAY_INPUT(type, arg_name, f_conv)
#endif

%inline %{
template <class FloatT>
struct GPS_Ionospheric_UTC_Parameters : public GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters {};
%}
%extend GPS_Ionospheric_UTC_Parameters {
  %fragment(SWIG_Traits_frag(FloatT));
  %typemap(in,numinputs=0) const FloatT *values[4] (FloatT *temp) "$1 = &temp;"
  %typemap(argout) const FloatT *values[4] {
    for(int i(0); i < 4; ++i){
      %append_output(swig::from((*$1)[i]));
    }
  }
  MAKE_ARRAY_INPUT(const FloatT, values, swig::asval);
  MAKE_ARRAY_INPUT(const unsigned int, buf, SWIG_AsVal(unsigned int));
  %rename("alpha=") set_alpha;
  void set_alpha(const FloatT values[4]){
    for(int i(0); i < 4; ++i){
      self->alpha[i] = values[i];
    } 
  }
  %rename("alpha") get_alpha;
  void get_alpha(const FloatT *values[4]) const {*values = self->alpha;}
  %rename("beta=") set_beta;
  void set_beta(const FloatT values[4]){
    for(int i(0); i < 4; ++i){
      self->beta[i] = values[i];
    }
  }
  %rename("beta") get_beta;
  void get_beta(const FloatT *values[4]) const {*values = self->beta;}
  MAKE_ACCESSOR(A1, FloatT);
  MAKE_ACCESSOR(A0, FloatT);
  MAKE_ACCESSOR(t_ot, unsigned int);
  MAKE_ACCESSOR(WN_t, unsigned int);
  MAKE_ACCESSOR(delta_t_LS, int);
  MAKE_ACCESSOR(WN_LSF, unsigned int);
  MAKE_ACCESSOR(DN, unsigned int);
  MAKE_ACCESSOR(delta_t_LSF, int);
  static GPS_Ionospheric_UTC_Parameters<FloatT> parse(const unsigned int buf[10]){
    typedef typename GPS_SpaceNode<FloatT>
        ::BroadcastedMessage<unsigned int, 30> parser_t;
    if((parser_t::subframe_id(buf) != 4) || (parser_t::sv_page_id(buf) != 56)){
      throw std::runtime_error("Not valid data");
    }
    typename GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters::raw_t raw;
    raw.update<2, 0>(buf);
    GPS_Ionospheric_UTC_Parameters<FloatT> res;
    (typename GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters &)res = raw;
    return res;
  }
}

%inline %{
template <class FloatT>
struct GPS_Ephemeris : public GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris {
  int iode_subframe3;
  void invalidate() {
    this->iodc = this->iode = iode_subframe3 = -1; 
  }
  bool is_consistent() const {
    return !((this->iodc < 0) 
        || (this->iode != this->iode_subframe3)
        || ((this->iodc & 0xFF) != this->iode));
  }
  GPS_Ephemeris() : GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris() {
    invalidate();
  }
  GPS_Ephemeris(const typename GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris &eph) 
      : GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris(eph),
      iode_subframe3(eph.iode) {}
  struct constellation_res_t {
    System_XYZ<FloatT, WGS84> position, velocity;
    FloatT clock_error, clock_error_dot;
  };
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
  
  MAKE_ARRAY_INPUT(const unsigned int, buf, SWIG_AsVal(unsigned int));
  %apply int *OUTPUT { int *subframe_no, int *iodc_or_iode };
  void parse(const unsigned int buf[10], int *subframe_no, int *iodc_or_iode){
    typedef typename GPS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris eph_t;
    typename eph_t::raw_t raw;
    eph_t eph;
    *subframe_no = GPS_SpaceNode<FloatT>
        ::BroadcastedMessage<unsigned int, 30>
        ::subframe_id(buf);
    *iodc_or_iode = -1; 
    switch(*subframe_no){
      case 1: 
        *iodc_or_iode = raw.update_subframe1<2, 0>(buf);
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
        *iodc_or_iode = raw.update_subframe2<2, 0>(buf);
        eph = raw;
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
        *iodc_or_iode = self->iode_subframe3 = raw.update_subframe3<2, 0>(buf);
        eph = raw;
        self->c_ic = eph.c_ic;
        self->Omega0 = eph.Omega0;
        self->c_is = eph.c_is;
        self->i0 = eph.i0;
        self->c_rc = eph.c_rc;
        self->omega = eph.omega;
        self->dot_Omega0 = eph.dot_Omega0;
        self->dot_i0 = eph.dot_i0;
        break;
    }
  }
  %typemap(out) constellation_res_t {
    %append_output(SWIG_NewPointerObj((new System_XYZ<FloatT, WGS84>($1.position)), 
        $descriptor(System_XYZ<FloatT, WGS84> *), SWIG_POINTER_OWN));
    %append_output(SWIG_NewPointerObj((new System_XYZ<FloatT, WGS84>($1.velocity)), 
        $descriptor(System_XYZ<FloatT, WGS84> *), SWIG_POINTER_OWN));
    %append_output(swig::from($1.clock_error));
    %append_output(swig::from($1.clock_error_dot));
  }
  typename GPS_Ephemeris<FloatT>::constellation_res_t constellation(
      const GPS_Time<FloatT> &t_tx, const FloatT &dt_transit = 0) const {
    typename GPS_SpaceNode<FloatT>::SatelliteProperties::constellation_t pv(
        self->constellation(t_tx, dt_transit, true));
    typename GPS_Ephemeris<FloatT>::constellation_res_t res = {
        pv.position, pv.velocity, self->clock_error(t_tx), self->clock_error_dot(t_tx)};
    return res;
  }
#if defined(SWIGRUBY)
  %rename("consistent?") is_consistent;
#endif
}

%extend GPS_SpaceNode {
  %fragment(SWIG_Traits_frag(FloatT));
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
  GPS_Ephemeris<FloatT> ephemeris(const int &prn) const {
    return GPS_Ephemeris<FloatT>(
        %const_cast(self, GPS_SpaceNode<FloatT> *)->satellite(prn).ephemeris());
  }
  %typemap(out) pierce_point_res_t {
    %append_output(swig::from($1.latitude));
    %append_output(swig::from($1.longitude));
  }
  int read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    return RINEX_NAV_Reader<FloatT>::read_all(fin, *self);
  }
}

%include navigation/GPS.h

%inline %{
template <class FloatT>
struct GLONASS_Ephemeris 
    : public GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris_with_GPS_Time {
  typedef typename GLONASS_SpaceNode<FloatT>::SatelliteProperties::Ephemeris_with_GPS_Time eph_t;
  unsigned int super_frame, has_string;
  typename eph_t::raw_t raw;
  void invalidate() {
    super_frame = 0;
    has_string = 0;
  }
  bool is_consistent() const {
    return has_string == 0x1F;
  }
  GLONASS_Ephemeris() : eph_t() {
    invalidate();
  }
  GLONASS_Ephemeris(const eph_t &eph) 
      : eph_t(eph),
      super_frame(0), has_string(0), raw() {
    raw = *this;
    has_string = 0x1F;
  }
};
%}
%extend GLONASS_Ephemeris {
  MAKE_ACCESSOR(svid, unsigned int);
  
  MAKE_ACCESSOR(freq_ch, int); // frequency channel to be configured
  MAKE_ACCESSOR(t_k, unsigned int);
  MAKE_ACCESSOR(t_b, unsigned int);
  MAKE_ACCESSOR(M, unsigned int);
  MAKE_ACCESSOR(gamma_n, FloatT);
  MAKE_ACCESSOR(tau_n, FloatT);

  MAKE_ACCESSOR(xn, FloatT); MAKE_ACCESSOR(xn_dot, FloatT); MAKE_ACCESSOR(xn_ddot, FloatT);
  MAKE_ACCESSOR(yn, FloatT); MAKE_ACCESSOR(yn_dot, FloatT); MAKE_ACCESSOR(yn_ddot, FloatT);
  MAKE_ACCESSOR(zn, FloatT); MAKE_ACCESSOR(zn_dot, FloatT); MAKE_ACCESSOR(zn_ddot, FloatT);

  MAKE_ACCESSOR(B_n, unsigned int);
  MAKE_ACCESSOR(p, unsigned int);
  MAKE_ACCESSOR(N_T, unsigned int);
  MAKE_ACCESSOR(F_T, FloatT);
  MAKE_ACCESSOR(n, unsigned int);
  MAKE_ACCESSOR(delta_tau_n, FloatT);
  MAKE_ACCESSOR(E_n, unsigned int);
  MAKE_ACCESSOR(P1, unsigned int);
  MAKE_ACCESSOR(P2, bool);
  MAKE_ACCESSOR(P4, bool);

  MAKE_ACCESSOR(tau_c, FloatT);
  MAKE_ACCESSOR(tau_GPS, FloatT);
  
  FloatT frequency_L1() const {
    return self->L1_frequency();
  };
  FloatT frequency_L2() const {
    return self->L2_frequency();
  };
  GPS_Time<FloatT> base_time() const {
    return self->base_time();
  }

  //MAKE_ACCESSOR(l_n, bool); // exists in both Ephemeris and Time_Properties

  MAKE_ARRAY_INPUT(const unsigned int, buf, SWIG_AsVal(unsigned int));
  bool parse(const unsigned int buf[4], const unsigned int &leap_seconds = 0){
    typedef typename GLONASS_SpaceNode<FloatT>
        ::template BroadcastedMessage<unsigned int> parser_t;
    unsigned int super_frame(buf[3] >> 16), frame(buf[3] & 0xF), string_no(parser_t::m(buf));
    unsigned int has_string(self->has_string);
    if((has_string > 0) && (self->super_frame != super_frame)){
      has_string = 0; // clean up
    }
    self->super_frame = super_frame;
    has_string |= (0x1 << (string_no - 1));
    switch(string_no){
      case 1: self->raw.template update_string1<0, 0>(buf); break;
      case 2: self->raw.template update_string2<0, 0>(buf); break;
      case 3: self->raw.template update_string3<0, 0>(buf); break;
      case 4: self->raw.template update_string4<0, 0>(buf); break;
      case 5: {
        self->raw.template update_string5<0, 0>(buf);
        if(frame == 4){
          // TODO: require special care for 50th frame? @see Table 4.9 note (4)
        }
        break;
      }
    }
    bool updated(false);
    if((has_string == 0x1F) && (has_string != self->has_string)){
      updated = true;
      // All ephemeris and time info. in the same super frame has been acquired, 
      // and this block is called once per one same super frame.
      // Ephemeris_with_Time::raw_t =(cast)=> Ephemeris_with_Time => Ephemeris_with_GPS_Time
      static_cast<GLONASS_Ephemeris<FloatT>::eph_t &>(*self) 
          = GLONASS_Ephemeris<FloatT>::eph_t(self->raw);
      self->t_b_gps += leap_seconds;
    }
    self->has_string = has_string;
    return updated;
  }
  typename GPS_Ephemeris<FloatT>::constellation_res_t constellation(
      const GPS_Time<FloatT> &t_tx, const FloatT &dt_transit = 0) const {
    typename GPS_SpaceNode<FloatT>::SatelliteProperties::constellation_t pv(
        self->constellation(t_tx, dt_transit));
    typename GPS_Ephemeris<FloatT>::constellation_res_t res = {
        pv.position, pv.velocity, self->clock_error(t_tx), self->clock_error_dot()};
    return res;
  }
#if defined(SWIGRUBY)
  %rename("consistent?") is_consistent;
  %rename("in_range?") is_in_range;
#endif
  bool is_in_range(const GPS_Time<FloatT> &t) const {
    // "invalidate()" is used to make raw and converted data inconsistent.
    return self->is_valid(t);
  }
}

%extend GLONASS_SpaceNode {
  %fragment(SWIG_Traits_frag(FloatT));
  %ignore satellites() const;
  %ignore satellite(const int &);
  %ignore latest_ephemeris() const;
  void register_ephemeris(
      const int &prn, const GLONASS_Ephemeris<FloatT> &eph,
      const int &priority_delta = 1){
    self->satellite(prn).register_ephemeris(eph, priority_delta);
  }
  GLONASS_Ephemeris<FloatT> ephemeris(const int &prn) const {
    return GLONASS_Ephemeris<FloatT>(
        %const_cast(self, GLONASS_SpaceNode<FloatT> *)->satellite(prn).ephemeris());
  }
  int read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    typename RINEX_NAV_Reader<FloatT>::space_node_list_t list = {NULL};
    list.glonass = self;
    return RINEX_NAV_Reader<FloatT>::read_all(fin, list);
  }
}

%include navigation/GLONASS.h

%extend GPS_User_PVT {
  %ignore solver_t;
  %ignore base_t;
  %ignore proxy_t;
  %ignore linear_solver;
  %ignore GPS_User_PVT(const base_t &);
  MAKE_VECTOR2ARRAY(int);
#ifdef SWIGRUBY
  %rename("position_solved?") position_solved;
  %rename("velocity_solved?") velocity_solved;
#endif
  %fragment(SWIG_Traits_frag(FloatT));
  %typemap(in, numinputs=0) const typename base_t::detection_t ** (typename base_t::detection_t *temp) {
    $1 = &temp;
  }
  %typemap(argout) const typename base_t::detection_t ** {
    if((*$1)->valid){
      %append_output(swig::from((*$1)->wssr));
      %append_output(swig::from((*$1)->wssr_sf));
      %append_output(swig::from((*$1)->weight_max));
      %append_output(swig::from((*$1)->slope_HV[0].max));
      %append_output(SWIG_From(int)(((*$1)->slope_HV[0].prn)));
      %append_output(swig::from((*$1)->slope_HV[1].max));
      %append_output(SWIG_From(int)(((*$1)->slope_HV[1].prn)));
    }
  }
  %typemap(in, numinputs=0) const typename base_t::exclusion_t ** (typename base_t::exclusion_t *temp) {
    $1 = &temp;
  }
  %typemap(argout) const typename base_t::exclusion_t ** {
    if((*$1)->valid){
      %append_output(SWIG_From(int)(((*$1)->excluded)));
      %append_output(SWIG_NewPointerObj(
          (new System_XYZ<FloatT, WGS84>((*$1)->user_position.xyz)),
          $descriptor(System_XYZ<FloatT, WGS84>), SWIG_POINTER_OWN));
      %append_output(SWIG_NewPointerObj(
          (new System_LLH<FloatT, WGS84>((*$1)->user_position.llh)),
          $descriptor(System_LLH<FloatT, WGS84>), SWIG_POINTER_OWN));
    }
  }
}
%inline %{
template <class FloatT>
struct GPS_User_PVT 
    : protected GPS_Solver_RAIM_LSR<FloatT,
      GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> > >::user_pvt_t {
  typedef GPS_Solver_RAIM_LSR<FloatT, 
      GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> > > solver_t;
  typedef typename solver_t::user_pvt_t base_t;
  GPS_User_PVT() : base_t() {}
  GPS_User_PVT(const base_t &base) : base_t(base) {}
  enum {
    ERROR_NO = 0,
    ERROR_UNSOLVED,
    ERROR_INVALID_IONO_MODEL,
    ERROR_INSUFFICIENT_SATELLITES,
    ERROR_POSITION_LS,
    ERROR_POSITION_NOT_CONVERGED,
    ERROR_DOP,
    ERROR_VELOCITY_SKIPPED,
    ERROR_VELOCITY_INSUFFICIENT_SATELLITES,
    ERROR_VELOCITY_LS,
  };
  int error_code() const {return (int)(base_t::error_code);}
  const GPS_Time<FloatT> &receiver_time() const {return base_t::receiver_time;}
  const System_XYZ<FloatT, WGS84> &xyz() const {return base_t::user_position.xyz;}
  const System_LLH<FloatT, WGS84> &llh() const {return base_t::user_position.llh;}
  const FloatT &receiver_error() const {return base_t::receiver_error;}
  const System_ENU<FloatT, WGS84> &velocity() const {return base_t::user_velocity_enu;}
  const FloatT &receiver_error_rate() const {return base_t::receiver_error_rate;}
  const FloatT &gdop() const {return base_t::dop.g;}
  const FloatT &pdop() const {return base_t::dop.p;}
  const FloatT &hdop() const {return base_t::dop.h;}
  const FloatT &vdop() const {return base_t::dop.v;}
  const FloatT &tdop() const {return base_t::dop.t;}
  const unsigned int &used_satellites() const {return base_t::used_satellites;}
  std::vector<int> used_satellite_list() const {return base_t::used_satellite_mask.indices_one();}
  bool position_solved() const {return base_t::position_solved();}
  bool velocity_solved() const {return base_t::velocity_solved();}
  
  const Matrix_Frozen<FloatT, Array2D_Dense<FloatT> > &G() const {return base_t::G;}
  const Matrix_Frozen<FloatT, Array2D_Dense<FloatT> > &W() const {return base_t::W;}
  const Matrix_Frozen<FloatT, Array2D_Dense<FloatT> > &delta_r() const {return base_t::delta_r;}
  struct proxy_t : public solver_t {
    typedef typename solver_t
        ::template linear_solver_t<Matrix<FloatT, Array2D_Dense<FloatT> > >
        linear_solver_t;
  };
  Matrix<FloatT, Array2D_Dense<FloatT> > G_enu() const {
    return proxy_t::linear_solver_t::rotate_G(base_t::G, base_t::user_position.ecef2enu());
  }
  typename proxy_t::linear_solver_t::partial_t linear_solver() const {
    return typename proxy_t::linear_solver_t(base_t::G, base_t::W, base_t::delta_r)
        .partial(used_satellites());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > C() const {
    return linear_solver().C();
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > C_enu() const {
    return proxy_t::linear_solver_t::rotate_C(C(), base_t::user_position.ecef2enu());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > S() const {
    Matrix<FloatT, Array2D_Dense<FloatT> > res;
    linear_solver().least_square(res);
    return res;
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > S_enu(
      const Matrix<FloatT, Array2D_Dense<FloatT> > &s) const {
    return proxy_t::linear_solver_t::rotate_S(s, base_t::user_position.ecef2enu());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > S_enu() const {
    return S_enu(S());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > slope_HV(
      const Matrix<FloatT, Array2D_Dense<FloatT> > &s) const {
    return linear_solver().slope_HV(s);
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > slope_HV() const {
    return slope_HV(S());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > slope_HV_enu(
      const Matrix<FloatT, Array2D_Dense<FloatT> > &s) const {
    return linear_solver().slope_HV(s, base_t::user_position.ecef2enu());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > slope_HV_enu() const {
    return slope_HV_enu(S());
  }
  
  void fd(const typename base_t::detection_t **out) const {*out = &(base_t::FD);}
  void fde_min(
      const typename base_t::detection_t **out0, 
      const typename base_t::exclusion_t **out1) const {
    *out0 = *out1 = &(base_t::FDE_min);
  }
  void fde_2nd(
      const typename base_t::detection_t **out0, 
      const typename base_t::exclusion_t **out1) const {
    *out0 = *out1 = &(base_t::FDE_2nd);
  }
};
%}

%extend GPS_Measurement {
  %ignore items_t;
  %ignore items;
  %exception each {
#ifdef SWIGRUBY
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_intern("each")), argc, argv);
    }
#endif
    try {
      $action
    } catch (const native_exception &e) {
      e.regenerate();
      SWIG_fail;
    } catch (const std::exception& e) {
      SWIG_exception_fail(SWIG_RuntimeError, e.what());
    }
  }
  %fragment(SWIG_Traits_frag(FloatT));
  void each() const {
    for(typename GPS_Measurement<FloatT>::items_t::const_iterator
          it(self->items.begin()), it_end(self->items.end());
        it != it_end; ++it){
      for(typename GPS_Measurement<FloatT>::items_t::mapped_type::const_iterator 
            it2(it->second.begin()), it2_end(it->second.end());
          it2 != it2_end; ++it2){
#ifdef SWIGRUBY
        VALUE values[] = {
          SWIG_From(int)(it->first), 
          SWIG_From(int)(it2->first),
          swig::from(it2->second)
        };
        yield_throw_if_error(3, values);
#endif
      }
    }
  }
  %fragment(SWIG_Traits_frag(GPS_Measurement<FloatT>), "header",
      fragment=SWIG_Traits_frag(FloatT)){
    namespace swig {
      template <>
      bool check<GPS_Measurement<FloatT> >(SWIG_Object obj) {
#ifdef SWIGRUBY
        return RB_TYPE_P(obj, T_ARRAY) || RB_TYPE_P(obj, T_HASH);
#else
        return false;
#endif
      }
      template <>
      int asval(SWIG_Object obj, GPS_Measurement<FloatT> *val) {
#ifdef SWIGRUBY
        if(RB_TYPE_P(obj, T_ARRAY)){
          int i(0), i_end(RARRAY_LEN(obj));
          VALUE values;
          for(; i < i_end; ++i){
            values = RARRAY_AREF(obj, i);
            if((!RB_TYPE_P(values, T_ARRAY)) || (RARRAY_LEN(values) < 3)){
              break;
            }
            int prn, key;
            FloatT v;
            if((!SWIG_IsOK(SWIG_AsVal(int)(RARRAY_AREF(values, 0), &prn)))
                || (!SWIG_IsOK(SWIG_AsVal(int)(RARRAY_AREF(values, 1), &key)))
                || (!SWIG_IsOK(swig::asval(RARRAY_AREF(values, 2), &v)))){
              break;
            }
            val->add(prn, key, v);
          }
          if(i < i_end){
            SWIG_exception(SWIG_TypeError, 
                std::string("Unexpected input [").append(std::to_string(i)).append("]: ")
                  .append(inspect_str(values)).c_str());
          }
          return SWIG_OK;
        }else if(RB_TYPE_P(obj, T_HASH)){
          struct arg_t {
            GPS_Measurement<FloatT> *meas;
            int prn;
            static int iter2(VALUE v_k, VALUE v_v, VALUE v_arg){
              int k;
              FloatT v;
              arg_t *arg(reinterpret_cast<arg_t *>(v_arg));
              if((!SWIG_IsOK(SWIG_AsVal(int)(v_k, &k)))
                  || (!SWIG_IsOK(swig::asval(v_v, &v)))){
                SWIG_exception(SWIG_TypeError, 
                    std::string("Unexpected input @ PRN").append(std::to_string(arg->prn)).append(": [")
                      .append(inspect_str(v_k)).append(", ")
                      .append(inspect_str(v_v)).append("]").c_str());
              }
              arg->meas->add(arg->prn, k, v);
              return ST_CONTINUE;
            }
            static int iter1(VALUE v_prn, VALUE v_key_value, VALUE v_arg){
              arg_t *arg(reinterpret_cast<arg_t *>(v_arg));
              if((!RB_TYPE_P(v_key_value, T_HASH))
                  || (!SWIG_IsOK(SWIG_AsVal(int)(v_prn, &(arg->prn))))){
                SWIG_exception(SWIG_TypeError, 
                    std::string("Unexpected input @ {")
                      .append(inspect_str(v_prn)).append(" => ")
                      .append(inspect_str(v_key_value)).append("}").c_str());
              }
              rb_hash_foreach(v_key_value,
#if RUBY_API_VERSION < 20700
                  // @see https://docs.ruby-lang.org/ja/latest/doc/news=2f2_7_0.html
                  (int (*)(ANYARGS))
#endif
                  arg_t::iter2, v_arg);
              return ST_CONTINUE;
            }
          } arg = {val};
          rb_hash_foreach(obj,
#if RUBY_API_VERSION < 20700
              (int (*)(ANYARGS))
#endif
              arg_t::iter1, reinterpret_cast<VALUE>(&arg));
          return SWIG_OK;
        }
#endif
        return SWIG_ERROR;
      }
#ifdef SWIGRUBY
      template <>
      VALUE from(const GPS_Measurement<FloatT>::items_t::mapped_type &val) {
        VALUE per_sat(rb_hash_new());
        for(typename GPS_Measurement<FloatT>::items_t::mapped_type::const_iterator 
              it(val.begin()), it_end(val.end());
            it != it_end; ++it){
          rb_hash_aset(per_sat, SWIG_From(int)(it->first), swig::from(it->second));
        }
        return per_sat;
      }
#endif
    }
  }
  %fragment(SWIG_Traits_frag(GPS_Measurement<FloatT>));
#ifdef SWIGRUBY
  VALUE to_hash() const {
    VALUE res(rb_hash_new());
    for(typename GPS_Measurement<FloatT>::items_t::const_iterator
          it(self->items.begin()), it_end(self->items.end());
        it != it_end; ++it){
      rb_hash_aset(res, SWIG_From(int)(it->first), swig::from(it->second));
    }
    return res;
  }
#endif
  %typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER) const GPS_Measurement<FloatT> & {
    $1 = SWIG_CheckState(SWIG_ConvertPtr($input, (void **)0, $1_descriptor, 0))
        || swig::check<GPS_Measurement<FloatT> >($input);
  }
  %typemap(in) const GPS_Measurement<FloatT> & (GPS_Measurement<FloatT> temp) {
    if((!SWIG_IsOK(SWIG_ConvertPtr($input, (void **)&$1, $1_descriptor, 0)))
        && (!SWIG_IsOK(swig::asval($input, ($1 = &temp))))){
      SWIG_exception(SWIG_TypeError, "in method '$symname', expecting type $*1_ltype");
    }
  }
}
%copyctor GPS_Measurement;
#ifdef SWIGRUBY
%mixin GPS_Measurement "Enumerable";
#endif
%inline {
template <class FloatT>
struct GPS_Measurement {
  typedef std::map<int, std::map<int, FloatT> > items_t;
  items_t items;
  enum {
    L1_PSEUDORANGE,
    L1_DOPPLER,
    L1_CARRIER_PHASE,
    L1_RANGE_RATE,
    L1_PSEUDORANGE_SIGMA,
    L1_DOPPLER_SIGMA,
    L1_CARRIER_PHASE_SIGMA,
    L1_RANGE_RATE_SIGMA,
    L1_SIGNAL_STRENGTH_dBHz,
    L1_LOCK_SEC,
    L1_FREQUENCY,
    ITEMS_PREDEFINED,
  };
  void add(const int &prn, const int &key, const FloatT &value){
    items[prn][key] = value;
  }
};
}

%extend GPS_SolverOptions_Common {
%define MAKE_ACCESSOR2(name, type)
%rename(%str(name ## =)) set_ ## name;
type set_ ## name (const type &v) {
  return self->cast_general()->name = v;
}
%rename(%str(name)) get_ ## name;
const type &get_ ## name () const {
  return self->cast_general()->name;
}
%enddef
  MAKE_ACCESSOR2(elevation_mask, FloatT);
  MAKE_ACCESSOR2(residual_mask, FloatT);
#undef MAKE_ACCESSOR2
  MAKE_VECTOR2ARRAY(int);
  %ignore cast_base;
}
%inline %{
template <class FloatT>
struct GPS_SolverOptions_Common {
  virtual ~GPS_SolverOptions_Common() {}
  virtual GPS_Solver_GeneralOptions<FloatT> *cast_general() = 0;
  virtual const GPS_Solver_GeneralOptions<FloatT> *cast_general() const = 0;
};
%}

%extend GPS_SolverOptions {
  %ignore base_t;
  %ignore cast_general;
  MAKE_VECTOR2ARRAY(int);
}
%inline %{
template <class FloatT>
struct GPS_SolverOptions 
    : public GPS_SinglePositioning<FloatT>::options_t, 
    GPS_SolverOptions_Common<FloatT> {
  typedef typename GPS_SinglePositioning<FloatT>::options_t base_t;
  void exclude(const int &prn){base_t::exclude_prn.set(prn);}
  void include(const int &prn){base_t::exclude_prn.reset(prn);}
  std::vector<int> excluded() const {return base_t::exclude_prn.excluded();}
  GPS_Solver_GeneralOptions<FloatT> *cast_general(){return this;}
  const GPS_Solver_GeneralOptions<FloatT> *cast_general() const {return this;}
};
%}

%extend GLONASS_SolverOptions {
  %ignore base_t;
  %ignore cast_general;
  MAKE_VECTOR2ARRAY(int);
}
%inline %{
template <class FloatT>
struct GLONASS_SolverOptions 
    : public GLONASS_SinglePositioning<FloatT>::options_t, 
    GPS_SolverOptions_Common<FloatT> {
  typedef typename GLONASS_SinglePositioning<FloatT>::options_t base_t;
  void exclude(const int &prn){base_t::exclude_prn.set(prn);}
  void include(const int &prn){base_t::exclude_prn.reset(prn);}
  std::vector<int> excluded() const {return base_t::exclude_prn.excluded();}
  GPS_Solver_GeneralOptions<FloatT> *cast_general(){return this;}
  const GPS_Solver_GeneralOptions<FloatT> *cast_general() const {return this;}
};
%}

%header {
template <class FloatT>
struct GPS_RangeCorrector
    : public GPS_Solver_Base<FloatT>::range_corrector_t {
  SWIG_Object callback;
  GPS_RangeCorrector(const SWIG_Object &callback_)
      : GPS_Solver_Base<FloatT>::range_corrector_t(),
      callback(callback_) {}
  bool is_available(const typename GPS_Solver_Base<FloatT>::gps_time_t &t) const {
    return false;
  }
  FloatT *calculate(
      const typename GPS_Solver_Base<FloatT>::gps_time_t &t, 
      const typename GPS_Solver_Base<FloatT>::pos_t &usr_pos, 
      const typename GPS_Solver_Base<FloatT>::enu_t &sat_rel_pos,
      FloatT &buf) const {
    return NULL;
  }
};
}

%extend GPS_Solver {
  %ignore super_t;
  %ignore base_t;
  %ignore gps_t;
  %ignore gps;
  %ignore glonass_t;
  %ignore glonass;
  %ignore select_solver;
  %ignore relative_property;
  %ignore select_satellite;
  %ignore update_position_solution;
  %ignore user_correctors_t;
  %ignore user_correctors;
  %immutable hooks;
  %ignore mark;
  %fragment("hook"{GPS_Solver<FloatT>}, "header",
      fragment=SWIG_From_frag(int),
      fragment=SWIG_Traits_frag(FloatT),
      fragment=SWIG_Traits_frag(GPS_Measurement<FloatT>)){
    template <>
    GPS_Solver<FloatT>::base_t::relative_property_t
        GPS_Solver<FloatT>::relative_property(
          const GPS_Solver<FloatT>::base_t::prn_t &prn,
          const GPS_Solver<FloatT>::base_t::measurement_t::mapped_type &measurement,
          const GPS_Solver<FloatT>::base_t::float_t &receiver_error,
          const GPS_Solver<FloatT>::base_t::gps_time_t &time_arrival,
          const GPS_Solver<FloatT>::base_t::pos_t &usr_pos,
          const GPS_Solver<FloatT>::base_t::xyz_t &usr_vel) const {
      union {
        base_t::relative_property_t prop;
        FloatT values[7];
      } res = {
          select_solver(prn).relative_property(
            prn, measurement, receiver_error, time_arrival,
            usr_pos, usr_vel)};
#ifdef SWIGRUBY
      do{
        static const VALUE key(ID2SYM(rb_intern("relative_property")));
        static const int prop_items(sizeof(res.values) / sizeof(res.values[0]));
        VALUE hook(rb_hash_lookup(hooks, key));
        if(NIL_P(hook)){break;}
        VALUE values[] = {
            SWIG_From(int)(prn), // prn
            rb_ary_new_from_args(prop_items, // relative_property
              swig::from(res.prop.weight),
              swig::from(res.prop.range_corrected),
              swig::from(res.prop.range_residual),
              swig::from(res.prop.rate_relative_neg),
              swig::from(res.prop.los_neg[0]),
              swig::from(res.prop.los_neg[1]),
              swig::from(res.prop.los_neg[2])),
            swig::from(measurement), // measurement => Hash
            swig::from(receiver_error), // receiver_error
            SWIG_NewPointerObj( // time_arrival
              new base_t::gps_time_t(time_arrival), $descriptor(GPS_Time<FloatT> *), SWIG_POINTER_OWN),
            SWIG_NewPointerObj( // usr_pos.xyz
              new base_t::xyz_t(usr_pos.xyz), $descriptor(System_XYZ<FloatT, WGS84> *), SWIG_POINTER_OWN),
            SWIG_NewPointerObj( // usr_vel
              new base_t::xyz_t(usr_vel), $descriptor(System_XYZ<FloatT, WGS84> *), SWIG_POINTER_OWN)};
        VALUE res_hook(proc_call_throw_if_error(hook, sizeof(values) / sizeof(values[0]), values));
        if((!RB_TYPE_P(res_hook, T_ARRAY))
            || (RARRAY_LEN(res_hook) != prop_items)){
          throw std::runtime_error(
              std::string("[d * ").append(std::to_string(prop_items))
                .append("] is expected (d: " %str(FloatT) "), however ")
                .append(inspect_str(res_hook)));
        }
        for(int i(0); i < prop_items; ++i){
          VALUE v(RARRAY_AREF(res_hook, i));
          if(!SWIG_IsOK(swig::asval(v, &res.values[i]))){
            throw std::runtime_error(
                std::string(%str(FloatT) " is exepcted, however ")
                  .append(inspect_str(v))
                  .append(" @ [").append(std::to_string(i)).append("]"));
          }
        }
      }while(false);
#endif
      return res.prop;
    }
    template <>
    bool GPS_Solver<FloatT>::update_position_solution(
        const GPS_Solver<FloatT>::base_t::geometric_matrices_t &geomat,
        GPS_Solver<FloatT>::base_t::user_pvt_t &res) const {
#ifdef SWIGRUBY
      do{
        static const VALUE key(ID2SYM(rb_intern("update_position_solution")));
        VALUE hook(rb_hash_lookup(hooks, key));
        if(NIL_P(hook)){break;}
        base_t::geometric_matrices_t &geomat_(
            %const_cast(geomat, base_t::geometric_matrices_t &));
        VALUE values[] = {
            SWIG_NewPointerObj(&geomat_.G,
              $descriptor(Matrix<FloatT, Array2D_Dense<FloatT>, MatrixViewBase<> > *), 0),
            SWIG_NewPointerObj(&geomat_.W,
              $descriptor(Matrix<FloatT, Array2D_Dense<FloatT>, MatrixViewBase<> > *), 0),
            SWIG_NewPointerObj(&geomat_.delta_r,
              $descriptor(Matrix<FloatT, Array2D_Dense<FloatT>, MatrixViewBase<> > *), 0),
            SWIG_NewPointerObj(&res,
              $descriptor(GPS_User_PVT<FloatT> *), 0)};
        proc_call_throw_if_error(hook, sizeof(values) / sizeof(values[0]), values);
      }while(false);
#endif
      return super_t::update_position_solution(geomat, res);
    }
    template <>
    GPS_Solver<FloatT>::base_t::satellite_t GPS_Solver<FloatT>::select_satellite(
        const GPS_Solver<FloatT>::base_t::prn_t &prn,
        const GPS_Solver<FloatT>::base_t::gps_time_t &time) const {
      GPS_Solver<FloatT>::base_t::satellite_t res(
          select_solver(prn).select_satellite(prn, time));
#ifdef SWIGRUBY
      if(!res.is_available()){
        static const VALUE key(ID2SYM(rb_intern("relative_property")));
        VALUE hook(rb_hash_lookup(hooks, key));
        if(!NIL_P(hook)){
          if(!res.impl_xyz){res.impl_xyz = this;}
          if(!res.impl_t){res.impl_t = this;}
        }
      }
#endif
      return res;
    }
  }
  %fragment("hook"{GPS_Solver<FloatT>});
  %ignore update_correction;
#ifdef SWIGRUBY
  %fragment("correction"{GPS_Solver<FloatT>}, "header",
      fragment=SWIG_From_frag(int),
      fragment=SWIG_Traits_frag(FloatT)){
    template <>
    bool GPS_RangeCorrector<FloatT>::is_available(
        const typename GPS_Solver_Base<FloatT>::gps_time_t &t) const {
      VALUE values[] = {
        SWIG_NewPointerObj(
            %const_cast(&t, GPS_Time<FloatT> *), $descriptor(GPS_Time<FloatT> *), 0),
      };
      VALUE res(proc_call_throw_if_error(
          callback, sizeof(values) / sizeof(values[0]), values));
      return RTEST(res) ? true : false;
    }
    template <>
    FloatT *GPS_RangeCorrector<FloatT>::calculate(
        const typename GPS_Solver_Base<FloatT>::gps_time_t &t,
        const typename GPS_Solver_Base<FloatT>::pos_t &usr_pos, 
        const typename GPS_Solver_Base<FloatT>::enu_t &sat_rel_pos,
        FloatT &buf) const {
      VALUE values[] = {
        SWIG_NewPointerObj(
            %const_cast(&t, GPS_Time<FloatT> *),
            $descriptor(GPS_Time<FloatT> *), 0),
        SWIG_NewPointerObj(
            (%const_cast(&usr_pos.xyz, System_XYZ<FloatT, WGS84> *)),
            $descriptor(System_XYZ<FloatT, WGS84> *), 0),
        SWIG_NewPointerObj(
            (%const_cast(&sat_rel_pos, System_ENU<FloatT, WGS84> *)),
            $descriptor(System_ENU<FloatT, WGS84> *), 0),
      };
      VALUE res(proc_call_throw_if_error(
          callback, sizeof(values) / sizeof(values[0]), values));
      return SWIG_IsOK(swig::asval(res, &buf)) ? &buf : NULL;
    }
    template<>
    VALUE GPS_Solver<FloatT>::update_correction(
        const bool &update, const VALUE &hash){
      typedef range_correction_list_t list_t;
      static const VALUE k_root[] = {
        ID2SYM(rb_intern("gps_ionospheric")),
        ID2SYM(rb_intern("gps_tropospheric")),
        ID2SYM(rb_intern("glonass_ionospheric")),
        ID2SYM(rb_intern("glonass_tropospheric")),
      };
      static const VALUE k_opt(ID2SYM(rb_intern("options")));
      static const VALUE k_f_10_7(ID2SYM(rb_intern("f_10_7")));
      static const VALUE k_known(ID2SYM(rb_intern("known")));
      struct {
        VALUE sym;
        list_t::mapped_type::value_type obj;
      } item[] = {
        {ID2SYM(rb_intern("no_correction")), &base_t::no_correction},
        {ID2SYM(rb_intern("klobuchar")), &this->gps.solver.ionospheric_klobuchar},
        {ID2SYM(rb_intern("ntcm_gl")), &this->gps.solver.ionospheric_ntcm_gl},
        {ID2SYM(rb_intern("hopfield")), &this->gps.solver.tropospheric_simplified},
      };
      list_t input;
      if(update){
        if(!RB_TYPE_P(hash, T_HASH)){
          throw std::runtime_error(
              std::string("Hash is expected, however ").append(inspect_str(hash)));
        }
        for(std::size_t i(0); i < sizeof(k_root) / sizeof(k_root[0]); ++i){
          VALUE ary = rb_hash_lookup(hash, k_root[i]);
          if(NIL_P(ary)){continue;}
          if(!RB_TYPE_P(ary, T_ARRAY)){
            ary = rb_ary_new_from_values(1, &ary);
          }
          for(int j(0); j < RARRAY_LEN(ary); ++j){
            std::size_t k(0);
            VALUE v(rb_ary_entry(ary, j));
            for(; k < sizeof(item) / sizeof(item[0]); ++k){
              if(v == item[k].sym){break;}
            }
            if(k >= sizeof(item) / sizeof(item[0])){ // other than symbol
              user_correctors.push_back(GPS_RangeCorrector<FloatT>(v));
              input[i].push_back(&user_correctors.back());
            }else{
              input[i].push_back(item[k].obj);
            }
          }
        }
        VALUE opt(rb_hash_lookup(hash, k_opt));
        if(RB_TYPE_P(opt, T_HASH)){
          swig::asval(rb_hash_lookup(opt, k_f_10_7), // ntcm_gl
              &this->gps.solver.ionospheric_ntcm_gl.f_10_7);
        }
      }
      list_t output(update_correction(update, input));
      VALUE res = rb_hash_new();
      for(list_t::const_iterator it(output.begin()), it_end(output.end());
          it != it_end; ++it){
        VALUE k;
        if((it->first < 0) || (it->first >= (int)(sizeof(k_root) / sizeof(k_root[0])))){
          k = SWIG_From(int)(it->first);
        }else{
          k = k_root[it->first];
        }
        VALUE v = rb_ary_new();
        for(list_t::mapped_type::const_iterator
              it2(it->second.begin()), it2_end(it->second.end());
            it2 != it2_end; ++it2){
          std::size_t i(0);
          for(; i < sizeof(item) / sizeof(item[0]); ++i){
            if(*it2 == item[i].obj){break;}
          }
          if(i >= sizeof(item) / sizeof(item[0])){ // other than built-in corrector
            rb_ary_push(v, 
                reinterpret_cast<const GPS_RangeCorrector<FloatT> *>(*it2)->callback);
          }else{
            rb_ary_push(v, item[i].sym);
          }
        }
        rb_hash_aset(res, k, v);
      }
      { // common options
        VALUE opt = rb_hash_new();
        rb_hash_aset(res, k_opt, opt);
        rb_hash_aset(opt, k_f_10_7, // ntcm_gl 
            swig::from(this->gps.solver.ionospheric_ntcm_gl.f_10_7));
      }
      { // known models
        VALUE ary = rb_ary_new_capa((int)(sizeof(item) / sizeof(item[0])));
        for(std::size_t i(0); i < sizeof(item) / sizeof(item[0]); ++i){
          rb_ary_push(ary, item[i].sym);
        }
        rb_hash_aset(res, k_known, ary);
      }
      return res;
    }
  }
  %fragment("correction"{GPS_Solver<FloatT>});
  %rename("correction") get_correction;
  %rename("correction=") set_correction;
  VALUE get_correction() const {
    return const_cast<GPS_Solver<FloatT> *>(self)->update_correction(false, Qnil);
  }
  VALUE set_correction(VALUE hash){
    return self->update_correction(true, hash);
  }
#endif
#ifdef SWIGRUBY
  %typemap(out) typename super_t::options_t {
    VALUE res(rb_hash_new());
    rb_hash_aset(res, ID2SYM(rb_intern("skip_exclusion")), SWIG_From(bool)($1.skip_exclusion));
    %set_output(res);
  }
#endif
  %rename("options") get_options;
  typename super_t::options_t get_options() const {
    return self->available_options();
  }
  %rename("options=") set_options;
  typename super_t::options_t set_options(SWIG_Object obj) {
    GPS_Solver<FloatT>::super_t::options_t opt(self->available_options());
#ifdef SWIGRUBY
    if(!RB_TYPE_P(obj, T_HASH)){SWIG_exception(SWIG_TypeError, "Hash is expected");}
    SWIG_AsVal(bool)(
        rb_hash_lookup(obj, ID2SYM(rb_intern("skip_exclusion"))),
        &opt.skip_exclusion);
#endif
    return self->update_options(opt);
  }
}
%inline {
template <class FloatT>
struct GPS_Solver 
    : public GPS_Solver_RAIM_LSR<FloatT, 
        GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> > > {
  typedef GPS_Solver_RAIM_LSR<FloatT, 
      GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> > > super_t;
  typedef GPS_Solver_Base<FloatT> base_t;
  struct gps_t {
    GPS_SpaceNode<FloatT> space_node;
    GPS_SolverOptions<FloatT> options;
    GPS_SinglePositioning<FloatT> solver;
    gps_t() : space_node(), options(), solver(space_node) {}
  } gps;
  struct glonass_t {
    GLONASS_SpaceNode<FloatT> space_node;
    GLONASS_SolverOptions<FloatT> options;
    GLONASS_SinglePositioning<FloatT> solver;
    glonass_t() : space_node(), options(), solver(space_node) {}
  } glonass;
  SWIG_Object hooks;
  typedef std::vector<GPS_RangeCorrector<FloatT> > user_correctors_t;
  user_correctors_t user_correctors;
#ifdef SWIGRUBY
  static void mark(void *ptr){
    GPS_Solver<FloatT> *solver = (GPS_Solver<FloatT> *)ptr;
    rb_gc_mark(solver->hooks);
    for(typename user_correctors_t::const_iterator 
          it(solver->user_correctors.begin()), it_end(solver->user_correctors.end());
        it != it_end; ++it){
      rb_gc_mark(it->callback);
    }
  }
#endif
  GPS_Solver() : super_t(),
      gps(), glonass(),
      hooks(), user_correctors() {
#ifdef SWIGRUBY
    hooks = rb_hash_new();
#endif
    glonass.solver.ionospheric_correction.add(gps.solver.ionospheric_correction);
    glonass.solver.tropospheric_correction.add(gps.solver.tropospheric_correction);
  }
  GPS_SpaceNode<FloatT> &gps_space_node() {return gps.space_node;}
  GPS_SolverOptions<FloatT> &gps_options() {return gps.options;}
  GLONASS_SpaceNode<FloatT> &glonass_space_node() {return glonass.space_node;}
  GLONASS_SolverOptions<FloatT> &glonass_options() {return glonass.options;}
  const base_t &select_solver(
      const typename base_t::prn_t &prn) const {
    if(prn > 0 && prn <= 32){return gps.solver;}
    if(prn > 0x100 && prn <= (0x100 + 24)){return glonass.solver;}
    // call order: base_t::solve => this returned by select() 
    //     => relative_property() => select_solver()
    // For not supported satellite, call loop prevention is required.
    static const base_t dummy; 
    return dummy;
  }
  virtual typename base_t::relative_property_t relative_property(
      const typename base_t::prn_t &prn,
      const typename base_t::measurement_t::mapped_type &measurement,
      const typename base_t::float_t &receiver_error,
      const typename base_t::gps_time_t &time_arrival,
      const typename base_t::pos_t &usr_pos,
      const typename base_t::xyz_t &usr_vel) const;
  virtual typename base_t::satellite_t select_satellite(
      const typename base_t::prn_t &prn,
      const typename base_t::gps_time_t &time) const;
  virtual bool update_position_solution(
      const typename base_t::geometric_matrices_t &geomat,
      typename base_t::user_pvt_t &res) const;
  GPS_User_PVT<FloatT> solve(
      const GPS_Measurement<FloatT> &measurement,
      const GPS_Time<FloatT> &receiver_time) const {
    const_cast<gps_t &>(gps).space_node.update_all_ephemeris(receiver_time);
    const_cast<gps_t &>(gps).solver.update_options(gps.options);
    const_cast<glonass_t &>(glonass).space_node.update_all_ephemeris(receiver_time);
    const_cast<glonass_t &>(glonass).solver.update_options(glonass.options);
    return super_t::solve().user_pvt(measurement.items, receiver_time);
  }
  typedef 
      std::map<int, std::vector<const typename base_t::range_corrector_t *> >
      range_correction_list_t;
  range_correction_list_t update_correction(
      const bool &update,
      const range_correction_list_t &list = range_correction_list_t()){
    range_correction_list_t res;
    typename base_t::range_correction_t *root[] = {
      &gps.solver.ionospheric_correction,
      &gps.solver.tropospheric_correction,
      &glonass.solver.ionospheric_correction,
      &glonass.solver.tropospheric_correction,
    };
    for(std::size_t i(0); i < sizeof(root) / sizeof(root[0]); ++i){
      do{
        if(!update){break;}
        typename range_correction_list_t::const_iterator it(list.find(i));
        if(it == list.end()){break;}

        // Remove user defined unused correctors
        for(typename base_t::range_correction_t::const_iterator
              it2(root[i]->begin()), it2_end(root[i]->end());
            it2 != it2_end; ++it2){
          for(typename user_correctors_t::const_iterator
                it3(user_correctors.begin()), it3_end(user_correctors.end());
              it3 != it3_end; ++it3){
            if(*it2 != &(*it3)){continue;}
            user_correctors.erase(it3);
          }
        }

        root[i]->clear();
        for(typename range_correction_list_t::mapped_type::const_iterator
              it2(it->second.begin()), it2_end(it->second.end());
            it2 != it2_end; ++it2){
          root[i]->push_back(*it2);
        }
      }while(false);
      for(typename base_t::range_correction_t::const_iterator
            it(root[i]->begin()), it_end(root[i]->end());
          it != it_end; ++it){
        res[i].push_back(*it);
      }
    }
    return res;
  }
  SWIG_Object update_correction(const bool &update, const SWIG_Object &hash);
};
}

%fragment(SWIG_From_frag(int));
%fragment(SWIG_From_frag(bool));
%fragment(SWIG_From_frag(char));

%extend RINEX_Observation {
  %exception read {
#ifdef SWIGRUBY
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_intern("read")), argc, argv);
    }
#endif
    try {
      $action
    } catch (const native_exception &e) {
      e.regenerate();
      SWIG_fail;
    } catch (const std::exception& e) {
      SWIG_exception_fail(SWIG_RuntimeError, e.what());
    }
  }
  %fragment(SWIG_Traits_frag(FloatT));
  static void read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    struct reader_t : public RINEX_OBS_Reader<FloatT> {
      typedef RINEX_OBS_Reader<FloatT> super_t;
      SWIG_Object header;
      SWIG_Object obs_types;
      reader_t(std::istream &in) : RINEX_OBS_Reader<FloatT>(in) {
#ifdef SWIGRUBY
        { // header
          header = rb_hash_new();
          typedef typename super_t::header_t header_t;
          for(typename header_t::const_iterator
              it(super_t::header().begin()), it_end(super_t::header().end());
              it != it_end; ++it){
            SWIG_Object types_per_key(rb_ary_new_capa(it->second.size()));
            for(typename header_t::mapped_type::const_iterator
                it2(it->second.begin()), it2_end(it->second.end());
                it2 != it2_end; ++it2){
              rb_ary_push(types_per_key, rb_str_new_cstr(it2->c_str()));
            }
            rb_hash_aset(header, rb_str_new_cstr(it->first.c_str()), types_per_key);
          }
        }
        { // observation types
          obs_types = rb_hash_new();
          typedef typename super_t::obs_types_t types_t;
          for(typename types_t::const_iterator
              it(super_t::obs_types.begin()), it_end(super_t::obs_types.end());
              it != it_end; ++it){
            SWIG_Object types_per_sys(rb_ary_new_capa(it->second.size()));
            for(typename types_t::mapped_type::const_iterator
                it2(it->second.begin()), it2_end(it->second.end());
                it2 != it2_end; ++it2){
              rb_ary_push(types_per_sys, rb_str_new_cstr(it2->c_str()));
            }
            rb_hash_aset(obs_types, SWIG_From(char)(it->first), types_per_sys);
          }
        }
#endif
      }
    } reader(fin);
    while(reader.has_next()){
      typedef typename reader_t::observation_t obs_t;
      obs_t obs(reader.next());
#ifdef SWIGRUBY
      SWIG_Object res(rb_hash_new());
      static const SWIG_Object
          sym_header(ID2SYM(rb_intern("header"))),
          sym_t(ID2SYM(rb_intern("time"))),
          sym_clke(ID2SYM(rb_intern("rcv_clock_error"))),
          sym_meas(ID2SYM(rb_intern("meas"))),
          sym_meas_types(ID2SYM(rb_intern("meas_types")));
      rb_hash_aset(res, sym_header, reader.header);
      rb_hash_aset(res, sym_t, 
          SWIG_NewPointerObj(
            new GPS_Time<FloatT>(obs.t_epoch), 
            $descriptor(GPS_Time<FloatT> *), SWIG_POINTER_OWN));
      rb_hash_aset(res, sym_clke, swig::from(obs.receiver_clock_error));
      SWIG_Object meas(rb_hash_new());
      for(typename obs_t::per_satellite_t::const_iterator 
          it(obs.per_satellite.begin()), it_end(obs.per_satellite.end());
          it != it_end; ++it){
        SWIG_Object meas_per_sat(rb_ary_new_capa(it->second.size()));
        int i(0);
        for(typename obs_t::per_satellite_t::mapped_type::const_iterator 
            it2(it->second.begin()), it2_end(it->second.end());
            it2 != it2_end; ++it2, ++i){
          if(!it2->valid){continue;}
          rb_ary_store(meas_per_sat, i,
              rb_ary_new_from_args(3,
                swig::from(it2->value),
                SWIG_From(int)(it2->lli),
                SWIG_From(int)(it2->ss)));
        }
        int offset;
        char sys_c(reader_t::serial2sys(it->first, offset));
        rb_hash_aset(meas, 
            rb_ary_new_from_args(2,
              SWIG_From(char)(sys_c),
              SWIG_From(int)(offset)),
            meas_per_sat);
      }
      rb_hash_aset(res, sym_meas, meas);
      rb_hash_aset(res, sym_meas_types, reader.obs_types);
      yield_throw_if_error(1, &res);
#endif
    }
  }
}

%inline {
template <class FloatT>
struct RINEX_Observation {};
}

%inline {
struct PushableData {
  enum system_t {
    SYS_GPS,
    SYS_SBAS,
    SYS_QZSS,
    SYS_GLONASS,
    SYS_GALILEO,
    SYS_BEIDOU,
    SYS_SYSTEMS,
  };
  template <class DataT, class FloatT>
  static bool push(DataT &data, GPS_Solver<FloatT> &solver, const system_t &sys){
    switch(sys){
      case SYS_GPS:
        return data.push(
            solver.gps.solver.satellites, DataT::SYSTEM_GPS);
      case SYS_SBAS:
      case SYS_QZSS:
        break;
      case SYS_GLONASS:
        return data.push(
            solver.glonass.solver.satellites, DataT::SYSTEM_GLONASS);
      case SYS_GALILEO:
      case SYS_BEIDOU:
      default:
        break;
    }
    return false;
  }
  template <class DataT, class FloatT>
  static bool push(DataT &data, GPS_Solver<FloatT> &solver){
    system_t target[] = {
      SYS_GPS,
      //SYS_SBAS,
      //SYS_QZSS,
      SYS_GLONASS,
      //SYS_GALILEO,
      //SYS_BEIDOU,
    };
    for(std::size_t i(0); i < sizeof(target) / sizeof(target[0]); ++i){
      if(!push(data, solver, target[i])){return false;}
    }
    return true;
  }
};
}

%extend SP3 {
  %typemap(out) typename SP3_Product<FloatT>::satellite_count_t {
    %append_output(SWIG_From(int)($1.gps));
    %append_output(SWIG_From(int)($1.sbas));
    %append_output(SWIG_From(int)($1.qzss));
    %append_output(SWIG_From(int)($1.glonass));
    %append_output(SWIG_From(int)($1.galileo));
    %append_output(SWIG_From(int)($1.beidou));
  }
}
%inline {
template <class FloatT>
struct SP3 : public SP3_Product<FloatT>, PushableData {
  int read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    return SP3_Reader<FloatT>::read_all(fin, *this);
  }
  typename SP3_Product<FloatT>::satellite_count_t satellites() const {
    return SP3_Product<FloatT>::satellite_count();
  }
  bool push(GPS_Solver<FloatT> &solver, const PushableData::system_t &sys) const {
    return PushableData::push((SP3_Product<FloatT> &)*this, solver, sys);
  }
  bool push(GPS_Solver<FloatT> &solver) const {
    return PushableData::push((SP3_Product<FloatT> &)*this, solver);
  }
  System_XYZ<FloatT, WGS84> position(
      const int &sat_id, const GPS_Time<FloatT> &t) const {
    return SP3_Product<FloatT>::select(sat_id, t).position(t);
  }
  System_XYZ<FloatT, WGS84> velocity(
      const int &sat_id, const GPS_Time<FloatT> &t) const {
    return SP3_Product<FloatT>::select(sat_id, t).velocity(t);
  }
  FloatT clock_error(
      const int &sat_id, const GPS_Time<FloatT> &t) const {
    return SP3_Product<FloatT>::select(sat_id, t).clock_error(t);
  }
  FloatT clock_error_dot(
      const int &sat_id, const GPS_Time<FloatT> &t) const {
    return SP3_Product<FloatT>::select(sat_id, t).clock_error_dot(t);
  }
  int apply_antex(const char *fname) {
    ANTEX_Product<FloatT> antex;
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    int read_items(ANTEX_Reader<FloatT>::read_all(fin, antex));
    if(read_items < 0){return read_items;}
    return antex.move_to_antenna_position(*this);
  }
};
}

%extend RINEX_Clock {
  %typemap(out) typename RINEX_CLK<FloatT>::satellites_t::count_t {
    %append_output(SWIG_From(int)($1.gps));
    %append_output(SWIG_From(int)($1.sbas));
    %append_output(SWIG_From(int)($1.qzss));
    %append_output(SWIG_From(int)($1.glonass));
    %append_output(SWIG_From(int)($1.galileo));
    %append_output(SWIG_From(int)($1.beidou));
  }
}
%inline {
template <class FloatT>
struct RINEX_Clock : public RINEX_CLK<FloatT>::satellites_t, PushableData {
  typedef typename RINEX_CLK<FloatT>::satellites_t super_t;
  int read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    return RINEX_CLK_Reader<FloatT>::read_all(fin, *this);
  }
  typename RINEX_CLK<FloatT>::satellites_t::count_t satellites() const {
    return RINEX_CLK<FloatT>::satellites_t::count();
  }
  bool push(GPS_Solver<FloatT> &solver, const PushableData::system_t &sys) const {
    return PushableData::push((typename RINEX_CLK<FloatT>::satellites_t &)*this, solver, sys);
  }
  bool push(GPS_Solver<FloatT> &solver) const {
    return PushableData::push((typename RINEX_CLK<FloatT>::satellites_t &)*this, solver);
  }
  FloatT clock_error(const int &sat_id, const GPS_Time<FloatT> &t) const {
    typename super_t::buf_t::const_iterator it(this->buf.find(sat_id));
    if(it == this->buf.end()){return super_t::sat_t::unavailable().clock_error(t);}
    return it->second.clock_error(t);
  }
  FloatT clock_error_dot(const int &sat_id, const GPS_Time<FloatT> &t) const {
    typename super_t::buf_t::const_iterator it(this->buf.find(sat_id));
    if(it == this->buf.end()){return super_t::sat_t::unavailable().clock_error(t);}
    return it->second.clock_error_dot(t);
  }
};
}

#undef MAKE_ACCESSOR
#undef MAKE_VECTOR2ARRAY
#undef MAKE_ARRAY_INPUT

%define CONCRETIZE(type)
%template(Time) GPS_Time<type>;
%template(SpaceNode) GPS_SpaceNode<type>;
%template(Ionospheric_UTC_Parameters) GPS_Ionospheric_UTC_Parameters<type>;
%template(Ephemeris) GPS_Ephemeris<type>;
%template(PVT) GPS_User_PVT<type>;
%template(Measurement) GPS_Measurement<type>;
%template(SolverOptionsCommon) GPS_SolverOptions_Common<type>;
%template(SolverOptions) GPS_SolverOptions<type>;
#if defined(SWIGRUBY)
%markfunc GPS_Solver<type> "GPS_Solver<type>::mark";
#endif
%template(Solver) GPS_Solver<type>;

%template(SpaceNode_GLONASS) GLONASS_SpaceNode<type>;
%template(Ephemeris_GLONASS) GLONASS_Ephemeris<type>;
%template(SolverOptions_GLONASS) GLONASS_SolverOptions<type>;

%template(RINEX_Observation) RINEX_Observation<type>;
%template(SP3) SP3<type>;
%template(RINEX_Clock) RINEX_Clock<type>;
%enddef

CONCRETIZE(double);

#undef CONCRETIZE
