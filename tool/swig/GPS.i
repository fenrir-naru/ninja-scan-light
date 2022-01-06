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
#include "navigation/RINEX.h"

#include "navigation/GPS_Solver_Base.h"
#include "navigation/GPS_Solver.h"
#include "navigation/GPS_Solver_RAIM.h"

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

%inline %{
template <class FloatT>
struct GPS_Ionospheric_UTC_Parameters : public GPS_SpaceNode<FloatT>::Ionospheric_UTC_Parameters {};
%}
%extend GPS_Ionospheric_UTC_Parameters {
  %fragment(SWIG_Traits_frag(FloatT));
  %typemap(in,numinputs=0) FloatT values[4] (FloatT temp[4]) %{
    $1 = temp;
  %}
  %typemap(argout) FloatT values[4] {
    for(int i(0); i < 4; ++i){
      %append_output(swig::from($1[i]));
    }
  }
  %typemap(in) const FloatT values[4] (FloatT temp[4]) {
#ifdef SWIGRUBY
    if(!(RB_TYPE_P($input, T_ARRAY) && (RARRAY_LEN($input) == 4))){
      SWIG_exception(SWIG_TypeError, "array[4] is expected");
    }
    for(int i(0); i < 4; ++i){
      SWIG_Object obj(RARRAY_AREF($input, i));
      if(!SWIG_IsOK(swig::asval(obj, &temp[i]))){
        SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
      }
    }
#endif
    $1 = temp;
  }
  %typemap(in) const unsigned int *buf (unsigned int temp[10]) {
#ifdef SWIGRUBY
    if((!RB_TYPE_P($input, T_ARRAY))
        || (RARRAY_LEN($input) < sizeof(temp) / sizeof(temp[0]))){
      SWIG_exception(SWIG_TypeError, "array is expected, or too short array");
    }
    $1 = temp;
    for(int i(0); i < sizeof(temp) / sizeof(temp[0]); ++i){
      if(!SWIG_IsOK(SWIG_AsVal(unsigned int)(RARRAY_AREF($input, i), &($1[i])))){
        SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
      }
    }
#endif
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
  %typemap(in) const unsigned int *buf (unsigned int temp[10]) {
#ifdef SWIGRUBY
    if((!RB_TYPE_P($input, T_ARRAY))
        || (RARRAY_LEN($input) < sizeof(temp) / sizeof(temp[0]))){
      SWIG_exception(SWIG_TypeError, "array is expected, or too short array");
    }
    $1 = temp;
    for(int i(0); i < sizeof(temp) / sizeof(temp[0]); ++i){
      if(!SWIG_IsOK(SWIG_AsVal(unsigned int)(RARRAY_AREF($input, i), &($1[i])))){
        SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
      }
    }
#endif
  }
  %apply int *OUTPUT { int *subframe_no, int *iodc_or_iode };
  void parse(const unsigned int *buf, int *subframe_no, int *iodc_or_iode){
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
  %ignore iono_correction() const;
  %ignore tropo_correction() const;
  int read(const char *fname) {
    std::fstream fin(fname, std::ios::in | std::ios::binary);
    return RINEX_NAV_Reader<FloatT>::read_all(fin, *self);
  }
}

%include navigation/GPS.h

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
  typename proxy_t::linear_solver_t linear_solver() const {
    return typename proxy_t::linear_solver_t(base_t::G, base_t::W, base_t::delta_r);
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
  Matrix<FloatT, Array2D_Dense<FloatT> > S_enu() const {
    return proxy_t::linear_solver_t::rotate_S(S(), base_t::user_position.ecef2enu());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > slope_HV() const {
    return linear_solver().slope_HV(S());
  }
  Matrix<FloatT, Array2D_Dense<FloatT> > slope_HV_enu() const {
    return linear_solver().slope_HV(S(), base_t::user_position.ecef2enu());
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
#ifdef SWIGRUBY
  VALUE to_hash() const {
    VALUE res(rb_hash_new());
    for(typename GPS_Measurement<FloatT>::items_t::const_iterator
          it(self->items.begin()), it_end(self->items.end());
        it != it_end; ++it){
      VALUE per_sat(rb_hash_new());
      rb_hash_aset(res, SWIG_From(int)(it->first), per_sat);
      for(typename GPS_Measurement<FloatT>::items_t::mapped_type::const_iterator 
            it2(it->second.begin()), it2_end(it->second.end());
          it2 != it2_end; ++it2){
        rb_hash_aset(per_sat, SWIG_From(int)(it2->first), swig::from(it2->second));
      }
    }
    return res;
  }
#endif
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
    }
  }
  %fragment(SWIG_Traits_frag(GPS_Measurement<FloatT>));
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
    ITEMS_PREDEFINED,
  };
  void add(const int &prn, const int &key, const FloatT &value){
    items[prn][key] = value;
  }
};
}

%extend GPS_SolverOptions {
  %ignore base_t;
  MAKE_ACCESSOR(elevation_mask, FloatT);
  MAKE_ACCESSOR(residual_mask, FloatT);
  MAKE_ACCESSOR(f_10_7, FloatT);
  MAKE_VECTOR2ARRAY(int);
#ifdef SWIGRUBY
  %rename("ionospheric_models=") set_ionospheric_models;
  %rename("ionospheric_models") get_ionospheric_models;
#endif
  %typemap(in) const std::vector<int> &models (std::vector<int> temp) {
    $1 = &temp;
#ifdef SWIGRUBY
    if(RB_TYPE_P($input, T_ARRAY)){
      for(int i(0), i_max(RARRAY_LEN($input)); i < i_max; ++i){
        SWIG_Object obj(RARRAY_AREF($input, i));
        int v;
        if(SWIG_IsOK(SWIG_AsVal(int)(obj, &v))){
          temp.push_back(v);
        }else{
          SWIG_exception(SWIG_TypeError, "$*1_ltype is expected");
        }
      }
    }
#endif
  }
}
%inline %{
template <class FloatT>
struct GPS_SolverOptions : public GPS_SinglePositioning<FloatT>::options_t {
  typedef typename GPS_SinglePositioning<FloatT>::options_t base_t;
  void exclude(const int &prn){base_t::exclude_prn.set(prn);}
  void include(const int &prn){base_t::exclude_prn.reset(prn);}
  std::vector<int> excluded() const {return base_t::exclude_prn.excluded();}
  enum {
    IONOSPHERIC_KLOBUCHAR,
    IONOSPHERIC_NTCM_GL,
    IONOSPHERIC_NONE, // which allows no correction
    IONOSPHERIC_MODELS,
    IONOSPHERIC_SKIP = IONOSPHERIC_MODELS, // which means delegating the next slot
  };
  std::vector<int> get_ionospheric_models() const {
    std::vector<int> res;
    for(int i(0); i < base_t::IONOSPHERIC_MODELS; ++i){
      int v((int)(base_t::ionospheric_models[i]));
      if(v == base_t::IONOSPHERIC_SKIP){break;}
      res.push_back(v);
    }
    return res;
  }
  std::vector<int> set_ionospheric_models(const std::vector<int> &models){
    typedef typename base_t::ionospheric_model_t model_t;
    for(int i(0), j(0), j_max(models.size()); i < base_t::IONOSPHERIC_MODELS; ++i){
      model_t v(base_t::IONOSPHERIC_SKIP);
      if(j < j_max){
        if((models[j] >= 0) && (models[j] < base_t::IONOSPHERIC_SKIP)){
          v = (model_t)models[j];
        }
        ++j;
      }
      base_t::ionospheric_models[i] = v;
    }
    return get_ionospheric_models();
  }
};
%}

%extend GPS_Solver {
  %ignore super_t;
  %ignore base_t;
  %ignore gps_t;
  %ignore gps;
  %ignore select_solver;
  %ignore relative_property;
  %ignore satellite_position;
  %ignore update_position_solution;
  %immutable hooks;
  %ignore mark;
  %fragment("hook"{GPS_Solver<FloatT>}, "header",
      fragment=SWIG_From_frag(int),
      fragment=SWIG_Traits_frag(FloatT)){
    template <>
    typename GPS_Solver<FloatT>::base_t::relative_property_t
        GPS_Solver<FloatT>::relative_property(
          const typename GPS_Solver<FloatT>::base_t::prn_t &prn,
          const typename GPS_Solver<FloatT>::base_t::measurement_t::mapped_type &measurement,
          const typename GPS_Solver<FloatT>::base_t::float_t &receiver_error,
          const typename GPS_Solver<FloatT>::base_t::gps_time_t &time_arrival,
          const typename GPS_Solver<FloatT>::base_t::pos_t &usr_pos,
          const typename GPS_Solver<FloatT>::base_t::xyz_t &usr_vel) const {
      union {
        typename base_t::relative_property_t prop;
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
        const typename GPS_Solver<FloatT>::base_t::geometric_matrices_t &geomat,
        typename GPS_Solver<FloatT>::base_t::user_pvt_t &res) const {
#ifdef SWIGRUBY
      do{
        static const VALUE key(ID2SYM(rb_intern("update_position_solution")));
        VALUE hook(rb_hash_lookup(hooks, key));
        if(NIL_P(hook)){break;}
        typename base_t::geometric_matrices_t &geomat_(
            %const_cast(geomat, typename base_t::geometric_matrices_t &));
        VALUE values[] = {
            SWIG_NewPointerObj(&geomat_.G,
              $descriptor(Matrix<FloatT, Array2D_Dense<FloatT>, MatrixViewBase<> > *), 0),
            SWIG_NewPointerObj(&geomat_.W,
              $descriptor(Matrix<FloatT, Array2D_Dense<FloatT>, MatrixViewBase<> > *), 0),
            SWIG_NewPointerObj(&geomat_.delta_r,
              $descriptor(Matrix<FloatT, Array2D_Dense<FloatT>, MatrixViewBase<> > *), 0)};
        proc_call_throw_if_error(hook, sizeof(values) / sizeof(values[0]), values);
      }while(false);
#endif
      return super_t::update_position_solution(geomat, res);
    }
  }
  %fragment("hook"{GPS_Solver<FloatT>});
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
  SWIG_Object hooks;
#ifdef SWIGRUBY
  static void mark(void *ptr){
    GPS_Solver<FloatT> *solver = (GPS_Solver<FloatT> *)ptr;
    if(solver->hooks == Qnil){return;}
    rb_gc_mark(solver->hooks);
  }
#endif
  GPS_Solver() : super_t(), gps(), hooks() {
#ifdef SWIGRUBY
    hooks = rb_hash_new();
#endif
  }
  GPS_SpaceNode<FloatT> &gps_space_node() {return gps.space_node;}
  GPS_SolverOptions<FloatT> &gps_options() {return gps.options;}
  const base_t &select_solver(
      const typename base_t::prn_t &prn) const {
    if(prn > 0 && prn <= 32){return gps.solver;}
    return *this;
  }
  virtual typename base_t::relative_property_t relative_property(
      const typename base_t::prn_t &prn,
      const typename base_t::measurement_t::mapped_type &measurement,
      const typename base_t::float_t &receiver_error,
      const typename base_t::gps_time_t &time_arrival,
      const typename base_t::pos_t &usr_pos,
      const typename base_t::xyz_t &usr_vel) const;
  virtual typename base_t::xyz_t *satellite_position(
      const typename base_t::prn_t &prn,
      const typename base_t::gps_time_t &time,
      typename base_t::xyz_t &res) const {
    return select_solver(prn).satellite_position(prn, time, res);
  }
  virtual bool update_position_solution(
      const typename base_t::geometric_matrices_t &geomat,
      typename base_t::user_pvt_t &res) const;
  GPS_User_PVT<FloatT> solve(
      const GPS_Measurement<FloatT> &measurement,
      const GPS_Time<FloatT> &receiver_time) const {
    const_cast<gps_t &>(gps).space_node.update_all_ephemeris(receiver_time);
    const_cast<gps_t &>(gps).solver.update_options(gps.options);
    return super_t::solve().user_pvt(measurement.items, receiver_time);
  }
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

#undef MAKE_ACCESSOR
#undef MAKE_VECTOR2ARRAY

%define CONCRETIZE(type)
%template(Time) GPS_Time<type>;
%template(SpaceNode) GPS_SpaceNode<type>;
%template(Ionospheric_UTC_Parameters) GPS_Ionospheric_UTC_Parameters<type>;
%template(Ephemeris) GPS_Ephemeris<type>;
%template(PVT) GPS_User_PVT<type>;
%template(Measurement) GPS_Measurement<type>;
%template(SolverOptions) GPS_SolverOptions<type>;
#if defined(SWIGRUBY)
%markfunc GPS_Solver<type> "GPS_Solver<type>::mark";
#endif
%template(Solver) GPS_Solver<type>;

%template(RINEX_Observation) RINEX_Observation<type>;
%enddef

CONCRETIZE(double);

#undef CONCRETIZE