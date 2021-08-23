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

#include "navigation/GPS_Solver_Base.h"
#include "navigation/GPS_Solver.h"
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

%extend GPS_User_PVT {
  %ignore base_t;
  %ignore GPS_User_PVT(const base_t &);
  MAKE_VECTOR2ARRAY(int);
#ifdef SWIGRUBY
  %rename("position_solved?") position_solved;
  %rename("velocity_solved?") velocity_solved;
#endif
}
%inline %{
template <class FloatT>
struct GPS_User_PVT : protected GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> >::user_pvt_t {
  typedef typename GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> >::user_pvt_t base_t;
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
  std::vector<int> used_satellites() const {return base_t::used_satellite_mask.indices_one();}
  bool position_solved() const {return base_t::position_solved();}
  bool velocity_solved() const {return base_t::velocity_solved();}
  
  const Matrix_Frozen<FloatT, Array2D_Dense<FloatT> > &G() const {return base_t::G;}
  const Matrix_Frozen<FloatT, Array2D_Dense<FloatT> > &W() const {return base_t::W;}
  const Matrix_Frozen<FloatT, Array2D_Dense<FloatT> > &delta_r() const {return base_t::delta_r;}
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
    $action
  }
  void each() const {
    for(typename GPS_Measurement<FloatT>::items_t::const_iterator
          it(self->items.begin()), it_end(self->items.end());
        it != it_end; ++it){
      for(typename GPS_Measurement<FloatT>::items_t::mapped_type::const_iterator 
            it2(it->second.begin()), it2_end(it->second.end());
          it2 != it2_end; ++it2){
#ifdef SWIGRUBY
        rb_yield_values(3,
            SWIG_From(int)(it->first), 
            SWIG_From(int)(it2->first), SWIG_From(double)((double)it2->second));
#endif
      }
    }
  }
}
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
    for(int i(0), j(0), j_max(models.size()); i < model_t::IONOSPHERIC_MODELS; ++i){
      model_t v(model_t::IONOSPHERIC_SKIP);
      if(j < j_max){
        if((models[j] >= 0) && (models[j] < model_t::IONOSPHERIC_SKIP)){
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
  %ignore base_t;
  %ignore gps_t;
  %ignore gps;
  %ignore select;
  %typemap(in) const std::map<int, std::map<int, FloatT> > &measurement (
      std::map<int, std::map<int, FloatT> > temp) {
    $1 = &temp;
#ifdef SWIGRUBY
    // TODO
#endif
  }
}
%inline %{
template <class FloatT>
struct GPS_Solver : public GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> > {
  typedef GPS_Solver_Base_Debug<FloatT, GPS_Solver_Base<FloatT> > base_t;
  struct gps_t {
    GPS_SpaceNode<FloatT> space_node;
    GPS_SolverOptions<FloatT> options;
    GPS_SinglePositioning<FloatT> solver;
    gps_t() : space_node(), options(), solver(space_node) {}
  } gps;
  GPS_Solver() : base_t(), gps() {}
  GPS_SpaceNode<FloatT> &gps_space_node() {return gps.space_node;}
  GPS_SolverOptions<FloatT> &gps_options() {return gps.options;}
  const GPS_Solver_Base<FloatT> &select(
      const typename GPS_Solver_Base<FloatT>::prn_t &prn) const {
    if(prn > 0 && prn <= 32){return gps.solver;}
    return *this;
  }
  GPS_User_PVT<FloatT> solve(
      const std::map<int, std::map<int, FloatT> > &measurement,
      const GPS_Time<FloatT> &receiver_time) const {
    const_cast<gps_t &>(gps).space_node.update_all_ephemeris(receiver_time);
    const_cast<gps_t &>(gps).solver.update_options(gps.options);
    return base_t::solve().user_pvt(measurement, receiver_time);
  }
  GPS_User_PVT<FloatT> solve(
      const GPS_Measurement<FloatT> &measurement,
      const GPS_Time<FloatT> &receiver_time) const {
    return solve(measurement.items, receiver_time);
  }
};
%}

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
%template(Solver) GPS_Solver<type>;
%enddef

CONCRETIZE(double);

#undef CONCRETIZE