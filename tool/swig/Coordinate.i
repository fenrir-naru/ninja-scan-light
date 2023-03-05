%module Coordinate

%{
#include <sstream>
#include <string>
#include <exception>

#if defined(SWIGRUBY) && defined(isfinite)
#undef isfinite_
#undef isfinite
#endif

#include "navigation/coordinate.h"
%}

%include std_common.i
%include std_string.i
%include exception.i
%include std_except.i

//%import "SylphideMath.i"

%feature("autodoc", "1");

%define MAKE_SETTER(name, type)
%rename(%str(name ## =)) set_ ## name;
type set_ ## name (const type &v) {
  return self->name() = v;
}
%enddef

#if !defined(SWIGIMPORTED)
%exceptionclass native_exception;
%typemap(throws,noblock=1) native_exception {
  $1.regenerate();
  SWIG_fail;
}
%ignore native_exception;
%inline {
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
#endif

%extend System_3D {
  std::string __str__() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
  %fragment(SWIG_Traits_frag(FloatT));
  %typemap(out) FloatT & {
    $result = swig::from(*$1);
  }
#if !defined(SWIGRUBY)
  %typemap(in,numinputs=0) FloatT values[3] (FloatT temp[3]) %{
    $1 = temp;
  %}
  %typemap(argout) FloatT values[3] {
    for(int i(0); i < 3; ++i){
      %append_output(swig::from($1[i]));
    }
  }
  void to_a(FloatT values[3]) const {
    for(int i(0); i < 3; ++i){
      values[i] = (*self)[i];
    }
  }
#endif
  %typemap(in,numinputs=0) const void *check_block {
#ifdef SWIGRUBY
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_frame_callee()), argc, argv);
    }
#endif
  }
  %catches(native_exception) each;
  void each(const void *check_block) const {
    for(int i(0); i < 3; ++i){
#ifdef SWIGRUBY
      int state;
      struct yield_t {
        const VALUE v;
        static VALUE run(VALUE v){
          yield_t *arg(reinterpret_cast<yield_t *>(v));
          return rb_yield_values(1, arg->v);
        }
      } arg = {swig::from((*self)[i])};
      rb_protect(yield_t::run, reinterpret_cast<VALUE>(&arg), &state);
      if(state != 0){throw native_exception(state);}
#endif
    }
  }
};
#ifdef SWIGRUBY
%mixin System_3D "Enumerable";
#endif

%extend System_XYZ {
#ifdef SWIGRUBY
  MAKE_SETTER(x, FloatT);
  MAKE_SETTER(y, FloatT);
  MAKE_SETTER(z, FloatT);
#endif
};

%extend System_LLH {
  %typemap(in,numinputs=0) (FloatT (&res)[3][3]) (FloatT temp[3][3]) {
    $1 = &($1_type)(temp);
  }
#ifdef SWIGRUBY
  %typemap(argout, fragment=SWIG_Traits_frag(FloatT)) FloatT (&res)[3][3] {
    $result = rb_ary_new_capa(3);
    for(int i(0); i < 3; ++i){
      rb_ary_store($result, i,
          rb_ary_new_from_args(3,
            swig::from((*$1)[i][0]), 
            swig::from((*$1)[i][1]),
            swig::from((*$1)[i][2]) ));
    }
  }
  %alias latitude "lat";
  %alias longitude "lng";
  %alias height "h,alt";
  MAKE_SETTER(latitude, FloatT);
  MAKE_SETTER(longitude, FloatT);
  MAKE_SETTER(height, FloatT);
  %alias set_latitude "lat=";
  %alias set_longitude "lng=";
  %alias set_height "h=,alt=";
#endif
}

%extend System_ENU {
  FloatT down() const {
    return self->up() * -1;
  }
  FloatT set_down(const FloatT &v) {
    return (self->up() = (v * -1)) * -1;
  }
#ifdef SWIGRUBY
  %alias east "e";
  %alias north "n";
  %alias up "u"; 
  MAKE_SETTER(east, FloatT);
  MAKE_SETTER(north, FloatT);
  MAKE_SETTER(up, FloatT);
  %alias set_east "e=";
  %alias set_north "n=";
  %alias set_up "u=";
  
  %alias down "d";
  %alias set_down "d=";
#endif
}

#undef MAKE_SETTER

%include navigation/coordinate.h

%define CONCRETIZE(type)
%template(Base) System_3D<type>;
%template(XYZ) System_XYZ<type, WGS84>;
%template(LLH) System_LLH<type, WGS84>;
%template(ENU) System_ENU<type, WGS84>;
%enddef

CONCRETIZE(double);

#undef CONCRETIZE