%module Coordinate

%{
#include <sstream>
#include <string>

#if defined(SWIGRUBY) && defined(isfinite)
#undef isfinite_
#undef isfinite
#endif

#include "navigation/coordinate.h"
%}

%include std_string.i
%include exception.i

//%import "SylphideMath.i"

%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

%feature("autodoc", "1");

%define MAKE_SETTER(name, type)
%rename(%str(name ## =)) set_ ## name;
type set_ ## name (const type &v) {
  return self->name() = v;
}
%enddef

%extend System_3D {
  std::string __str__() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
  %typemap(out) FloatT & {
    $result = SWIG_From(double)(*$1);
  }
};

%extend System_3D {
  %fragment(SWIG_From_frag(System_3D), "header", fragment=SWIG_From_frag(double)){
    static SWIG_Object system3d_c2target(const double &v){
      return SWIG_From(double)(v);
    }
  }
  %fragment(SWIG_From_frag(System_3D));
#if !defined(SWIGRUBY)
  %typemap(in,numinputs=0) FloatT values[3] (FloatT temp[3]) %{
    $1 = temp;
  %}
  %typemap(argout) FloatT values[3] {
    for(int i(0); i < 3; ++i){
      %append_output(system3d_c2target($1[i]));
    }
  }
  void to_a(FloatT values[3]) const {
    for(int i(0); i < 3; ++i){
      values[i] = (*self)[i];
    }
  }
#endif
  %exception each {
#ifdef SWIGRUBY
    if(!rb_block_given_p()){
      return rb_enumeratorize(self, ID2SYM(rb_intern("each")), argc, argv);
    }
#endif
    $action
  }
  void each() const {
    for(int i(0); i < 3; ++i){
#ifdef SWIGRUBY
      rb_yield_values(1, system3d_c2target((*self)[i]));
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
  %typemap(in,numinputs=0) FloatT (&res)[3][3] {
    $1 = &($1_type)(*(new FloatT[9]));
  }
  %typemap(freearg) FloatT (&res)[3][3] {
    delete [] &(FloatT (&)[])(*$1);
  }
#ifdef SWIGRUBY
  %typemap(argout, fragment=SWIG_From_frag(System_3D)) FloatT (&res)[3][3] {
    $result = rb_ary_new_capa(3);
    for(int i(0); i < 3; ++i){
      rb_ary_store($result, i,
          rb_ary_new_from_args(3,
            system3d_c2target((*$1)[i][0]), 
            system3d_c2target((*$1)[i][1]),
            system3d_c2target((*$1)[i][2]) ));
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