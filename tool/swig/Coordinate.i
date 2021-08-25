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

%extend System_3D {
  std::string __str__() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
  %typemap(in,numinputs=0) FloatT values[3] (FloatT temp[3]) %{
    $1 = temp;
  %}
  %typemap(argout) FloatT values[3] {
    for(int i(0); i < 3; ++i){
      %append_output(SWIG_From(double)((double)$1[i]));
    }
  }
  void to_a(FloatT values[3]) const {
    for(int i(0); i < 3; ++i){
      values[i] = (*self)[i];
    }
  }
};
%extend System_LLH {
  std::string __str__() const {
    std::stringstream s;
    s << (*self);
    return s.str();
  }
  %typemap(in,numinputs=0) FloatT (&res)[3][3] {
    $1 = &($1_type)(*(new FloatT[9]));
  }
  %typemap(freearg) FloatT (&res)[3][3] {
    delete [] &(FloatT (&)[])(*$1);
  }
#ifdef SWIGRUBY
  %typemap(argout) FloatT (&res)[3][3] {
    $result = rb_ary_new_capa(3);
    for(int i(0); i < 3; ++i){
      rb_ary_store($result, i,
          rb_ary_new_from_args(3,
            SWIG_From(double)((double)((*$1)[i][0])), 
            SWIG_From(double)((double)((*$1)[i][1])),
            SWIG_From(double)((double)((*$1)[i][2])) ));
    }
  }
#endif
}

%include navigation/coordinate.h

%define CONCRETIZE(type)
%template(Base) System_3D<type>;
%template(XYZ) System_XYZ<type, WGS84>;
%template(LLH) System_LLH<type, WGS84>;
%template(ENU) System_ENU<type, WGS84>;
%enddef

CONCRETIZE(double);

#undef CONCRETIZE